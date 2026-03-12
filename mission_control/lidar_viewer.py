"""
Lidar stream viewer — run this on your laptop.
Connects to the Jetson and displays the radar feed in a local pygame window.

Usage:
    python mission_control/lidar_viewer.py
"""

import json
import math
import socket
import struct
import pygame

STREAM_PORT = 5000
PI_IP = "100.87.109.7"
WIDTH, HEIGHT = 600, 600
CENTER = (WIDTH // 2, HEIGHT // 2)
MAX_DISTANCE = 3000    # mm
SCALE = (WIDTH // 2) / MAX_DISTANCE

# Colors
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
RED = (255, 0, 0)
DARK_GREEN = (0, 100, 0)
WHITE = (255, 255, 255)


def recv_exact(sock, n):
    """Read exactly n bytes from sock, or raise ConnectionError."""
    buf = bytearray(n)
    view = memoryview(buf)
    pos = 0
    while pos < n:
        nbytes = sock.recv_into(view[pos:])
        if not nbytes:
            raise ConnectionError("Connection closed by server")
        pos += nbytes
    return buf


def polar_to_cartesian(angle_deg, distance_mm):
    angle_rad = math.radians((angle_deg + 180) % 360)
    r = distance_mm * SCALE
    x = CENTER[0] + int(r * math.cos(angle_rad))
    y = CENTER[1] + int(r * math.sin(angle_rad))
    return x, y


def build_static_overlay():
    """Pre-render reference circles and title onto a surface (drawn once)."""
    overlay = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
    font_small = pygame.font.SysFont(None, 20)
    font_title = pygame.font.SysFont(None, 28, bold=True)
    for r in range(500, MAX_DISTANCE + 1, 500):
        pygame.draw.circle(overlay, DARK_GREEN, CENTER, int(r * SCALE), 1)
        label = font_small.render(f"{r // 10} cm", True, WHITE)
        overlay.blit(label, (CENTER[0] + int(r * SCALE) - 25, CENTER[1]))
    title = font_title.render("RPLidar Radar Map", True, WHITE)
    overlay.blit(title, (WIDTH // 2 - title.get_width() // 2, HEIGHT - 40))
    return overlay


def main():
    print(f"Connecting to {PI_IP}:{STREAM_PORT}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((PI_IP, STREAM_PORT))
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print("Connected. Receiving stream...")

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("RPLidar Remote View")
    clock = pygame.time.Clock()

    static_overlay = build_static_overlay()

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return

            # Read 4-byte length header, then the JSON scan data
            header = recv_exact(sock, 4)
            data_len = struct.unpack('>I', header)[0]
            data = recv_exact(sock, data_len)
            points = json.loads(data)

            # Render locally
            screen.fill(BLACK)
            screen.blit(static_overlay, (0, 0))

            for angle, distance in points:
                px, py = polar_to_cartesian(angle, distance)
                if distance <= 1000:
                    color = RED
                elif distance <= 2000:
                    color = YELLOW
                else:
                    color = GREEN
                pygame.draw.circle(screen, color, (px, py), 2)

            pygame.display.flip()
            clock.tick(60)

    except (ConnectionError, KeyboardInterrupt):
        print("\nDisconnected.")
    finally:
        sock.close()
        pygame.quit()


if __name__ == "__main__":
    main()
