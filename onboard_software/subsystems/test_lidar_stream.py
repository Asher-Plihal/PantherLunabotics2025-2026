import io
import math
import os
import socket
import struct
import time

from PIL import Image
from rplidar import RPLidar

# SDL must be configured before importing pygame
os.environ['SDL_VIDEODRIVER'] = 'dummy'
os.environ['SDL_AUDIODRIVER'] = 'dummy'
import pygame

# -------------------------------
# Configuration
# -------------------------------
WIDTH, HEIGHT = 800, 800
CENTER = (WIDTH // 2, HEIGHT // 2)
MIN_DISTANCE = 50      # mm
MAX_DISTANCE = 3000    # mm
SCALE = (WIDTH // 2) / MAX_DISTANCE
STREAM_PORT = 5000

# Colors
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
RED = (255, 0, 0)
DARK_GREEN = (0, 100, 0)
WHITE = (255, 255, 255)

# -------------------------------
# Helper functions
# -------------------------------
def polar_to_cartesian(angle_deg, distance_mm):
    angle_rad = math.radians((angle_deg + 180) % 360)
    r = distance_mm * SCALE
    x = CENTER[0] + int(r * math.cos(angle_rad))
    y = CENTER[1] + int(r * math.sin(angle_rad))
    return x, y

STREAM_WIDTH, STREAM_HEIGHT = WIDTH // 2, HEIGHT // 2

def encode_frame(surface):
    """Encode a pygame surface as JPEG bytes (downscaled for speed)."""
    small = pygame.transform.scale(surface, (STREAM_WIDTH, STREAM_HEIGHT))
    arr = pygame.surfarray.array3d(small)
    arr = arr.transpose(1, 0, 2)  # (w,h,3) -> (h,w,3) for PIL
    img = Image.fromarray(arr, 'RGB')
    buf = io.BytesIO()
    img.save(buf, format='JPEG', quality=50)
    return buf.getvalue()

def send_frame(conn, frame_bytes):
    """Send a length-prefixed frame over a socket."""
    header = struct.pack('>I', len(frame_bytes))
    conn.sendall(header + frame_bytes)

def build_static_overlay(font_small, font_title):
    """Pre-render reference circles and title onto a surface (drawn once)."""
    overlay = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
    for r in range(500, MAX_DISTANCE + 1, 500):
        pygame.draw.circle(overlay, DARK_GREEN, CENTER, int(r * SCALE), 1)
        label = font_small.render(f"{r // 10} cm", True, WHITE)
        overlay.blit(label, (CENTER[0] + int(r * SCALE) - 25, CENTER[1]))
    title = font_title.render("RPLidar Radar Map", True, WHITE)
    overlay.blit(title, (WIDTH // 2 - title.get_width() // 2, HEIGHT - 40))
    return overlay

# -------------------------------
# Main radar function
# -------------------------------
def radar_map():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    font_small = pygame.font.SysFont(None, 20)
    font_title = pygame.font.SysFont(None, 28, bold=True)

    static_overlay = build_static_overlay(font_small, font_title)

    # Set up streaming socket
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(('0.0.0.0', STREAM_PORT))
    server_sock.listen(1)
    print(f"Streaming server listening on port {STREAM_PORT} — waiting for viewer to connect...")
    conn, addr = server_sock.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"Viewer connected from {addr}")

    # Connect to LIDAR
    PORT_NAME = '/dev/ttyUSB0'
    lidar = RPLidar(PORT_NAME)
    time.sleep(2)  # let motor stabilize

    try:
        for scan in lidar.iter_scans():
            screen.fill(BLACK)
            screen.blit(static_overlay, (0, 0))

            # Draw scan points
            for _, angle, distance in scan:
                if MIN_DISTANCE <= distance <= MAX_DISTANCE:
                    px, py = polar_to_cartesian(angle, distance)
                    if distance <= 1000:
                        color = RED
                    elif distance <= 2000:
                        color = YELLOW
                    else:
                        color = GREEN
                    pygame.draw.circle(screen, color, (px, py), 2)

            try:
                send_frame(conn, encode_frame(screen))
            except (BrokenPipeError, ConnectionResetError):
                print("Viewer disconnected.")
                break

    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        pygame.quit()
        conn.close()
        server_sock.close()

# -------------------------------
# Entry point
# -------------------------------
if __name__ == "__main__":
    radar_map()
