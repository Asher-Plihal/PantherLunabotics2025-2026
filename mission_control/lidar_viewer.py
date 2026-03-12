"""
Lidar stream viewer — run this on your laptop.
Connects to the Pi and displays the radar feed in a local pygame window.

Usage:
    python mission_control/lidar_viewer.py
"""

import io
import socket
import struct
import pygame

STREAM_PORT = 5000
PI_IP = "100.87.109.7"
WIDTH, HEIGHT = 800, 800


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

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return

            # Read 4-byte length header, then the JPEG frame
            header = recv_exact(sock, 4)
            frame_len = struct.unpack('>I', header)[0]
            frame_bytes = recv_exact(sock, frame_len)

            surface = pygame.image.load(io.BytesIO(frame_bytes))
            surface = pygame.transform.scale(surface, (WIDTH, HEIGHT))
            screen.blit(surface, (0, 0))
            pygame.display.flip()
            clock.tick(60)

    except (ConnectionError, KeyboardInterrupt):
        print("\nDisconnected.")
    finally:
        sock.close()
        pygame.quit()


if __name__ == "__main__":
    main()
