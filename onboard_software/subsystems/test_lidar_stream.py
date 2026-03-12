import json
import socket
import time

from rplidar import RPLidar  # type: ignore

# -------------------------------
# Configuration
# -------------------------------
MIN_DISTANCE = 50      # mm
MAX_DISTANCE = 3000    # mm
STREAM_PORT = 5000

# -------------------------------
# Main
# -------------------------------
def main():
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
    time.sleep(1)  # let motor stabilize

    try:
        for scan in lidar.iter_scans():
            # Filter and send only angle + distance pairs
            points = [
                (round(angle, 1), round(distance, 1))
                for _, angle, distance in scan
                if MIN_DISTANCE <= distance <= MAX_DISTANCE
            ]

            # Send as length-prefixed JSON
            data = json.dumps(points).encode()
            header = len(data).to_bytes(4, 'big')
            try:
                conn.sendall(header + data)
            except (BrokenPipeError, ConnectionResetError):
                print("Viewer disconnected.")
                break

    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        conn.close()
        server_sock.close()

if __name__ == "__main__":
    main()
