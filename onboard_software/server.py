import os
import sys
import socket
import robot_params

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from library.protocol import Connection, MessageType


class Server(Connection):
    """Robot-side TCP server that receives commands and sends telemetry."""

    def __init__(self, port=robot_params.RobotConfig.Robot_Port):
        super().__init__()
        self._port = port
        self._server_socket = None

    def _get_role_name(self):
        return "Server"

    def _establish_connection(self):
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind(("0.0.0.0", self._port))
        print(f"[Server] Socket bound to 0.0.0.0:{self._port}")
        self._server_socket.listen(1)
        print("[Server] Waiting for client...")
        client_sock, addr = self._server_socket.accept()
        print(f"[Server] Got connection from {addr}")
        return client_sock

    # --- Public API (preserves interface used by robot.py) ---

    def start(self):
        """Blocking — run in a thread."""
        self._run()

    def send_telemetry(self, data):
        self._send(MessageType.TELEMETRY, data)

    def get_command(self):
        return self._receive()

    def stop(self):
        """Shut down both the client connection and the listening socket."""
        super().stop()
        if self._server_socket:
            try:
                self._server_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self._server_socket.close()
            except OSError:
                pass
