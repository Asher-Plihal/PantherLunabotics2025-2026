import os
import sys
import socket

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "onboard_software"))
from library.protocol import Connection, MessageType
from robot_params import RobotConfig


class Client(Connection):
    """Laptop-side TCP client that sends commands and receives telemetry."""

    def __init__(self, server_ip, port=RobotConfig.Robot_Port):
        super().__init__()
        self._host = server_ip
        self._port = port

    def _get_role_name(self):
        return "Client"

    def _establish_connection(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self._host, self._port))
        return sock

    # --- Public API (preserves interface used by control.py) ---

    def connect(self):
        """Blocking — run in a thread. Sets self.connected when ready."""
        self._run()

    def send_command(self, data):
        self._send(MessageType.COMMAND, data)

