import socket
import struct
import threading
from enum import Enum


class StreamMode(Enum):
    NONE = "none"      # No streaming
    REMOTE = "remote"  # Stream over TCP to a viewer on the laptop
    LOCAL = "local"    # Render locally on the robot (monitor plugged in)


class Stream:
    """Unified streaming infrastructure for robot <-> mission control.

    Source side (robot/perception subsystems):
        stream.start_source(mode, target)  — spawn daemon thread based on mode
        stream.accept_viewer()             — TCP bind/listen/accept
        stream.send_frame(data)            — length-prefixed send
        stream.stop()                      — signal stop, join thread, cleanup

    Viewer side (mission control):
        stream.connect(robot_ip)           — TCP connect to source
        stream.recv_frame()                — length-prefixed receive
        stream.stop()                      — cleanup sockets
    """

    def __init__(self, port: int, name: str):
        self.port = port
        self.name = name
        self._running = False
        self._thread = None
        self._server_sock: socket.socket | None = None
        self._client_conn: socket.socket | None = None
        self._viewer_sock: socket.socket | None = None

    @property
    def running(self) -> bool:
        return self._running

    # === Source side (used by robot/perception subsystems) ===

    def start_source(self, mode, target):
        """Start a daemon thread running target. No-op if mode is NONE."""
        if mode == StreamMode.NONE:
            return
        self._running = True
        self._thread = threading.Thread(target=target, daemon=True)
        self._thread.start()
        print(f"[{self.name}] Started in {mode.value} mode")

    def stop(self):
        """Signal thread to stop, join, and clean up all sockets."""
        if not self._running:
            return
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)
        self._cleanup_sockets()
        print(f"[{self.name}] Stopped")

    def accept_viewer(self):
        """Bind TCP server and block until a viewer connects."""
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.settimeout(1.0)
        self._server_sock.bind(("0.0.0.0", self.port))
        self._server_sock.listen(1)
        print(f"[{self.name}] Waiting for viewer on port {self.port}...")
        while self._running:
            try:
                self._client_conn, addr = self._server_sock.accept()
                self._client_conn.setsockopt(
                    socket.IPPROTO_TCP, socket.TCP_NODELAY, 1
                )
                print(f"[{self.name}] Viewer connected from {addr}")
                return
            except socket.timeout:
                continue
        raise RuntimeError("Stopped before viewer connected")

    def send_frame(self, data: bytes) -> bool:
        """Send a length-prefixed frame to the connected viewer."""
        assert self._client_conn is not None
        try:
            self._client_conn.sendall(len(data).to_bytes(4, "big") + data)
            return True
        except (BrokenPipeError, ConnectionResetError, OSError):
            print(f"[{self.name}] Viewer disconnected")
            return False

    # === Viewer side (used by mission control) ===

    def connect(self, robot_ip: str):
        """Connect to a stream source on the robot."""
        print(f"[{self.name}] Connecting to {robot_ip}:{self.port}...")
        self._viewer_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._viewer_sock.connect((robot_ip, self.port))
        self._viewer_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"[{self.name}] Connected")

    def recv_frame(self) -> bytes:
        """Receive a length-prefixed frame from the source."""
        header = self._recv_exact(4)
        length = struct.unpack(">I", header)[0]
        return bytes(self._recv_exact(length))

    def _recv_exact(self, n: int) -> bytearray:
        assert self._viewer_sock is not None
        buf = bytearray(n)
        view = memoryview(buf)
        pos = 0
        while pos < n:
            nbytes = self._viewer_sock.recv_into(view[pos:])
            if not nbytes:
                raise ConnectionError("Connection closed by server")
            pos += nbytes
        return buf

    # === Cleanup ===

    def _cleanup_sockets(self):
        for s in (self._client_conn, self._server_sock, self._viewer_sock):
            if s:
                try:
                    s.close()
                except OSError:
                    pass
