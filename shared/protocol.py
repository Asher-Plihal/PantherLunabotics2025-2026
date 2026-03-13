import socket
import threading
import json
import queue
import time
from enum import Enum
from abc import ABC, abstractmethod


# --- Enums (str, Enum so they JSON-serialize as strings) ---

class MessageType(str, Enum):
    COMMAND = "command"
    TELEMETRY = "telemetry"
    ACK = "ack"

class Command(str, Enum):
    READY = "READY"
    SHUTDOWN = "SHUTDOWN"

class Mode(str, Enum):
    TELEOP = "TELEOP"
    AUTO = "AUTO"

class Button(str, Enum):
    A = "A"
    B = "B"
    X = "X"
    Y = "Y"
    LB = "LB"
    RB = "RB"
    DPAD_UP = "DPAD_UP"
    DPAD_DOWN = "DPAD_DOWN"
    DPAD_LEFT = "DPAD_LEFT"
    DPAD_RIGHT = "DPAD_RIGHT"

class ButtonAction(str, Enum):
    PRESSED = "PRESSED"
    RELEASED = "RELEASED"

# --- Base Connection Class ---

class Connection(ABC):
    def __init__(self):
        self._input_queue = queue.Queue()
        self._output_queue = queue.Queue()
        self._data_queue = queue.Queue()
        self._running = True
        self._socket = None
        self._pending_acks = {}
        self._ack_lock = threading.Lock()
        self._ack_timeout = 0.5  # seconds
        self._message_id = 0
        self._message_id_lock = threading.Lock()
        self.connected = threading.Event()

    @abstractmethod
    def _establish_connection(self) -> socket.socket:
        """Return a connected socket. Client connects; Server listens and accepts."""
        ...

    @abstractmethod
    def _get_role_name(self) -> str:
        """Return a label for log messages (e.g. 'Client' or 'Server')."""
        ...

    def _run(self):
        """Main entry point: establish connection, start threads, dispatch messages."""
        try:
            self._socket = self._establish_connection()
        except ConnectionRefusedError:
            print(f"[{self._get_role_name()}] Connection refused — is the other side running?")
            return
        except OSError as e:
            print(f"[{self._get_role_name()}] Connection failed: {e}")
            return

        print(f"[{self._get_role_name()}] Connected")
        self.connected.set()

        threading.Thread(target=self._sender_thread, daemon=True).start()
        threading.Thread(target=self._receiver_thread, daemon=True).start()
        threading.Thread(target=self._ack_monitor_thread, daemon=True).start()

        self._dispatch_loop()

    def _dispatch_loop(self):
        """Route incoming messages: data -> _data_queue + ACK, ack -> remove pending."""
        while self._running:
            try:
                msg = self._input_queue.get(timeout=1)
            except queue.Empty:
                continue

            msg_type = msg.get("type")

            if msg_type == MessageType.ACK:
                msg_id = msg.get("id")
                with self._ack_lock:
                    self._pending_acks.pop(msg_id, None)
            elif msg_type in (MessageType.COMMAND, MessageType.TELEMETRY):
                self._data_queue.put(msg)
                ack = {"type": MessageType.ACK, "id": msg.get("id")}
                self._output_queue.put(ack)

    def _receiver_thread(self):
        stream = self._socket.makefile("r", encoding="utf-8")
        try:
            while self._running:
                try:
                    raw = stream.readline()
                except (OSError, socket.error) as e:
                    print(f"[{self._get_role_name()}] Receiver error: {e}")
                    self.stop()
                    break

                if not raw:
                    print(f"[{self._get_role_name()}] Connection closed by remote.")
                    self.stop()
                    break

                raw = raw.strip()
                if not raw:
                    continue

                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError as e:
                    print(f"[{self._get_role_name()}] Invalid JSON received: {e}")
                    continue

                self._input_queue.put(msg)
        finally:
            stream.close()

    def _sender_thread(self):
        stream = self._socket.makefile("w", encoding="utf-8")
        try:
            while self._running:
                try:
                    msg = self._output_queue.get(timeout=1)
                except queue.Empty:
                    continue
                try:
                    json_str = json.dumps(msg) + "\n"
                    stream.write(json_str)
                    stream.flush()
                except (OSError, socket.error) as e:
                    print(f"[{self._get_role_name()}] Sender error: {e}")
                    self.stop()
                    break
        finally:
            stream.close()

    def _ack_monitor_thread(self):
        while self._running:
            time.sleep(0.1)
            with self._ack_lock:
                current_time = time.time()
                to_resend = []
                for msg_id, (msg, sent_time) in list(self._pending_acks.items()):
                    if current_time - sent_time > self._ack_timeout:
                        to_resend.append(msg)
                for msg in to_resend:
                    print(f"[{self._get_role_name()}] Resending unacknowledged: {msg}")
                    self._output_queue.put(msg)
                    self._pending_acks[msg["id"]] = (msg, time.time())

    def _next_message_id(self) -> int:
        with self._message_id_lock:
            self._message_id += 1
            return self._message_id

    def _send(self, msg_type: str, data):
        """Build a message, enqueue it, and register for ACK tracking."""
        msg_id = self._next_message_id()
        msg = {"type": msg_type, "id": msg_id, "data": data}
        with self._ack_lock:
            self._pending_acks[msg_id] = (msg, time.time())
        self._output_queue.put(msg)

    def _receive(self):
        """Non-blocking get from the data queue. Returns the data payload or None."""
        try:
            msg = self._data_queue.get_nowait()
            return msg.get("data")
        except queue.Empty:
            return None

    def stop(self):
        """Gracefully shut down the connection."""
        self._running = False
        if self._socket:
            try:
                self._socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self._socket.close()
            except OSError:
                pass
