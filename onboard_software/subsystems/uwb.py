import threading
from typing import Optional


class UWBSensor:
    """
    Reads distance measurements from one ESP32+DWM1000 tag over serial (USB).

    Serial output format (TO BE CONFIRMED by connecting and running:
        screen /dev/ttyUSB1 115200
    Assumed: "DIST,<distance_metres>\\r\\n"  e.g. "DIST,1.523\\r\\n"
    Update _parse() once the actual firmware format is observed.
    """

    DEFAULT_BAUD = 115200
    READ_TIMEOUT = 1.0

    def __init__(self, port: str, baud: int = DEFAULT_BAUD):
        self.port = port
        self.baud = baud
        self._serial = None
        self._running = False
        self._distance: Optional[float] = None
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None

    def connect(self) -> bool:
        """Open the serial port. Returns True on success."""
        if self._serial and self._serial.is_open:
            return True
        import serial
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=self.READ_TIMEOUT)
            return True
        except serial.SerialException as e:
            print(f"[UWB] {self.port}: {e}")
            return False

    def disconnect(self):
        """Stop read thread and close serial port."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=self.READ_TIMEOUT + 1.0)
            self._thread = None
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._serial = None

    def start(self):
        """Begin background read loop. Call connect() first."""
        if not self._serial or not self._serial.is_open:
            raise RuntimeError("Call connect() before start()")
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def get_distance(self) -> Optional[float]:
        """Return the latest distance to anchor in metres, or None if no reading yet."""
        with self._lock:
            return self._distance

    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def shutdown(self):
        """Alias for disconnect — called by robot.py on shutdown."""
        self.disconnect()

    def log_data(self):
        pass  # UWB logging not yet implemented

    def _read_loop(self):
        while self._running:
            try:
                raw = self._serial.readline()
                if raw:
                    d = self._parse(raw.decode("utf-8", errors="ignore"))
                    if d is not None:
                        with self._lock:
                            self._distance = d
            except Exception as e:
                print(f"[UWB] Read error on {self.port}: {e}")
                break
        self._running = False

    @staticmethod
    def _parse(line: str) -> Optional[float]:
        """
        Parse one serial line to a distance in metres.
        *** UPDATE once actual firmware output format is confirmed ***
        """
        line = line.strip()
        if not line.startswith("DIST,"):
            return None
        parts = line.split(",")
        if len(parts) != 2:
            return None
        try:
            d = float(parts[1])
            return d if d >= 0 else None
        except ValueError:
            return None
