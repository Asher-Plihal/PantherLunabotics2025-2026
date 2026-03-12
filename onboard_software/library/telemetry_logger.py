import os
import csv
import time
import datetime


class TelemetryLogger:
    """
    Reusable telemetry file logger for robot subsystems.

    Usage:
        logger = TelemetryLogger("auger")
        logger.start_logging(["Velocity (RPM)", "Current (A)"])
        logger.log_row([120.4, 2.1])
        logger.stop_logging()

    Output is CSV — open the file in any spreadsheet app and each
    value lands in its own cell automatically.
    """

    def __init__(self, name: str, log_dir: str = None):
        """
        name:    subsystem label used in the filename and header (e.g. "auger")
        log_dir: directory to write log files into; defaults to
                 <this file's location>/../../logs/  (i.e. onboard_software/logs/)
        """
        self.name = name
        if log_dir is None:
            self.log_dir = os.path.join(os.path.dirname(__file__), '..', 'logs')
        else:
            self.log_dir = log_dir

        self._file = None
        self._writer = None
        self._filepath = None
        self._row_count = 0
        self._start_time = None

    @property
    def is_logging(self) -> bool:
        return self._file is not None

    def start_logging(self, columns: list):
        """
        Open a new CSV log file and write the header row.

        columns: ordered list of column header strings, e.g.
                 ["Duty Cycle", "Velocity (RPM)", "Position (ticks)"]
                 A "Timestamp" column is always prepended automatically.
        """
        if self._file is not None:
            return  # already logging

        os.makedirs(self.log_dir, exist_ok=True)

        file_tag = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"{self.name}_{file_tag}.csv"
        self._filepath = os.path.join(self.log_dir, filename)

        self._file = open(self._filepath, 'w', newline='', encoding='utf-8')
        self._writer = csv.writer(self._file)
        self._row_count = 0
        self._start_time = time.monotonic()

        self._writer.writerow(["Timestamp"] + columns)
        self._file.flush()

        print(f"[{self.name.capitalize()}] Logging started -> {self._filepath}")

    def timestamp(self) -> str:
        """Return a formatted timestamp relative to when logging started."""
        if self._start_time is None:
            return "00:00.00"
        e = time.monotonic() - self._start_time
        minutes = int(e) // 60
        seconds = e % 60
        return f"{minutes:02d}:{seconds:05.2f}"

    def log_row(self, values: list):
        """
        Write one data row to the CSV log.

        values: list of raw numbers matching the column order from start_logging()
        """
        if self._file is None:
            return

        self._writer.writerow([self.timestamp()] + [str(v) for v in values])
        self._file.flush()
        self._row_count += 1

    def stop_logging(self):
        """Finalize the log file and close it."""
        if self._file is None:
            return

        self._file.close()
        self._file = None
        self._writer = None

        print(f"[{self.name.capitalize()}] Logging stopped — {self._row_count} rows saved to {self._filepath}")
        self._filepath = None
        self._row_count = 0
        self._start_time = None
