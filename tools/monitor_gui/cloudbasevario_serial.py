"""Serial transport used by the CloudBaseVario monitor."""

from __future__ import annotations

import queue
import threading
from collections import deque
from typing import Any

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

try:
    from .cloudbasevario_protocol import format_command
except ImportError:
    from cloudbasevario_protocol import format_command


class SerialEventInbox:
    """Keep control events ordered while coalescing periodic telemetry."""

    TELEMETRY_LOG_LINES = 5000

    def __init__(self) -> None:
        self._events: queue.Queue[tuple[str, Any]] = queue.Queue()
        self._latest_lines: dict[str, str | None] = {
            "BARO": None,
            "GPS": None,
        }
        self._telemetry_log: deque[str] = deque(
            maxlen=self.TELEMETRY_LOG_LINES
        )
        self._lock = threading.Lock()

    def put(self, event: tuple[str, Any]) -> None:
        event_type, payload = event
        if event_type == "line":
            line = str(payload)
            prefix = line.split(maxsplit=1)[0] if line else ""
            if prefix in self._latest_lines:
                with self._lock:
                    self._latest_lines[prefix] = line
                    self._telemetry_log.append(line)
                return
        self._events.put(event)

    def get_nowait(self) -> tuple[str, Any]:
        return self._events.get_nowait()

    def take_latest_lines(self) -> tuple[str | None, str | None]:
        with self._lock:
            lines = (self._latest_lines["BARO"], self._latest_lines["GPS"])
            self._latest_lines["BARO"] = None
            self._latest_lines["GPS"] = None
        return lines

    def drain_telemetry_log(self) -> list[str]:
        with self._lock:
            lines = list(self._telemetry_log)
            self._telemetry_log.clear()
        return lines

    def discard_telemetry_log(self) -> None:
        with self._lock:
            self._telemetry_log.clear()

    @property
    def control_event_count(self) -> int:
        return self._events.qsize()


class SerialWorker:
    """Own the serial port and transfer complete lines to the GUI thread."""

    def __init__(self, events: SerialEventInbox) -> None:
        self.events = events
        self.port: Any = None
        self.thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self.port is not None and bool(self.port.is_open)

    def connect(self, port_name: str, baudrate: int) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed")
        self.disconnect()
        self.stop_event.clear()
        self.port = serial.Serial(
            port=port_name,
            baudrate=baudrate,
            timeout=0.2,
            write_timeout=1.0,
        )
        self.thread = threading.Thread(
            target=self._read_loop,
            name="cloudbasevario-serial",
            daemon=True,
        )
        self.thread.start()

    def disconnect(self) -> None:
        self.stop_event.set()
        port = self.port
        self.port = None
        if port is not None:
            try:
                port.close()
            except Exception:
                pass
        thread = self.thread
        self.thread = None
        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=1.0)

    def send(self, command: str) -> None:
        payload = format_command(command)
        port = self.port
        if port is None or not port.is_open:
            raise RuntimeError("serial port is not connected")
        with self.write_lock:
            port.write(payload)
            port.flush()

    def _read_loop(self) -> None:
        port = self.port
        if port is None:
            return
        try:
            while not self.stop_event.is_set():
                raw = port.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                self.events.put(("line", line))
        except Exception as exc:
            if not self.stop_event.is_set():
                self.events.put(("serial_error", str(exc)))
        finally:
            try:
                port.close()
            except Exception:
                pass
            if self.port is port:
                self.port = None
            self.events.put(("disconnected", None))
