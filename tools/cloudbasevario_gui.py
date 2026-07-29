"""CloudBaseVario telemetry monitor and parameter editor."""

from __future__ import annotations

from collections import deque
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Any

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

try:
    from .cloudbasevario_protocol import (
        TelemetrySample,
        format_command,
        parse_parameter_line,
        parse_telemetry_line,
    )
except ImportError:
    from cloudbasevario_protocol import (
        TelemetrySample,
        format_command,
        parse_parameter_line,
        parse_telemetry_line,
    )


APP_TITLE = "CloudBaseVario Monitor"
DEFAULT_BAUD = 115200
GUI_POLL_MS = 30
COMMAND_TIMEOUT_MS = 4000
MAX_LOG_LINES = 5000

COLOR_BACKGROUND = "#10151c"
COLOR_PANEL = "#18212b"
COLOR_PANEL_ALT = "#202b37"
COLOR_TEXT = "#e8edf2"
COLOR_MUTED = "#8fa1b3"
COLOR_ACCENT = "#38bdf8"
COLOR_GREEN = "#4ade80"
COLOR_YELLOW = "#facc15"
COLOR_RED = "#fb7185"
COLOR_BLUE = "#60a5fa"
COLOR_GRID = "#334155"


class SerialWorker:
    """Own the serial port and transfer complete lines to the GUI thread."""

    def __init__(self, events: queue.Queue[tuple[str, Any]]) -> None:
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


class MetricCard(tk.Frame):
    def __init__(
        self, parent: tk.Misc, title: str, unit: str = ""
    ) -> None:
        super().__init__(
            parent,
            bg=COLOR_PANEL,
            highlightbackground=COLOR_GRID,
            highlightthickness=1,
            padx=12,
            pady=9,
        )
        self.value_var = tk.StringVar(value="--")
        tk.Label(
            self,
            text=title,
            bg=COLOR_PANEL,
            fg=COLOR_MUTED,
            font=("Segoe UI", 9),
        ).pack(anchor="w")
        value_row = tk.Frame(self, bg=COLOR_PANEL)
        value_row.pack(fill="x", pady=(3, 0))
        tk.Label(
            value_row,
            textvariable=self.value_var,
            bg=COLOR_PANEL,
            fg=COLOR_TEXT,
            font=("Segoe UI Semibold", 18),
        ).pack(side="left")
        tk.Label(
            value_row,
            text=unit,
            bg=COLOR_PANEL,
            fg=COLOR_MUTED,
            font=("Segoe UI", 9),
        ).pack(side="left", padx=(5, 0), pady=(8, 0))

    def set(self, value: str) -> None:
        self.value_var.set(value)


class StatusBadge(tk.Label):
    def __init__(self, parent: tk.Misc, title: str) -> None:
        super().__init__(
            parent,
            text=f"{title}: --",
            bg=COLOR_PANEL_ALT,
            fg=COLOR_MUTED,
            font=("Segoe UI Semibold", 9),
            padx=10,
            pady=4,
        )
        self.title = title

    def set(self, active: bool, detail: str | None = None) -> None:
        text = detail if detail is not None else ("ON" if active else "OFF")
        self.configure(
            text=f"{self.title}: {text}",
            fg=COLOR_GREEN if active else COLOR_RED,
        )

    def unknown(self) -> None:
        self.configure(text=f"{self.title}: --", fg=COLOR_MUTED)


class StripChart(tk.Frame):
    """Dependency-free rolling chart drawn with a Tk canvas."""

    def __init__(
        self,
        parent: tk.Misc,
        title: str,
        unit: str,
        color: str,
        seconds: float = 60.0,
    ) -> None:
        super().__init__(parent, bg=COLOR_PANEL)
        self.title = title
        self.unit = unit
        self.color = color
        self.seconds = seconds
        self.values: deque[tuple[float, float | None]] = deque(maxlen=900)
        self.canvas = tk.Canvas(
            self,
            height=135,
            bg=COLOR_PANEL,
            highlightbackground=COLOR_GRID,
            highlightthickness=1,
        )
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.redraw())

    def add(self, timestamp: float, value: float | None) -> None:
        self.values.append((timestamp, value))
        cutoff = timestamp - self.seconds
        while self.values and self.values[0][0] < cutoff:
            self.values.popleft()
        self.redraw()

    def clear(self) -> None:
        self.values.clear()
        self.redraw()

    def redraw(self) -> None:
        canvas = self.canvas
        width = max(canvas.winfo_width(), 80)
        height = max(canvas.winfo_height(), 60)
        left, right, top, bottom = 45, width - 10, 20, height - 20
        canvas.delete("all")

        for index in range(4):
            y = top + (bottom - top) * index / 3.0
            canvas.create_line(
                left, y, right, y, fill=COLOR_GRID, dash=(2, 4)
            )
        canvas.create_text(
            9,
            7,
            anchor="nw",
            text=self.title,
            fill=COLOR_TEXT,
            font=("Segoe UI Semibold", 9),
        )

        valid_values = [
            value for _, value in self.values if value is not None
        ]
        if not valid_values:
            canvas.create_text(
                width / 2,
                height / 2,
                text="No valid data",
                fill=COLOR_MUTED,
                font=("Segoe UI", 9),
            )
            return

        minimum = min(valid_values)
        maximum = max(valid_values)
        span = maximum - minimum
        if span < 1e-6:
            margin = max(abs(maximum) * 0.05, 0.1)
            minimum -= margin
            maximum += margin
        else:
            margin = span * 0.12
            minimum -= margin
            maximum += margin

        now = self.values[-1][0]
        start = now - self.seconds
        canvas.create_text(
            left - 4,
            top,
            anchor="e",
            text=f"{maximum:.2f}",
            fill=COLOR_MUTED,
            font=("Consolas", 8),
        )
        canvas.create_text(
            left - 4,
            bottom,
            anchor="e",
            text=f"{minimum:.2f}",
            fill=COLOR_MUTED,
            font=("Consolas", 8),
        )
        canvas.create_text(
            right,
            height - 5,
            anchor="se",
            text=self.unit,
            fill=COLOR_MUTED,
            font=("Segoe UI", 8),
        )

        segment: list[float] = []
        for timestamp, value in self.values:
            if value is None:
                if len(segment) >= 4:
                    canvas.create_line(
                        *segment, fill=self.color, width=2, smooth=True
                    )
                segment = []
                continue
            x = left + (timestamp - start) / self.seconds * (right - left)
            y = bottom - (value - minimum) / (maximum - minimum) * (
                bottom - top
            )
            segment.extend((x, y))
        if len(segment) >= 4:
            canvas.create_line(
                *segment, fill=self.color, width=2, smooth=True
            )


class AttitudeIndicator(tk.Frame):
    def __init__(self, parent: tk.Misc) -> None:
        super().__init__(parent, bg=COLOR_PANEL)
        self.roll = 0.0
        self.pitch = 0.0
        self.valid = False
        self.canvas = tk.Canvas(
            self,
            width=360,
            height=280,
            bg="#20354d",
            highlightbackground=COLOR_GRID,
            highlightthickness=1,
        )
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.redraw())

    def update_attitude(
        self, roll_deg: float, pitch_deg: float, valid: bool
    ) -> None:
        self.roll = roll_deg
        self.pitch = pitch_deg
        self.valid = valid
        self.redraw()

    def redraw(self) -> None:
        canvas = self.canvas
        width = max(canvas.winfo_width(), 120)
        height = max(canvas.winfo_height(), 100)
        center_x = width / 2.0
        center_y = height / 2.0
        canvas.delete("all")
        canvas.create_rectangle(0, 0, width, height, fill="#203b5a", width=0)

        if not self.valid:
            canvas.create_text(
                center_x,
                center_y,
                text="ATTITUDE INVALID",
                fill=COLOR_MUTED,
                font=("Segoe UI Semibold", 13),
            )
            return

        angle = math.radians(-self.roll)
        direction_x = math.cos(angle)
        direction_y = math.sin(angle)
        normal_x = -direction_y
        normal_y = direction_x
        horizon_y = center_y + max(-45.0, min(45.0, self.pitch)) * 2.2
        length = math.hypot(width, height) * 1.5
        first = (
            center_x - direction_x * length,
            horizon_y - direction_y * length,
        )
        second = (
            center_x + direction_x * length,
            horizon_y + direction_y * length,
        )
        ground_first = (
            first[0] + normal_x * length,
            first[1] + normal_y * length,
        )
        ground_second = (
            second[0] + normal_x * length,
            second[1] + normal_y * length,
        )
        canvas.create_polygon(
            *first,
            *second,
            *ground_second,
            *ground_first,
            fill="#6b4f37",
            outline="",
        )
        canvas.create_line(
            *first, *second, fill=COLOR_TEXT, width=3
        )

        for pitch_mark in (-20, -10, 10, 20):
            offset = (self.pitch - pitch_mark) * 2.2
            mark_center_x = center_x + normal_x * offset
            mark_center_y = center_y + normal_y * offset
            half_length = 25 if abs(pitch_mark) == 10 else 36
            canvas.create_line(
                mark_center_x - direction_x * half_length,
                mark_center_y - direction_y * half_length,
                mark_center_x + direction_x * half_length,
                mark_center_y + direction_y * half_length,
                fill=COLOR_TEXT,
                width=1,
            )

        wing = 42
        canvas.create_line(
            center_x - wing,
            center_y,
            center_x - 8,
            center_y,
            center_x,
            center_y + 8,
            center_x + 8,
            center_y,
            center_x + wing,
            center_y,
            fill=COLOR_YELLOW,
            width=3,
        )
        canvas.create_oval(
            center_x - 3,
            center_y - 3,
            center_x + 3,
            center_y + 3,
            outline=COLOR_YELLOW,
            width=2,
        )
        canvas.create_text(
            10,
            10,
            anchor="nw",
            text=f"ROLL {self.roll:+.1f}°",
            fill=COLOR_TEXT,
            font=("Consolas", 10),
        )
        canvas.create_text(
            width - 10,
            10,
            anchor="ne",
            text=f"PITCH {self.pitch:+.1f}°",
            fill=COLOR_TEXT,
            font=("Consolas", 10),
        )


class CloudBaseVarioApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry("1280x820")
        self.root.minsize(1000, 680)
        self.root.configure(bg=COLOR_BACKGROUND)

        self.events: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.serial_worker = SerialWorker(self.events)
        self.pending_command: str | None = None
        self.pending_parameters: dict[str, str] = {}
        self.parameters: dict[str, str] = {}
        self.parameter_dirty = False
        self.log_line_count = 0
        self.detail_items: dict[str, str] = {}
        self.telemetry_arrivals: deque[float] = deque(maxlen=50)
        self.last_telemetry_time = 0.0
        self.auto_list_after_connect: str | None = None

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.connection_var = tk.StringVar(value="Disconnected")
        self.command_status_var = tk.StringVar(value="Ready")
        self.rate_var = tk.StringVar(value="0.0 Hz")
        self.sequence_var = tk.StringVar(value="seq --")
        self.drop_var = tk.StringVar(value="drops --")
        self.telemetry_log_var = tk.BooleanVar(value=False)
        self.parameter_filter_var = tk.StringVar()
        self.selected_parameter_var = tk.StringVar(value="--")
        self.parameter_value_var = tk.StringVar()

        self._configure_style()
        self._build_layout()
        self._refresh_ports()
        self._set_connected_state(False)
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.after(GUI_POLL_MS, self._poll_events)

        if serial is None:
            self.command_status_var.set(
                "pyserial is missing: pip install -r tools/requirements-gui.txt"
            )

    def _configure_style(self) -> None:
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", font=("Segoe UI", 10))
        style.configure("TFrame", background=COLOR_BACKGROUND)
        style.configure(
            "Panel.TFrame", background=COLOR_PANEL, relief="flat"
        )
        style.configure(
            "TLabel", background=COLOR_BACKGROUND, foreground=COLOR_TEXT
        )
        style.configure(
            "Panel.TLabel", background=COLOR_PANEL, foreground=COLOR_TEXT
        )
        style.configure(
            "Muted.TLabel", background=COLOR_BACKGROUND, foreground=COLOR_MUTED
        )
        style.configure(
            "TButton",
            background=COLOR_PANEL_ALT,
            foreground=COLOR_TEXT,
            borderwidth=0,
            padding=(10, 6),
        )
        style.map(
            "TButton",
            background=[
                ("active", "#2c3b4b"),
                ("disabled", COLOR_PANEL),
            ],
            foreground=[("disabled", "#64748b")],
        )
        style.configure(
            "Accent.TButton",
            background="#0369a1",
            foreground="#ffffff",
        )
        style.map("Accent.TButton", background=[("active", "#0284c7")])
        style.configure(
            "Danger.TButton", background="#881337", foreground="#ffffff"
        )
        style.map("Danger.TButton", background=[("active", "#be123c")])
        style.configure(
            "TNotebook",
            background=COLOR_BACKGROUND,
            borderwidth=0,
        )
        style.configure(
            "TNotebook.Tab",
            background=COLOR_PANEL,
            foreground=COLOR_MUTED,
            padding=(16, 8),
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", COLOR_PANEL_ALT)],
            foreground=[("selected", COLOR_TEXT)],
        )
        style.configure(
            "Treeview",
            background=COLOR_PANEL,
            fieldbackground=COLOR_PANEL,
            foreground=COLOR_TEXT,
            rowheight=26,
            borderwidth=0,
        )
        style.map(
            "Treeview",
            background=[("selected", "#075985")],
            foreground=[("selected", "#ffffff")],
        )
        style.configure(
            "Treeview.Heading",
            background=COLOR_PANEL_ALT,
            foreground=COLOR_TEXT,
            relief="flat",
        )
        style.configure(
            "TEntry",
            fieldbackground=COLOR_PANEL_ALT,
            foreground=COLOR_TEXT,
        )
        style.configure(
            "TCombobox",
            fieldbackground=COLOR_PANEL_ALT,
            background=COLOR_PANEL_ALT,
            foreground=COLOR_TEXT,
        )

    def _build_layout(self) -> None:
        self._build_connection_bar()
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=12, pady=(0, 8))

        dashboard = ttk.Frame(notebook)
        detail = ttk.Frame(notebook)
        parameters = ttk.Frame(notebook)
        log_tab = ttk.Frame(notebook)
        notebook.add(dashboard, text="Telemetry")
        notebook.add(detail, text="All fields")
        notebook.add(parameters, text="Parameters")
        notebook.add(log_tab, text="Serial log")

        self._build_dashboard(dashboard)
        self._build_detail_tab(detail)
        self._build_parameter_tab(parameters)
        self._build_log_tab(log_tab)

        footer = ttk.Frame(self.root)
        footer.pack(fill="x", padx=12, pady=(0, 8))
        ttk.Label(
            footer, textvariable=self.command_status_var, style="Muted.TLabel"
        ).pack(side="left")
        ttk.Label(
            footer, textvariable=self.drop_var, style="Muted.TLabel"
        ).pack(side="right", padx=(14, 0))
        ttk.Label(
            footer, textvariable=self.sequence_var, style="Muted.TLabel"
        ).pack(side="right", padx=(14, 0))
        ttk.Label(
            footer, textvariable=self.rate_var, style="Muted.TLabel"
        ).pack(side="right")

    def _build_connection_bar(self) -> None:
        bar = ttk.Frame(self.root)
        bar.pack(fill="x", padx=12, pady=12)
        ttk.Label(
            bar,
            text="CloudBaseVario",
            font=("Segoe UI Semibold", 16),
        ).pack(side="left", padx=(0, 24))
        ttk.Label(bar, text="COM").pack(side="left")
        self.port_combo = ttk.Combobox(
            bar, textvariable=self.port_var, width=28, state="readonly"
        )
        self.port_combo.pack(side="left", padx=(6, 6))
        self.refresh_button = ttk.Button(
            bar, text="Refresh", command=self._refresh_ports
        )
        self.refresh_button.pack(side="left", padx=(0, 14))
        ttk.Label(bar, text="Baud").pack(side="left")
        self.baud_combo = ttk.Combobox(
            bar,
            textvariable=self.baud_var,
            width=9,
            values=("115200", "230400", "460800", "921600"),
        )
        self.baud_combo.pack(side="left", padx=(6, 14))
        self.connect_button = ttk.Button(
            bar,
            text="Connect",
            style="Accent.TButton",
            command=self._toggle_connection,
        )
        self.connect_button.pack(side="left")
        ttk.Label(
            bar,
            textvariable=self.connection_var,
            style="Muted.TLabel",
        ).pack(side="right")

    def _build_dashboard(self, parent: ttk.Frame) -> None:
        metrics = tk.Frame(parent, bg=COLOR_BACKGROUND)
        metrics.pack(fill="x", pady=(8, 8))
        for column in range(5):
            metrics.grid_columnconfigure(column, weight=1)

        self.pressure_card = MetricCard(metrics, "Pressure", "Pa")
        self.altitude_card = MetricCard(metrics, "Altitude", "m")
        self.climb_card = MetricCard(metrics, "Climb rate", "m/s")
        self.vertical_accel_card = MetricCard(
            metrics, "Vertical accel", "m/s²"
        )
        self.battery_card = MetricCard(metrics, "BLE battery", "V")
        cards = (
            self.pressure_card,
            self.altitude_card,
            self.climb_card,
            self.vertical_accel_card,
            self.battery_card,
        )
        for column, card in enumerate(cards):
            card.grid(
                row=0,
                column=column,
                sticky="nsew",
                padx=(0 if column == 0 else 4, 0 if column == 4 else 4),
            )

        badges = tk.Frame(parent, bg=COLOR_BACKGROUND)
        badges.pack(fill="x", pady=(0, 8))
        self.baro_badge = StatusBadge(badges, "BARO")
        self.imu_badge = StatusBadge(badges, "IMU")
        self.calibration_badge = StatusBadge(badges, "CAL")
        self.attitude_badge = StatusBadge(badges, "ATT")
        self.fusion_badge = StatusBadge(badges, "FUSION")
        self.ble_badge = StatusBadge(badges, "BLE")
        for badge in (
            self.baro_badge,
            self.imu_badge,
            self.calibration_badge,
            self.attitude_badge,
            self.fusion_badge,
            self.ble_badge,
        ):
            badge.pack(side="left", padx=(0, 6))

        content = tk.Frame(parent, bg=COLOR_BACKGROUND)
        content.pack(fill="both", expand=True)
        content.grid_columnconfigure(0, weight=3)
        content.grid_columnconfigure(1, weight=2)
        content.grid_rowconfigure(0, weight=1)

        charts = tk.Frame(content, bg=COLOR_BACKGROUND)
        charts.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self.pressure_chart = StripChart(
            charts, "Pressure", "Pa / last 60 s", COLOR_BLUE
        )
        self.climb_chart = StripChart(
            charts, "Climb rate", "m/s / last 60 s", COLOR_GREEN
        )
        self.accel_chart = StripChart(
            charts, "Vertical acceleration", "m/s² / last 60 s", COLOR_YELLOW
        )
        for chart in (
            self.pressure_chart,
            self.climb_chart,
            self.accel_chart,
        ):
            chart.pack(fill="both", expand=True, pady=(0, 6))

        attitude_panel = tk.Frame(
            content,
            bg=COLOR_PANEL,
            highlightbackground=COLOR_GRID,
            highlightthickness=1,
        )
        attitude_panel.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        tk.Label(
            attitude_panel,
            text="6DoF attitude",
            bg=COLOR_PANEL,
            fg=COLOR_TEXT,
            font=("Segoe UI Semibold", 11),
        ).pack(anchor="w", padx=10, pady=(8, 6))
        self.attitude_indicator = AttitudeIndicator(attitude_panel)
        self.attitude_indicator.pack(
            fill="both", expand=True, padx=10, pady=(0, 8)
        )
        self.attitude_text_var = tk.StringVar(
            value="q = --\nyaw = --  (relative, not magnetic heading)"
        )
        tk.Label(
            attitude_panel,
            textvariable=self.attitude_text_var,
            justify="left",
            anchor="w",
            bg=COLOR_PANEL,
            fg=COLOR_MUTED,
            font=("Consolas", 9),
        ).pack(fill="x", padx=10, pady=(0, 10))

    def _build_detail_tab(self, parent: ttk.Frame) -> None:
        container = ttk.Frame(parent)
        container.pack(fill="both", expand=True, padx=8, pady=8)
        self.detail_tree = ttk.Treeview(
            container,
            columns=("field", "value"),
            show="headings",
        )
        self.detail_tree.heading("field", text="Field")
        self.detail_tree.heading("value", text="Current value")
        self.detail_tree.column("field", width=300, anchor="w")
        self.detail_tree.column("value", width=260, anchor="w")
        detail_scroll = ttk.Scrollbar(
            container, orient="vertical", command=self.detail_tree.yview
        )
        self.detail_tree.configure(yscrollcommand=detail_scroll.set)
        self.detail_tree.pack(side="left", fill="both", expand=True)
        detail_scroll.pack(side="right", fill="y")

    def _build_parameter_tab(self, parent: ttk.Frame) -> None:
        toolbar = ttk.Frame(parent)
        toolbar.pack(fill="x", padx=8, pady=8)
        self.load_parameters_button = ttk.Button(
            toolbar, text="Reload from device", command=self._request_parameters
        )
        self.load_parameters_button.pack(side="left")
        self.save_parameters_button = ttk.Button(
            toolbar,
            text="PARAM SAVE",
            style="Accent.TButton",
            command=lambda: self._begin_command("PARAM SAVE"),
        )
        self.save_parameters_button.pack(side="left", padx=(8, 0))
        self.reset_all_button = ttk.Button(
            toolbar,
            text="Reset all in RAM",
            style="Danger.TButton",
            command=self._reset_all_parameters,
        )
        self.reset_all_button.pack(side="left", padx=(8, 0))
        ttk.Label(toolbar, text="Filter").pack(side="right", padx=(8, 4))
        filter_entry = ttk.Entry(
            toolbar, textvariable=self.parameter_filter_var, width=28
        )
        filter_entry.pack(side="right")
        self.parameter_filter_var.trace_add(
            "write", lambda *_args: self._refresh_parameter_tree()
        )

        content = ttk.Frame(parent)
        content.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        content.grid_columnconfigure(0, weight=3)
        content.grid_columnconfigure(1, weight=2)
        content.grid_rowconfigure(0, weight=1)

        table_frame = ttk.Frame(content)
        table_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self.parameter_tree = ttk.Treeview(
            table_frame,
            columns=("name", "value"),
            show="headings",
            selectmode="browse",
        )
        self.parameter_tree.heading("name", text="Parameter")
        self.parameter_tree.heading("value", text="RAM value")
        self.parameter_tree.column("name", width=330, anchor="w")
        self.parameter_tree.column("value", width=150, anchor="w")
        parameter_scroll = ttk.Scrollbar(
            table_frame,
            orient="vertical",
            command=self.parameter_tree.yview,
        )
        self.parameter_tree.configure(yscrollcommand=parameter_scroll.set)
        self.parameter_tree.pack(side="left", fill="both", expand=True)
        parameter_scroll.pack(side="right", fill="y")
        self.parameter_tree.bind(
            "<<TreeviewSelect>>", self._parameter_selected
        )
        self.parameter_tree.bind(
            "<Double-1>", lambda _event: self.parameter_value_combo.focus_set()
        )

        editor = tk.Frame(
            content,
            bg=COLOR_PANEL,
            highlightbackground=COLOR_GRID,
            highlightthickness=1,
            padx=16,
            pady=14,
        )
        editor.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        tk.Label(
            editor,
            text="Selected parameter",
            bg=COLOR_PANEL,
            fg=COLOR_MUTED,
            font=("Segoe UI", 9),
        ).pack(anchor="w")
        tk.Label(
            editor,
            textvariable=self.selected_parameter_var,
            bg=COLOR_PANEL,
            fg=COLOR_TEXT,
            font=("Consolas", 11, "bold"),
            wraplength=360,
            justify="left",
        ).pack(anchor="w", pady=(3, 16))
        tk.Label(
            editor,
            text="New RAM value",
            bg=COLOR_PANEL,
            fg=COLOR_MUTED,
            font=("Segoe UI", 9),
        ).pack(anchor="w")
        self.parameter_value_combo = ttk.Combobox(
            editor, textvariable=self.parameter_value_var
        )
        self.parameter_value_combo.pack(fill="x", pady=(4, 12))
        self.set_parameter_button = ttk.Button(
            editor,
            text="Set in RAM",
            style="Accent.TButton",
            command=self._set_selected_parameter,
        )
        self.set_parameter_button.pack(fill="x")
        self.reset_parameter_button = ttk.Button(
            editor,
            text="Reset selected in RAM",
            command=self._reset_selected_parameter,
        )
        self.reset_parameter_button.pack(fill="x", pady=(8, 0))
        self.parameter_dirty_label = tk.Label(
            editor,
            text="No unsaved GUI changes",
            bg=COLOR_PANEL,
            fg=COLOR_MUTED,
            font=("Segoe UI", 9),
            wraplength=360,
            justify="left",
        )
        self.parameter_dirty_label.pack(anchor="w", pady=(20, 0))
        tk.Label(
            editor,
            text=(
                "SET and RESET change RAM only. Use PARAM SAVE explicitly "
                "to persist all values. Firmware validates type, range, and "
                "cross-parameter relationships."
            ),
            bg=COLOR_PANEL,
            fg=COLOR_MUTED,
            font=("Segoe UI", 9),
            wraplength=360,
            justify="left",
        ).pack(anchor="w", pady=(14, 0))

    def _build_log_tab(self, parent: ttk.Frame) -> None:
        toolbar = ttk.Frame(parent)
        toolbar.pack(fill="x", padx=8, pady=8)
        ttk.Button(
            toolbar, text="Clear", command=self._clear_log
        ).pack(side="left")
        ttk.Checkbutton(
            toolbar,
            text="Include 10 Hz BARO telemetry",
            variable=self.telemetry_log_var,
        ).pack(side="left", padx=(12, 0))
        self.log_text = tk.Text(
            parent,
            bg="#0b1118",
            fg="#cbd5e1",
            insertbackground=COLOR_TEXT,
            selectbackground="#075985",
            wrap="none",
            font=("Consolas", 9),
            state="disabled",
        )
        log_y_scroll = ttk.Scrollbar(
            parent, orient="vertical", command=self.log_text.yview
        )
        log_x_scroll = ttk.Scrollbar(
            parent, orient="horizontal", command=self.log_text.xview
        )
        self.log_text.configure(
            yscrollcommand=log_y_scroll.set,
            xscrollcommand=log_x_scroll.set,
        )
        log_y_scroll.pack(side="right", fill="y", padx=(0, 8))
        log_x_scroll.pack(side="bottom", fill="x", padx=8, pady=(0, 8))
        self.log_text.pack(fill="both", expand=True, padx=(8, 0), pady=(0, 0))

    def _refresh_ports(self) -> None:
        if list_ports is None:
            self.port_combo.configure(values=())
            return
        ports = sorted(
            list_ports.comports(), key=lambda item: item.device.lower()
        )
        values = [
            f"{item.device} — {item.description}" for item in ports
        ]
        current_device = self._selected_port_device()
        self.port_combo.configure(values=values)
        if values:
            matching = [
                value
                for value in values
                if value.split(" — ", 1)[0] == current_device
            ]
            self.port_var.set(matching[0] if matching else values[0])
        elif not self.serial_worker.connected:
            self.port_var.set("")

    def _selected_port_device(self) -> str:
        return self.port_var.get().split(" — ", 1)[0].strip()

    def _toggle_connection(self) -> None:
        if self.serial_worker.connected:
            self._disconnect()
            return
        port_name = self._selected_port_device()
        if not port_name:
            messagebox.showerror(APP_TITLE, "Select a COM port.")
            return
        try:
            baudrate = int(self.baud_var.get(), 10)
            self.serial_worker.connect(port_name, baudrate)
        except Exception as exc:
            messagebox.showerror(APP_TITLE, f"Connection failed:\n{exc}")
            self._set_connected_state(False)
            return
        self._set_connected_state(True)
        self.connection_var.set(f"Connected: {port_name}")
        self.command_status_var.set("Connected; waiting for telemetry")
        self._append_log(f"[GUI] connected to {port_name}")
        self.auto_list_after_connect = self.root.after(
            600, self._request_parameters
        )

    def _disconnect(self) -> None:
        if self.auto_list_after_connect is not None:
            self.root.after_cancel(self.auto_list_after_connect)
            self.auto_list_after_connect = None
        self.serial_worker.disconnect()
        self.pending_command = None
        self.pending_parameters.clear()
        self._set_connected_state(False)
        self.connection_var.set("Disconnected")
        self.command_status_var.set("Disconnected")
        self._append_log("[GUI] disconnected")

    def _set_connected_state(self, connected: bool) -> None:
        dependency_ready = serial is not None
        self.connect_button.configure(
            text="Disconnect" if connected else "Connect",
            style="Danger.TButton" if connected else "Accent.TButton",
            state="normal" if dependency_ready else "disabled",
        )
        port_state = "disabled" if connected else "readonly"
        self.port_combo.configure(state=port_state)
        self.baud_combo.configure(state="disabled" if connected else "normal")
        self.refresh_button.configure(
            state="disabled" if connected else "normal"
        )
        self._update_command_controls()
        if not connected:
            for badge in (
                self.baro_badge,
                self.imu_badge,
                self.calibration_badge,
                self.attitude_badge,
                self.fusion_badge,
                self.ble_badge,
            ):
                badge.unknown()

    def _update_command_controls(self) -> None:
        enabled = self.serial_worker.connected and self.pending_command is None
        state = "normal" if enabled else "disabled"
        selection = bool(self.parameter_tree.selection())
        self.load_parameters_button.configure(state=state)
        self.save_parameters_button.configure(state=state)
        self.reset_all_button.configure(state=state)
        self.set_parameter_button.configure(
            state=state if selection else "disabled"
        )
        self.reset_parameter_button.configure(
            state=state if selection else "disabled"
        )

    def _request_parameters(self) -> None:
        self.auto_list_after_connect = None
        self._begin_command("PARAM LIST")

    def _begin_command(self, command: str) -> None:
        if not self.serial_worker.connected:
            self.command_status_var.set("Connect before sending a command")
            return
        if self.pending_command is not None:
            self.command_status_var.set(
                f"Waiting for: {self.pending_command}"
            )
            return
        try:
            self.serial_worker.send(command)
        except Exception as exc:
            self.command_status_var.set(f"Send failed: {exc}")
            return

        self.pending_command = command
        self.pending_parameters.clear()
        self.command_status_var.set(f"Sent: {command}")
        self._append_log(f"> {command}")
        self._update_command_controls()
        self.root.after(
            COMMAND_TIMEOUT_MS,
            lambda expected=command: self._command_timeout(expected),
        )

    def _command_timeout(self, expected: str) -> None:
        if self.pending_command != expected:
            return
        self.pending_command = None
        self.pending_parameters.clear()
        self.command_status_var.set(f"Timeout: {expected}")
        self._update_command_controls()

    def _finish_command(self, response: str) -> None:
        command = self.pending_command
        if command is None:
            return
        self.pending_command = None
        successful = response == "OK"

        if successful and command == "PARAM LIST":
            self.parameters = dict(self.pending_parameters)
            self._refresh_parameter_tree()
            self.command_status_var.set(
                f"Loaded {len(self.parameters)} parameters"
            )
        elif successful and command.startswith("PARAM SET "):
            parts = command.split(maxsplit=3)
            if len(parts) == 4:
                self.parameters[parts[2]] = parts[3]
            self._set_parameter_dirty(True)
            self._refresh_parameter_tree()
            self.command_status_var.set("RAM value updated; not saved")
        elif successful and command.startswith("PARAM RESET "):
            self._set_parameter_dirty(True)
            self.command_status_var.set("RAM value reset; not saved")
            self.root.after(100, self._request_parameters)
        elif successful and command == "PARAM SAVE":
            self._set_parameter_dirty(False)
            self.command_status_var.set("parameters.json saved")
        elif successful:
            self.command_status_var.set(f"Completed: {command}")
        else:
            self.command_status_var.set(f"{command}: {response}")

        self.pending_parameters.clear()
        self._update_command_controls()

    def _set_parameter_dirty(self, dirty: bool) -> None:
        self.parameter_dirty = dirty
        self.parameter_dirty_label.configure(
            text=(
                "RAM has unsaved changes"
                if dirty
                else "No unsaved GUI changes"
            ),
            fg=COLOR_YELLOW if dirty else COLOR_MUTED,
        )

    def _refresh_parameter_tree(self) -> None:
        selected_name = self.selected_parameter_var.get()
        filter_text = self.parameter_filter_var.get().strip().lower()
        for item in self.parameter_tree.get_children():
            self.parameter_tree.delete(item)
        selected_item = None
        for name in sorted(self.parameters):
            value = self.parameters[name]
            if filter_text and filter_text not in name.lower():
                continue
            item = self.parameter_tree.insert(
                "", "end", values=(name, value)
            )
            if name == selected_name:
                selected_item = item
        if selected_item is not None:
            self.parameter_tree.selection_set(selected_item)
            self.parameter_tree.see(selected_item)
        self._update_command_controls()

    def _parameter_selected(self, _event: tk.Event[Any]) -> None:
        selection = self.parameter_tree.selection()
        if not selection:
            self.selected_parameter_var.set("--")
            self.parameter_value_var.set("")
            self._update_command_controls()
            return
        values = self.parameter_tree.item(selection[0], "values")
        if len(values) < 2:
            return
        name, value = str(values[0]), str(values[1])
        self.selected_parameter_var.set(name)
        self.parameter_value_var.set(value)
        suggestions: tuple[str, ...] = ()
        if value.lower() in ("true", "false"):
            suggestions = ("true", "false")
        elif name == "filter_mode":
            suggestions = ("AUTO", "BARO_ONLY")
        elif name.endswith("_sign"):
            suggestions = ("-1", "1")
        self.parameter_value_combo.configure(values=suggestions)
        self._update_command_controls()

    def _set_selected_parameter(self) -> None:
        name = self.selected_parameter_var.get()
        value = self.parameter_value_var.get().strip()
        if name == "--" or not value:
            self.command_status_var.set("Select a parameter and enter a value")
            return
        if any(character.isspace() for character in value):
            self.command_status_var.set(
                "Parameter values cannot contain whitespace"
            )
            return
        self._begin_command(f"PARAM SET {name} {value}")

    def _reset_selected_parameter(self) -> None:
        name = self.selected_parameter_var.get()
        if name == "--":
            return
        if messagebox.askyesno(
            APP_TITLE, f"Reset {name} to its built-in default in RAM?"
        ):
            self._begin_command(f"PARAM RESET {name}")

    def _reset_all_parameters(self) -> None:
        if messagebox.askyesno(
            APP_TITLE,
            "Reset every parameter to built-in defaults in RAM?\n"
            "This does not save until PARAM SAVE is used.",
        ):
            self._begin_command("PARAM RESET ALL")

    def _poll_events(self) -> None:
        processed = 0
        while processed < 300:
            try:
                event_type, payload = self.events.get_nowait()
            except queue.Empty:
                break
            processed += 1
            if event_type == "line":
                self._process_serial_line(str(payload))
            elif event_type == "serial_error":
                self.command_status_var.set(f"Serial error: {payload}")
                self._append_log(f"[ERROR] {payload}")
            elif event_type == "disconnected":
                if not self.serial_worker.connected:
                    self.pending_command = None
                    self._set_connected_state(False)
                    self.connection_var.set("Disconnected")

        if (
            self.serial_worker.connected
            and self.last_telemetry_time > 0.0
            and time.monotonic() - self.last_telemetry_time > 1.0
        ):
            self.connection_var.set("Connected — telemetry stale")
            self.baro_badge.set(False, "STALE")
        self.root.after(GUI_POLL_MS, self._poll_events)

    def _process_serial_line(self, line: str) -> None:
        telemetry = parse_telemetry_line(line)
        if telemetry is not None:
            self._handle_telemetry(telemetry)
            if self.telemetry_log_var.get():
                self._append_log(line)
            return

        self._append_log(line)
        if self.pending_command is None:
            return
        if line == "OK" or line.startswith("ERR"):
            self._finish_command(line)
            return
        if self.pending_command in ("PARAM LIST",) or self.pending_command.startswith(
            "PARAM GET "
        ):
            parameter = parse_parameter_line(line)
            if parameter is not None:
                name, value = parameter
                self.pending_parameters[name] = value

    def _handle_telemetry(self, sample: TelemetrySample) -> None:
        now = time.monotonic()
        self.last_telemetry_time = now
        self.telemetry_arrivals.append(now)
        if len(self.telemetry_arrivals) >= 2:
            elapsed = self.telemetry_arrivals[-1] - self.telemetry_arrivals[0]
            if elapsed > 0.0:
                rate = (len(self.telemetry_arrivals) - 1) / elapsed
                self.rate_var.set(f"{rate:.1f} Hz")

        sequence = sample.integer("seq")
        drops = sample.integer("stream_drops")
        self.sequence_var.set(f"seq {sequence}")
        self.drop_var.set(f"drops {drops}")
        self.connection_var.set(
            f"Connected — telemetry {self.rate_var.get()}"
        )

        pressure_valid = sample.flag("pressure_valid")
        climb_valid = sample.flag("climb_valid")
        vertical_accel_valid = sample.flag("vertical_accel_valid")
        attitude_valid = sample.flag("imu_attitude_valid")

        pressure = sample.number("pressure_pa")
        altitude = sample.number("altitude_m")
        climb = sample.number("climb_mps")
        vertical_accel = sample.number("vertical_accel_mps2")
        roll = sample.number("roll_deg")
        pitch = sample.number("pitch_deg")
        yaw = sample.number("yaw_deg")

        self.pressure_card.set(f"{pressure:.2f}" if pressure_valid else "--")
        self.altitude_card.set(
            f"{altitude:.2f}" if sample.flag("estimate_valid") else "--"
        )
        self.climb_card.set(f"{climb:+.3f}" if climb_valid else "--")
        self.vertical_accel_card.set(
            f"{vertical_accel:+.3f}" if vertical_accel_valid else "--"
        )
        battery = sample.text("ble_battery", "999")
        self.battery_card.set("--" if battery == "999" else battery)

        self.baro_badge.set(sample.flag("online"))
        self.imu_badge.set(sample.flag("imu_online"))
        self.calibration_badge.set(sample.flag("imu_calibrated"))
        self.attitude_badge.set(attitude_valid)
        self.fusion_badge.set(sample.flag("fusion_active"))
        self.ble_badge.set(sample.flag("ble_notify"))

        self.attitude_indicator.update_attitude(
            roll, pitch, attitude_valid
        )
        if attitude_valid:
            self.attitude_text_var.set(
                "q = "
                f"{sample.number('q_w'):+.5f}, "
                f"{sample.number('q_x'):+.5f}, "
                f"{sample.number('q_y'):+.5f}, "
                f"{sample.number('q_z'):+.5f}\n"
                f"yaw = {yaw:+.2f}°  (relative, not magnetic heading)"
            )
        else:
            self.attitude_text_var.set(
                "q = --\nyaw = --  (relative, not magnetic heading)"
            )

        self.pressure_chart.add(
            now, pressure if pressure_valid else None
        )
        self.climb_chart.add(now, climb if climb_valid else None)
        self.accel_chart.add(
            now, vertical_accel if vertical_accel_valid else None
        )
        self._update_detail_fields(sample)

    def _update_detail_fields(self, sample: TelemetrySample) -> None:
        for name in sorted(sample.fields):
            value = sample.fields[name]
            item = self.detail_items.get(name)
            if item is None:
                item = self.detail_tree.insert(
                    "", "end", values=(name, str(value))
                )
                self.detail_items[name] = item
            else:
                self.detail_tree.item(item, values=(name, str(value)))

    def _append_log(self, line: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", line + "\n")
        self.log_line_count += 1
        if self.log_line_count > MAX_LOG_LINES:
            remove_count = self.log_line_count - MAX_LOG_LINES
            self.log_text.delete("1.0", f"{remove_count + 1}.0")
            self.log_line_count = MAX_LOG_LINES
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _clear_log(self) -> None:
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")
        self.log_line_count = 0

    def _close(self) -> None:
        self.serial_worker.disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    app = CloudBaseVarioApp(root)
    _ = app
    root.mainloop()


if __name__ == "__main__":
    main()
