"""Reusable Tk widgets for CloudBaseVario telemetry."""

from __future__ import annotations

from collections import deque
import math
import tkinter as tk
from tkinter import ttk
from typing import Any

try:
    from .cloudbasevario_protocol import DisplayItem, TelemetryGroup
    from .cloudbasevario_theme import (
        COLOR_ACCENT,
        COLOR_BACKGROUND,
        COLOR_BLUE,
        COLOR_GREEN,
        COLOR_GRID,
        COLOR_MUTED,
        COLOR_PANEL,
        COLOR_PANEL_ALT,
        COLOR_RED,
        COLOR_TEXT,
        COLOR_YELLOW,
        DISPLAY_COLORS,
    )
except ImportError:
    from cloudbasevario_protocol import DisplayItem, TelemetryGroup
    from cloudbasevario_theme import (
        COLOR_ACCENT,
        COLOR_BACKGROUND,
        COLOR_BLUE,
        COLOR_GREEN,
        COLOR_GRID,
        COLOR_MUTED,
        COLOR_PANEL,
        COLOR_PANEL_ALT,
        COLOR_RED,
        COLOR_TEXT,
        COLOR_YELLOW,
        DISPLAY_COLORS,
    )


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
        self.unit = unit
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
        self.value_label = tk.Label(
            value_row,
            textvariable=self.value_var,
            bg=COLOR_PANEL,
            fg=COLOR_TEXT,
            font=("Segoe UI Semibold", 18),
        )
        self.value_label.pack(side="left")
        tk.Label(
            value_row,
            text=unit,
            bg=COLOR_PANEL,
            fg=COLOR_MUTED,
            font=("Segoe UI", 9),
        ).pack(side="left", padx=(5, 0), pady=(8, 0))

    def set(self, value: str) -> None:
        self.value_var.set(value)

    def set_item(self, item: DisplayItem) -> None:
        value = item.value
        unit_suffix = f" {self.unit}" if self.unit else ""
        if unit_suffix and value.endswith(unit_suffix):
            value = value[: -len(unit_suffix)]
        self.set(value)
        self.value_label.configure(
            fg=DISPLAY_COLORS.get(item.state, COLOR_TEXT)
        )


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

    def set_item(self, item: DisplayItem) -> None:
        self.configure(
            text=f"{self.title}: {item.value}",
            fg=DISPLAY_COLORS.get(item.state, COLOR_MUTED),
        )

    def unknown(self) -> None:
        self.configure(text=f"{self.title}: --", fg=COLOR_MUTED)


class DiagnosticTable(ttk.Labelframe):
    """Compact, color-coded current-value table for one telemetry domain."""

    def __init__(self, parent: tk.Misc, title: str) -> None:
        super().__init__(parent, text=title, padding=(8, 6))
        self.items: dict[str, str] = {}
        self.tree = ttk.Treeview(
            self,
            columns=("field", "value"),
            show="headings",
            height=9,
        )
        self.tree.heading("field", text="Field")
        self.tree.heading("value", text="Current value")
        self.tree.column("field", width=220, anchor="w", stretch=True)
        self.tree.column("value", width=150, anchor="e", stretch=False)
        for state, color in DISPLAY_COLORS.items():
            self.tree.tag_configure(state, foreground=color)
        scroll = ttk.Scrollbar(self, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll.set)
        self.tree.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")

    def update_group(self, group: TelemetryGroup) -> None:
        visible_keys = {item.key for item in group.items}
        for key, row in tuple(self.items.items()):
            if key not in visible_keys:
                self.tree.delete(row)
                del self.items[key]
        for item in group.items:
            row = self.items.get(item.key)
            values = (item.label, item.value)
            if row is None:
                row = self.tree.insert("", "end", values=values, tags=(item.state,))

                self.items[item.key] = row
            else:
                self.tree.item(row, values=values, tags=(item.state,))


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


