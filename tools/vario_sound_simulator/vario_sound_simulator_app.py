"""Vario sound simulator application composition and controller."""

from __future__ import annotations

import math
from pathlib import Path
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Any

try:
    from .vario_sound_audio import AudioEngine
    from .parameters_model import (
        AUDIO_PARAMETER_NAMES,
        MODEL_PARAMETER_SPECS,
        PARAMETER_SPECS,
        PROFILE_PARAMETER_SPECS,
        SHARED_PARAMETER_SPECS,
        ConfigDocument,
        ConfigError,
        default_config_document,
        default_parameters,
        load_config_document_file,
        save_config_document_file,
        validate_parameters,
    )
    from .vario_sound_model import (
        AudioMode,
        VarioAudioCommand,
        VarioAudioState,
        VarioSample,
        reset_audio_state,
        vario_audio_step,
    )
except ImportError:
    from vario_sound_audio import AudioEngine
    from parameters_model import (
        AUDIO_PARAMETER_NAMES,
        MODEL_PARAMETER_SPECS,
        PARAMETER_SPECS,
        PROFILE_PARAMETER_SPECS,
        SHARED_PARAMETER_SPECS,
        ConfigDocument,
        ConfigError,
        default_config_document,
        default_parameters,
        load_config_document_file,
        save_config_document_file,
        validate_parameters,
    )
    from vario_sound_model import (
        AudioMode,
        VarioAudioCommand,
        VarioAudioState,
        VarioSample,
        reset_audio_state,
        vario_audio_step,
    )


APP_TITLE = "CloudBaseVario Vario Sound Simulator"
SIMULATION_PERIOD_MS = 10
CLIMB_RATE_MIN_MPS = -15.0
CLIMB_RATE_MAX_MPS = 15.0

COLOR_BACKGROUND = "#10151c"
COLOR_PANEL = "#18212b"
COLOR_PANEL_ALT = "#202b37"
COLOR_TEXT = "#e8edf2"
COLOR_MUTED = "#8fa1b3"
COLOR_ACCENT = "#38bdf8"
COLOR_GREEN = "#4ade80"
COLOR_YELLOW = "#facc15"
COLOR_RED = "#fb7185"


PARAMETER_GROUPS = (
    (
        "General",
        (
            "audio_enabled",
            "audio_climb_rate_average_s",
            "audio_state_hold_ms",
            "audio_stale_ms",
        ),
    ),
    (
        "Lift detection",
        (
            "lift_start_mps",
            "lift_end_mps",
        ),
    ),
    (
        "Lift tone",
        (
            "lift_freq_base_hz",
            "lift_freq_rate_hz_per_mps",
            "lift_freq_max_hz",
            "lift_time_ms_at_0p2",
            "lift_time_ms_at_1p0",
            "lift_time_ms_at_2p5",
            "lift_time_ms_at_5p0",
        ),
    ),
    (
        "Sink",
        (
            "sink_enabled",
            "sink_start_mps",
            "sink_end_mps",
            "sink_freq_start_hz",
            "sink_freq_rate_hz_per_mps",
            "sink_freq_min_hz",
        ),
    ),
    (
        "Predictive",
        (
            "predictive_buzzer_enabled",
            "predictive_min_mps",
            "predictive_interval_ms",
            "predictive_duration_ms",
        ),
    ),
    (
        "Output",
        (
            "audio_duty_percent",
            "audio_amp_mode",
        ),
    ),
)


class VarioSoundSimulatorApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry("1280x820")
        self.root.minsize(1040, 700)
        self.root.configure(bg=COLOR_BACKGROUND)

        self.audio_engine = AudioEngine()
        self.audio_state = VarioAudioState()
        self.command = VarioAudioCommand()
        self.document = default_config_document()
        self.values = self.document.effective_parameters(1)
        self.saved_document: ConfigDocument | None = None
        self.active_parameter_number = 1
        self.current_path: Path | None = None
        self.parameter_vars: dict[str, tk.Variable] = {}
        self.validation_after: str | None = None
        self.loading_editor = False
        self.editor_valid = True
        self.dirty = True

        self.running = False
        self.virtual_altitude_m = 0.0
        self.climb_rate_mps = 0.0
        self.last_tick_s = time.monotonic()

        self.file_var = tk.StringVar(value="New version 1 configuration")
        self.parameter_number_var = tk.StringVar(value="1")
        self.validation_var = tk.StringVar(value="Configuration is valid")
        self.rate_entry_var = tk.StringVar(value="0.00")
        self.rate_scale_var = tk.DoubleVar(value=0.0)
        self.volume_var = tk.DoubleVar(value=35.0)
        self.volume_text_var = tk.StringVar(value="35 %")
        self.run_status_var = tk.StringVar(value="Stopped")
        self.mode_var = tk.StringVar(value=AudioMode.SILENT.value)
        self.output_var = tk.StringVar(value="OFF")
        self.frequency_var = tk.StringVar(value="-- Hz")
        self.altitude_var = tk.StringVar(value="0.000 m")
        self.instant_rate_var = tk.StringVar(value="0.000 m/s")
        self.average_rate_var = tk.StringVar(value="-- m/s")
        self.audio_status_var = tk.StringVar(value="Default output device")

        self._configure_style()
        self._build_layout()
        self._load_values_into_editor(self.values)
        self._update_title()
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.after(SIMULATION_PERIOD_MS, self._tick)

    def _configure_style(self) -> None:
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", font=("Segoe UI", 10))
        style.configure("TFrame", background=COLOR_BACKGROUND)
        style.configure("Panel.TFrame", background=COLOR_PANEL)
        style.configure("TLabel", background=COLOR_BACKGROUND, foreground=COLOR_TEXT)
        style.configure("Panel.TLabel", background=COLOR_PANEL, foreground=COLOR_TEXT)
        style.configure("Muted.TLabel", background=COLOR_BACKGROUND, foreground=COLOR_MUTED)
        style.configure(
            "TButton", background=COLOR_PANEL_ALT, foreground=COLOR_TEXT, padding=(9, 6)
        )
        style.map("TButton", background=[("active", "#2c3b4b")])
        style.configure("Accent.TButton", background="#0369a1", foreground="#ffffff")
        style.map("Accent.TButton", background=[("active", "#0284c7")])
        style.configure("Danger.TButton", background="#881337", foreground="#ffffff")
        style.map("Danger.TButton", background=[("active", "#be123c")])
        style.configure("TNotebook", background=COLOR_BACKGROUND, borderwidth=0)
        style.configure(
            "TNotebook.Tab",
            background=COLOR_PANEL,
            foreground=COLOR_MUTED,
            padding=(12, 7),
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", COLOR_PANEL_ALT)],
            foreground=[("selected", COLOR_TEXT)],
        )
        style.configure("TEntry", fieldbackground=COLOR_PANEL_ALT, foreground=COLOR_TEXT)
        style.configure("TSpinbox", fieldbackground=COLOR_PANEL_ALT, foreground=COLOR_TEXT)
        style.configure("TCheckbutton", background=COLOR_PANEL, foreground=COLOR_TEXT)
        style.configure("TLabelframe", background=COLOR_PANEL, foreground=COLOR_TEXT)
        style.configure("TLabelframe.Label", background=COLOR_PANEL, foreground=COLOR_TEXT)

    def _build_layout(self) -> None:
        self._build_file_bar()
        panes = ttk.Panedwindow(self.root, orient="horizontal")
        panes.pack(fill="both", expand=True, padx=12, pady=(0, 8))
        editor = ttk.Frame(panes, style="Panel.TFrame", width=680)
        simulator = ttk.Frame(panes, style="Panel.TFrame", width=500)
        panes.add(editor, weight=3)
        panes.add(simulator, weight=2)
        self._build_parameter_editor(editor)
        self._build_simulator(simulator)

        footer = ttk.Frame(self.root)
        footer.pack(fill="x", padx=12, pady=(0, 8))
        self.validation_label = tk.Label(
            footer,
            textvariable=self.validation_var,
            bg=COLOR_BACKGROUND,
            fg=COLOR_GREEN,
            anchor="w",
        )
        self.validation_label.pack(side="left", fill="x", expand=True)
        ttk.Label(footer, textvariable=self.audio_status_var, style="Muted.TLabel").pack(
            side="right"
        )

    def _build_file_bar(self) -> None:
        bar = ttk.Frame(self.root)
        bar.pack(fill="x", padx=12, pady=12)
        ttk.Label(bar, text="Vario Sound Simulator", font=("Segoe UI Semibold", 16)).pack(
            side="left", padx=(0, 20)
        )
        ttk.Button(bar, text="New", command=self._new_configuration).pack(side="left")
        ttk.Button(bar, text="Open...", command=self._open_configuration).pack(
            side="left", padx=(6, 0)
        )
        self.save_button = ttk.Button(bar, text="Save", command=self._save_configuration)
        self.save_button.pack(side="left", padx=(6, 0))
        self.save_as_button = ttk.Button(
            bar, text="Save As...", command=self._save_configuration_as
        )
        self.save_as_button.pack(side="left", padx=(6, 0))
        ttk.Label(bar, text="Parameter set", style="Muted.TLabel").pack(
            side="left", padx=(18, 6)
        )
        self.parameter_number_combo = ttk.Combobox(
            bar, textvariable=self.parameter_number_var, state="readonly", width=4
        )
        self.parameter_number_combo.pack(side="left")
        self.parameter_number_combo.bind(
            "<<ComboboxSelected>>", self._parameter_set_changed
        )
        ttk.Label(bar, textvariable=self.file_var, style="Muted.TLabel").pack(
            side="left", padx=(18, 0), fill="x", expand=True
        )

    def _build_parameter_editor(self, parent: ttk.Frame) -> None:
        header = ttk.Frame(parent, style="Panel.TFrame")
        header.pack(fill="x", padx=12, pady=(12, 4))
        ttk.Label(
            header,
            text="setting.json audio parameters",
            style="Panel.TLabel",
            font=("Segoe UI Semibold", 12),
        ).pack(side="left")

        notebook = ttk.Notebook(parent)
        notebook.pack(fill="both", expand=True, padx=12, pady=(4, 12))
        for group_name, names in PARAMETER_GROUPS:
            page = ttk.Frame(notebook, style="Panel.TFrame")
            notebook.add(page, text=group_name)
            page.grid_columnconfigure(1, weight=1)
            for row, name in enumerate(names):
                spec = MODEL_PARAMETER_SPECS[name]
                display_name = (
                    f"{name} (runtime only)"
                    if name not in PARAMETER_SPECS
                    else name
                )
                ttk.Label(page, text=display_name, style="Panel.TLabel").grid(
                    row=row,
                    column=0,
                    sticky="w",
                    padx=(14, 10),
                    pady=9,
                )
                if spec.kind == "bool":
                    variable: tk.Variable = tk.BooleanVar(value=bool(spec.default))
                    widget = ttk.Checkbutton(page, variable=variable)
                else:
                    variable = tk.StringVar(value=self._format_editor_value(spec.default))
                    widget = ttk.Entry(page, textvariable=variable, width=18)
                self.parameter_vars[name] = variable
                widget.grid(row=row, column=1, sticky="ew", padx=(0, 10), pady=7)
                if spec.kind in ("uint", "float"):
                    range_text = f"{spec.minimum:g} .. {spec.maximum:g}"
                else:
                    range_text = "true / false"
                ttk.Label(page, text=range_text, style="Panel.TLabel").grid(
                    row=row, column=2, sticky="w", padx=(0, 14), pady=7
                )
                variable.trace_add("write", self._parameter_changed)

        if set(self.parameter_vars) != set(AUDIO_PARAMETER_NAMES):
            raise RuntimeError("parameter editor does not cover every audio parameter")

    def _build_simulator(self, parent: ttk.Frame) -> None:
        controls = ttk.Labelframe(parent, text="Simulation input")
        controls.pack(fill="x", padx=12, pady=12)
        controls.grid_columnconfigure(1, weight=1)

        ttk.Label(controls, text="Climb rate", style="Panel.TLabel").grid(
            row=0, column=0, sticky="w", padx=10, pady=(12, 4)
        )
        spinbox = ttk.Spinbox(
            controls,
            from_=CLIMB_RATE_MIN_MPS,
            to=CLIMB_RATE_MAX_MPS,
            increment=0.05,
            textvariable=self.rate_entry_var,
            width=10,
            command=self._rate_from_entry,
        )
        spinbox.grid(row=0, column=2, sticky="e", padx=10, pady=(12, 4))
        spinbox.bind("<Return>", self._rate_from_entry)
        spinbox.bind("<FocusOut>", self._rate_from_entry)
        ttk.Label(controls, text="m/s", style="Panel.TLabel").grid(
            row=0, column=3, sticky="w", padx=(0, 10), pady=(12, 4)
        )
        ttk.Scale(
            controls,
            from_=CLIMB_RATE_MIN_MPS,
            to=CLIMB_RATE_MAX_MPS,
            variable=self.rate_scale_var,
            command=self._rate_from_scale,
        ).grid(row=1, column=0, columnspan=4, sticky="ew", padx=10, pady=(2, 12))

        ttk.Label(controls, text="PC volume", style="Panel.TLabel").grid(
            row=2, column=0, sticky="w", padx=10, pady=(6, 4)
        )
        ttk.Scale(
            controls,
            from_=0.0,
            to=100.0,
            variable=self.volume_var,
            command=self._volume_changed,
        ).grid(row=2, column=1, columnspan=2, sticky="ew", padx=(4, 8), pady=(6, 4))
        ttk.Label(controls, textvariable=self.volume_text_var, style="Panel.TLabel").grid(
            row=2, column=3, sticky="e", padx=(0, 10), pady=(6, 4)
        )

        buttons = ttk.Frame(controls, style="Panel.TFrame")
        buttons.grid(row=3, column=0, columnspan=4, sticky="ew", padx=10, pady=(8, 12))
        self.start_button = ttk.Button(
            buttons, text="Start", style="Accent.TButton", command=self._start
        )
        self.start_button.pack(side="left")
        self.stop_button = ttk.Button(
            buttons, text="Stop", style="Danger.TButton", command=self._stop
        )
        self.stop_button.pack(side="left", padx=(6, 0))
        ttk.Button(buttons, text="Reset state", command=self._reset_simulation).pack(
            side="left", padx=(6, 0)
        )
        ttk.Label(buttons, textvariable=self.run_status_var, style="Panel.TLabel").pack(
            side="right"
        )

        status = ttk.Labelframe(parent, text="Firmware-compatible state")
        status.pack(fill="both", expand=True, padx=12, pady=(0, 12))
        rows = (
            ("Mode", self.mode_var),
            ("Output phase", self.output_var),
            ("Frequency", self.frequency_var),
            ("Virtual altitude", self.altitude_var),
            ("Input climb rate", self.instant_rate_var),
            ("Audio average rate", self.average_rate_var),
        )
        for row, (label, variable) in enumerate(rows):
            ttk.Label(status, text=label, style="Panel.TLabel").grid(
                row=row, column=0, sticky="w", padx=12, pady=10
            )
            value_label = tk.Label(
                status,
                textvariable=variable,
                bg=COLOR_PANEL,
                fg=COLOR_ACCENT if row < 3 else COLOR_TEXT,
                font=("Consolas", 12 if row < 3 else 10),
                anchor="e",
            )
            value_label.grid(row=row, column=1, sticky="ew", padx=12, pady=10)
        status.grid_columnconfigure(1, weight=1)

        note = tk.Label(
            parent,
            text=(
                "The volume slider is PC playback gain and is not saved. "
                "audio_duty_percent changes the rectangular waveform; "
                "Runtime Amplifier mode approximates the PAM8904E 1x/2x/3x levels."
            ),
            bg=COLOR_PANEL,
            fg=COLOR_MUTED,
            wraplength=430,
            justify="left",
        )
        note.pack(fill="x", padx=16, pady=(0, 14))

    @staticmethod
    def _format_editor_value(value: Any) -> str:
        if isinstance(value, float):
            return format(value, ".9g")
        return str(value)

    def _load_values_into_editor(self, values: dict[str, Any]) -> None:
        self.loading_editor = True
        try:
            for name, variable in self.parameter_vars.items():
                value = values[name]
                if isinstance(variable, tk.BooleanVar):
                    variable.set(bool(value))
                else:
                    variable.set(self._format_editor_value(value))
        finally:
            self.loading_editor = False
        self.values = validate_parameters(values)
        self.editor_valid = True
        self._set_validation("Configuration is valid", True)

    def _parameter_changed(self, *_args: Any) -> None:
        if self.loading_editor:
            return
        if self.validation_after is not None:
            self.root.after_cancel(self.validation_after)
        self.validation_after = self.root.after(120, self._validate_editor)

    def _validate_editor(self) -> None:
        self.validation_after = None
        candidate = dict(self.values)
        try:
            for name, variable in self.parameter_vars.items():
                spec = MODEL_PARAMETER_SPECS[name]
                if spec.kind == "bool":
                    candidate[name] = bool(variable.get())
                    continue
                text = str(variable.get()).strip()
                if not text:
                    raise ConfigError(f"{name}: value is empty")
                if spec.kind == "uint":
                    if any(character in text for character in ".eE"):
                        raise ConfigError(f"{name}: expected integer")
                    candidate[name] = int(text, 10)
                else:
                    candidate[name] = float(text)
            self.values = validate_parameters(candidate)
        except (ConfigError, ValueError) as exc:
            self.editor_valid = False
            self._set_validation(str(exc), False)
        else:
            self.editor_valid = True
            self.document.update_profile(self.active_parameter_number, self.values)
            self.dirty = (
                self.saved_document is None
                or self.document != self.saved_document
            )
            self._set_validation("Configuration is valid; preview updated", True)
            self._update_title()
        self._update_save_controls()

    def _set_validation(self, text: str, valid: bool) -> None:
        self.validation_var.set(text)
        self.validation_label.configure(fg=COLOR_GREEN if valid else COLOR_RED)

    def _update_save_controls(self) -> None:
        state = "normal" if self.editor_valid else "disabled"
        self.save_button.configure(state=state)
        self.save_as_button.configure(state=state)

    def _confirm_discard(self) -> bool:
        if not self.dirty:
            return True
        return messagebox.askyesno(
            APP_TITLE, "Discard the current unsaved parameter changes?"
        )

    def _new_configuration(self) -> None:
        if not self._confirm_discard():
            return
        self.current_path = None
        self.document = default_config_document()
        values = self.document.effective_parameters(1)
        self.saved_document = None
        self.active_parameter_number = 1
        self.dirty = True
        self.file_var.set("New version 1 configuration")
        self._refresh_parameter_number_selector()
        self._load_values_into_editor(values)
        self._reset_simulation()
        self._update_title()
        self._update_save_controls()

    def _open_configuration(self) -> None:
        if not self._confirm_discard():
            return
        filename = filedialog.askopenfilename(
            title="Open setting.json",
            filetypes=(("JSON files", "*.json"), ("All files", "*.*")),
        )
        if not filename:
            return
        try:
            document = load_config_document_file(filename)
        except ConfigError as exc:
            messagebox.showerror(APP_TITLE, f"Could not open configuration:\n{exc}")
            return
        self.current_path = Path(filename)
        self.document = ConfigDocument(
            dict(document.mc_parameters),
            {number: dict(values) for number, values in document.vario_parameter_sets.items()}
        )
        self.saved_document = ConfigDocument(
            dict(document.mc_parameters),
            {number: dict(values) for number, values in document.vario_parameter_sets.items()}
        )
        self.active_parameter_number = document.sorted_numbers()[0]
        self.dirty = False
        self.file_var.set(str(self.current_path))
        self._refresh_parameter_number_selector()
        self._load_values_into_editor(
            self.document.effective_parameters(self.active_parameter_number)
        )
        self._reset_simulation()
        self._update_title()
        self._update_save_controls()

    def _save_configuration(self) -> None:
        self._validate_editor()
        if not self.editor_valid:
            return
        if self.current_path is None:
            self._save_configuration_as()
            return
        if not messagebox.askyesno(
            APP_TITLE,
            f"Overwrite this file?\n\n{self.current_path}\n\n"
            "Non-audio parameter values will be preserved.",
        ):
            return
        self._save_to_path(self.current_path)

    def _save_configuration_as(self) -> None:
        self._validate_editor()
        if not self.editor_valid:
            return
        initial_name = self.current_path.name if self.current_path else "setting.json"
        filename = filedialog.asksaveasfilename(
            title="Save setting.json",
            defaultextension=".json",
            initialfile=initial_name,
            confirmoverwrite=True,
            filetypes=(("JSON files", "*.json"), ("All files", "*.*")),
        )
        if not filename:
            return
        self._save_to_path(Path(filename))

    def _save_to_path(self, path: Path) -> None:
        try:
            self.document.update_profile(self.active_parameter_number, self.values)
            save_config_document_file(path, self.document)
        except ConfigError as exc:
            messagebox.showerror(APP_TITLE, f"Could not save configuration:\n{exc}")
            return
        self.current_path = path
        self.saved_document = ConfigDocument(
            dict(self.document.mc_parameters),
            {
                number: dict(values)
                for number, values in self.document.vario_parameter_sets.items()
            }
        )
        self.dirty = False
        self.file_var.set(str(path))
        self._set_validation("Saved and verified as format_version 1", True)
        self._update_title()

    def _refresh_parameter_number_selector(self) -> None:
        numbers = tuple(str(number) for number in self.document.sorted_numbers())
        self.parameter_number_combo.configure(values=numbers)
        self.parameter_number_var.set(str(self.active_parameter_number))

    def _parameter_set_changed(self, _event: Any = None) -> None:
        self._validate_editor()
        if not self.editor_valid:
            self.parameter_number_var.set(str(self.active_parameter_number))
            return
        requested = int(self.parameter_number_var.get())
        if requested == self.active_parameter_number:
            return
        self.document.update_profile(self.active_parameter_number, self.values)
        self.active_parameter_number = requested
        self._load_values_into_editor(
            self.document.effective_parameters(requested)
        )
        self._reset_simulation()

    def _update_title(self) -> None:
        name = self.current_path.name if self.current_path else "Untitled"
        marker = " *" if self.dirty else ""
        self.root.title(f"{APP_TITLE} — {name}{marker}")

    def _rate_from_scale(self, value: str) -> None:
        rate = min(max(float(value), CLIMB_RATE_MIN_MPS), CLIMB_RATE_MAX_MPS)
        self.climb_rate_mps = rate
        self.rate_entry_var.set(f"{rate:.2f}")

    def _rate_from_entry(self, _event: Any = None) -> None:
        try:
            value = float(self.rate_entry_var.get().strip())
        except ValueError:
            self.rate_entry_var.set(f"{self.climb_rate_mps:.2f}")
            return
        if not math.isfinite(value):
            self.rate_entry_var.set(f"{self.climb_rate_mps:.2f}")
            return
        value = min(max(value, CLIMB_RATE_MIN_MPS), CLIMB_RATE_MAX_MPS)
        self.climb_rate_mps = value
        self.rate_scale_var.set(value)
        self.rate_entry_var.set(f"{value:.2f}")

    def _volume_changed(self, value: str) -> None:
        self.volume_text_var.set(f"{float(value):.0f} %")

    def _start(self) -> None:
        if self.running:
            return
        try:
            self.audio_engine.start()
        except Exception as exc:
            messagebox.showerror(APP_TITLE, f"Could not start audio output:\n{exc}")
            self.audio_status_var.set("Audio unavailable")
            return
        self.running = True
        self.last_tick_s = time.monotonic()
        self.run_status_var.set("Running")
        self.audio_status_var.set("48 kHz mono — default output device")

    def _stop(self) -> None:
        self.running = False
        reset_audio_state(self.audio_state)
        self.command = VarioAudioCommand()
        self.audio_engine.update(self.command, self.volume_var.get() / 100.0)
        self.run_status_var.set("Stopped")
        self._update_state_display()

    def _reset_simulation(self) -> None:
        self.virtual_altitude_m = 0.0
        reset_audio_state(self.audio_state)
        self.command = VarioAudioCommand()
        self.last_tick_s = time.monotonic()
        self.audio_engine.update(self.command, self.volume_var.get() / 100.0)
        self._update_state_display()

    def _tick(self) -> None:
        now_s = time.monotonic()
        if self.running:
            elapsed_s = max(0.0, now_s - self.last_tick_s)
            self.virtual_altitude_m += self.climb_rate_mps * elapsed_s
            self.last_tick_s = now_s
            sample = VarioSample(
                timestamp_s=now_s,
                altitude_m=self.virtual_altitude_m,
                climb_rate_mps=self.climb_rate_mps,
            )
            self.command = vario_audio_step(
                self.audio_state, self.values, sample, now_s
            )
        else:
            self.last_tick_s = now_s
            self.command = VarioAudioCommand()
        self.audio_engine.update(self.command, self.volume_var.get() / 100.0)
        self._update_state_display()
        callback_error = self.audio_engine.callback_error
        if callback_error:
            self.audio_status_var.set(f"Audio warning: {callback_error}")
        self.root.after(SIMULATION_PERIOD_MS, self._tick)

    def _update_state_display(self) -> None:
        self.mode_var.set(self.command.mode.value)
        self.output_var.set("ON" if self.command.sounding else "OFF")
        self.frequency_var.set(
            f"{self.command.frequency_hz} Hz" if self.command.frequency_hz else "-- Hz"
        )
        self.altitude_var.set(f"{self.virtual_altitude_m:.3f} m")
        self.instant_rate_var.set(f"{self.climb_rate_mps:.3f} m/s")
        if self.audio_state.averaged_climb_rate_valid:
            self.average_rate_var.set(
                f"{self.audio_state.averaged_climb_rate_mps:.3f} m/s"
            )
        else:
            self.average_rate_var.set("-- m/s")

    def _close(self) -> None:
        if not self._confirm_discard():
            return
        self.running = False
        self.audio_engine.close()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    VarioSoundSimulatorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
