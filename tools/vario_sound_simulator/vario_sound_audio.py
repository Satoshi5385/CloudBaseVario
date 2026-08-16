"""Default-device audio transport for the vario sound simulator."""

from __future__ import annotations

import threading
from typing import Any

try:
    import numpy as np
    import sounddevice as sd
except Exception as exc:  # Keep callers importable so they can explain the fix.
    np = None
    sd = None
    AUDIO_IMPORT_ERROR: Exception | None = exc
else:
    AUDIO_IMPORT_ERROR = None

try:
    from .vario_sound_model import PwmWaveform, VarioAudioCommand
except ImportError:
    from vario_sound_model import PwmWaveform, VarioAudioCommand


SAMPLE_RATE = 48000
BLOCK_SIZE = 480


class AudioEngine:
    """Own the default-device output stream and its callback snapshot."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._stream: Any = None
        self._waveform = PwmWaveform(sample_rate=SAMPLE_RATE)
        self._command = VarioAudioCommand()
        self._volume = 0.35
        self._callback_error = ""

    @property
    def callback_error(self) -> str:
        with self._lock:
            return self._callback_error

    def start(self) -> None:
        if AUDIO_IMPORT_ERROR is not None or sd is None or np is None:
            raise RuntimeError(
                "Audio dependencies are unavailable. Install "
                "tools/vario_sound_simulator/requirements-vario-simulator.txt.\n"
                f"{AUDIO_IMPORT_ERROR}"
            )
        if self._stream is not None:
            return
        stream = sd.OutputStream(
            samplerate=SAMPLE_RATE,
            blocksize=BLOCK_SIZE,
            channels=1,
            dtype="float32",
            callback=self._audio_callback,
        )
        try:
            stream.start()
        except Exception:
            stream.close()
            raise
        self._stream = stream

    def update(self, command: VarioAudioCommand, volume: float) -> None:
        with self._lock:
            self._command = command
            self._volume = min(max(volume, 0.0), 1.0)

    def _audio_callback(
        self, outdata: Any, frames: int, time_info: Any, status: Any
    ) -> None:
        del time_info
        try:
            with self._lock:
                command = self._command
                volume = self._volume
                if status:
                    self._callback_error = str(status)
            samples = self._waveform.render(
                frames,
                sounding=command.sounding,
                frequency_hz=command.frequency_hz,
                duty_percent=command.duty_percent or 50,
                amplifier_mode=command.amplifier_mode or 1,
                volume=volume,
            )
            outdata[:, 0] = np.asarray(samples, dtype=np.float32)
        except Exception as exc:
            outdata.fill(0)
            with self._lock:
                self._callback_error = str(exc)

    def close(self) -> None:
        stream = self._stream
        self._stream = None
        if stream is not None:
            try:
                stream.stop()
            finally:
                stream.close()
        self._waveform.reset()
