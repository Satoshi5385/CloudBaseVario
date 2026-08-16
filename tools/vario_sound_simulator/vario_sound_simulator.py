"""Command-line entry point for the CloudBaseVario sound simulator."""

try:
    from .vario_sound_simulator_app import (
        CLIMB_RATE_MAX_MPS,
        CLIMB_RATE_MIN_MPS,
        VarioSoundSimulatorApp,
        main,
    )
except ImportError:
    from vario_sound_simulator_app import (
        CLIMB_RATE_MAX_MPS,
        CLIMB_RATE_MIN_MPS,
        VarioSoundSimulatorApp,
        main,
    )

__all__ = [
    "CLIMB_RATE_MAX_MPS",
    "CLIMB_RATE_MIN_MPS",
    "VarioSoundSimulatorApp",
    "main",
]


if __name__ == "__main__":
    main()
