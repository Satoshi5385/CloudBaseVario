"""Command-line entry point for the CloudBaseVario monitor."""

try:
    from .cloudbasevario_gui_app import CloudBaseVarioApp, main
except ImportError:
    from cloudbasevario_gui_app import CloudBaseVarioApp, main

__all__ = ["CloudBaseVarioApp", "main"]


if __name__ == "__main__":
    main()
