"""Application entrypoint."""

from __future__ import annotations

import argparse
import sys

from nero_control_studio.constants import DEFAULT_CHANNEL, DEFAULT_INTERFACE


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Nero Control Studio")
    parser.add_argument(
        "--channel",
        default=DEFAULT_CHANNEL,
        help=f"CAN channel (default: {DEFAULT_CHANNEL})",
    )
    parser.add_argument(
        "--interface",
        default=DEFAULT_INTERFACE,
        help=f"python-can interface (default: {DEFAULT_INTERFACE})",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        from PySide6.QtWidgets import QApplication
    except ImportError:
        print("PySide6 is required. pip install -e '.[dev]'", file=sys.stderr)
        return 1
    try:
        import pyAgxArm  # noqa: F401
    except ImportError:
        print(
            "pyAgxArm is required. pip install -e /home/bci/Projects/pyAgxArm",
            file=sys.stderr,
        )
        return 1

    from nero_control_studio.ui.main_window import MainWindow
    from nero_control_studio.ui.styles import APP_STYLESHEET

    app = QApplication([sys.argv[0]])
    app.setApplicationName("Nero Control Studio")
    app.setStyleSheet(APP_STYLESHEET)
    window = MainWindow(channel=args.channel, interface=args.interface)
    window.show()
    return app.exec()
