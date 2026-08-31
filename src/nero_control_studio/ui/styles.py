"""Dark industrial stylesheet for the teaching pendant."""

from __future__ import annotations

from PySide6.QtWidgets import QWidget

APP_STYLESHEET = """
QMainWindow, QDialog, QWidget {
    background: #1b1e24;
    color: #e8eaed;
    font-size: 13px;
}
QGroupBox {
    border: 1px solid #3a404c;
    border-radius: 6px;
    margin-top: 14px;
    padding: 10px 8px 8px 8px;
    font-weight: 600;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
    color: #9aa0a6;
}
QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QListWidget, QPlainTextEdit {
    background: #12141a;
    border: 1px solid #3a404c;
    border-radius: 4px;
    padding: 4px 6px;
    color: #e8eaed;
    selection-background-color: #3d5afe;
}
QComboBox QAbstractItemView {
    background: #12141a;
    color: #e8eaed;
}
QPushButton {
    background: #2b313c;
    border: 1px solid #4a5160;
    border-radius: 4px;
    padding: 6px 12px;
    min-height: 28px;
}
QPushButton:hover { background: #394153; }
QPushButton:pressed { background: #252a33; }
QPushButton:disabled { color: #6b7280; background: #22262e; border-color: #333; }
QPushButton:checked,
QPushButton[active="true"] {
    background: #ef6c00;
    color: #fff8e1;
    border: 2px solid #ffcc80;
    font-weight: 700;
}
QPushButton#connectButton { background: #1b5e20; }
QPushButton#connectButton[active="true"],
QPushButton#connectButton[active="true"]:disabled {
    background: #2e7d32;
    border: 2px solid #81c784;
    color: white;
    font-weight: 700;
}
QPushButton#enableButton { background: #1565c0; }
QPushButton#enableButton[active="true"],
QPushButton#enableButton[active="true"]:disabled {
    background: #0277bd;
    border: 2px solid #81d4fa;
    color: white;
    font-weight: 700;
}
QPushButton#leaderButton:checked {
    background: #f9a825;
    color: #1b1e24;
    border: 2px solid #ffe082;
}
QPushButton#playButton[active="true"] {
    background: #1565c0;
    border: 2px solid #90caf9;
    color: white;
}
QPushButton#estopButton {
    background: #c62828;
    color: white;
    font-size: 18px;
    font-weight: 700;
    min-height: 48px;
    min-width: 120px;
    border: 2px solid #ff8a80;
}
QPushButton#estopButton:pressed { background: #8e0000; }
QRadioButton { spacing: 6px; }
QRadioButton::indicator {
    width: 14px;
    height: 14px;
    border-radius: 7px;
    border: 2px solid #9aa0a6;
    background: #12141a;
}
QRadioButton::indicator:checked {
    background: #f9a825;
    border: 2px solid #ffe082;
}
QRadioButton:checked { color: #ffe082; font-weight: 700; }
QSlider::groove:horizontal {
    height: 8px;
    background: #12141a;
    border-radius: 4px;
}
QSlider::handle:horizontal {
    width: 16px;
    margin: -5px 0;
    background: #90caf9;
    border-radius: 8px;
}
QSlider::sub-page:horizontal { background: #1565c0; border-radius: 4px; }
QLabel#statusValue { color: #80cbc4; }
QLabel#warningValue { color: #ffcc80; }
QLabel#errorValue { color: #ef9a9a; }
QLabel#activityBanner {
    font-size: 16px;
    font-weight: 700;
    padding: 8px 12px;
    border-radius: 6px;
    min-height: 28px;
}
QPlainTextEdit#logView {
    font-family: monospace;
    font-size: 12px;
}
"""

ACTIVITY_BANNER_COLORS = {
    "idle": ("#9aa0a6", "#2b313c"),
    "ready": ("#c8e6c9", "#1b5e20"),
    "leader": ("#1b1e24", "#f9a825"),
    "play": ("#e3f2fd", "#1565c0"),
    "teach": ("#fce4ec", "#ad1457"),
    "jog": ("#e0f7fa", "#00838f"),
    "warn": ("#ffe0b2", "#e65100"),
    "error": ("#ffcdd2", "#b71c1c"),
}


def set_widget_active(widget: QWidget, active: bool) -> None:
    """Toggle a QSS dynamic property and force a restyle."""
    current = widget.property("active")
    if current is True or current == "true":
        was = True
    elif current is False or current == "false" or current is None:
        was = False
    else:
        was = bool(current)
    if was == bool(active):
        return
    widget.setProperty("active", bool(active))
    style = widget.style()
    style.unpolish(widget)
    style.polish(widget)
    widget.update()
