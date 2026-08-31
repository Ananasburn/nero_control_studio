"""Firmware drag-teach controls (CAN 0x150 Byte 2)."""

from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QGroupBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from nero_control_studio.arm.drag_teach import DragTeachCmd
from nero_control_studio.arm.status import ArmSnapshot
from nero_control_studio.ui.styles import set_widget_active


class DragTeachPanel(QGroupBox):
    command_requested = Signal(int)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__("固件拖动示教", parent)
        root = QVBoxLayout(self)
        hint = QLabel(
            "主控录制/复现拖动轨迹（协议 0x150）。与点动、路点回放互斥。"
            "若点「开始记录」后状态不变，请改用上方软件路点。"
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #9aa0a6; font-weight: 400;")
        root.addWidget(hint)

        row1 = QHBoxLayout()
        row2 = QHBoxLayout()
        self._buttons: dict[DragTeachCmd, QPushButton] = {}
        specs = [
            (row1, DragTeachCmd.START_RECORD, "开始记录"),
            (row1, DragTeachCmd.STOP_RECORD, "结束记录"),
            (row1, DragTeachCmd.GOTO_START, "去起点"),
            (row2, DragTeachCmd.EXECUTE, "复现"),
            (row2, DragTeachCmd.PAUSE, "暂停"),
            (row2, DragTeachCmd.RESUME, "继续"),
            (row2, DragTeachCmd.TERMINATE, "终止"),
        ]
        for row, cmd, label in specs:
            btn = QPushButton(label)
            btn.clicked.connect(lambda _=False, c=cmd: self.command_requested.emit(int(c)))
            row.addWidget(btn)
            self._buttons[cmd] = btn
        root.addLayout(row1)
        root.addLayout(row2)
        self._status = QLabel("示教状态: —")
        root.addWidget(self._status)

    def apply_snapshot(self, snap: ArmSnapshot) -> None:
        enabled = snap.connected and snap.enabled and not snap.leader_mode and not snap.playing
        for cmd, btn in self._buttons.items():
            btn.setEnabled(enabled)
            set_widget_active(btn, snap.teach_status == int(cmd) and int(cmd) != 0)
        self._status.setText(f"示教状态: {snap.teach_status_label}")
