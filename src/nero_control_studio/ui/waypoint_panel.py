"""Software waypoint teaching: record, reorder, play, JSON."""

from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from nero_control_studio.arm.status import ArmSnapshot
from nero_control_studio.constants import RAD2DEG
from nero_control_studio.ui.styles import set_widget_active


class WaypointPanel(QGroupBox):
    record_requested = Signal()
    delete_requested = Signal(int)
    move_requested = Signal(int, int)
    goto_requested = Signal(int)
    play_requested = Signal()
    stop_requested = Signal()
    save_requested = Signal(str)
    load_requested = Signal(str)
    leader_toggled = Signal(bool)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__("软件路点示教", parent)
        root = QVBoxLayout(self)

        row1 = QHBoxLayout()
        self._leader_btn = QPushButton("零力拖动")
        self._leader_btn.setObjectName("leaderButton")
        self._leader_btn.setCheckable(True)
        self._leader_btn.setToolTip("打开后电机进入零力，可手掰到位；再点一次退出")
        self._record_btn = QPushButton("记录路点")
        self._delete_btn = QPushButton("删除")
        self._up_btn = QPushButton("上移")
        self._down_btn = QPushButton("下移")
        row1.addWidget(self._leader_btn)
        row1.addWidget(self._record_btn)
        row1.addWidget(self._delete_btn)
        row1.addWidget(self._up_btn)
        row1.addWidget(self._down_btn)
        root.addLayout(row1)

        self._list = QListWidget()
        root.addWidget(self._list)

        row2 = QHBoxLayout()
        self._goto_btn = QPushButton("运动到选中")
        self._play_btn = QPushButton("顺序回放")
        self._play_btn.setObjectName("playButton")
        self._stop_btn = QPushButton("停止回放")
        self._save_btn = QPushButton("保存")
        self._load_btn = QPushButton("加载")
        row2.addWidget(self._goto_btn)
        row2.addWidget(self._play_btn)
        row2.addWidget(self._stop_btn)
        row2.addWidget(self._save_btn)
        row2.addWidget(self._load_btn)
        root.addLayout(row2)

        self._leader_btn.toggled.connect(self.leader_toggled)
        self._record_btn.clicked.connect(self.record_requested)
        self._delete_btn.clicked.connect(self._delete)
        self._up_btn.clicked.connect(lambda: self._move(-1))
        self._down_btn.clicked.connect(lambda: self._move(1))
        self._goto_btn.clicked.connect(self._goto)
        self._play_btn.clicked.connect(self.play_requested)
        self._stop_btn.clicked.connect(self.stop_requested)
        self._save_btn.clicked.connect(self._save)
        self._load_btn.clicked.connect(self._load)

        self._mode_hint = QLabel("当前：未进入零力 / 回放")
        self._mode_hint.setWordWrap(True)
        self._mode_hint.setStyleSheet("color: #9aa0a6; font-weight: 400;")
        root.addWidget(self._mode_hint)

    def _selected_index(self) -> int:
        items = self._list.selectedIndexes()
        return items[0].row() if items else -1

    def _delete(self) -> None:
        index = self._selected_index()
        if index >= 0:
            self.delete_requested.emit(index)

    def _move(self, delta: int) -> None:
        index = self._selected_index()
        if index >= 0:
            self.move_requested.emit(index, delta)

    def _goto(self) -> None:
        index = self._selected_index()
        if index >= 0:
            self.goto_requested.emit(index)

    def _save(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "保存路点", "waypoints.json", "JSON (*.json)"
        )
        if path:
            self.save_requested.emit(path)

    def _load(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "加载路点", "", "JSON (*.json)"
        )
        if path:
            self.load_requested.emit(path)

    def set_waypoints(self, payload: list[dict]) -> None:
        current = self._selected_index()
        self._list.clear()
        for i, item in enumerate(payload):
            joints = item.get("joints", [])
            deg = " ".join(f"{v * RAD2DEG:6.1f}" for v in joints)
            row = QListWidgetItem(f"{item.get('name', f'WP{i+1}'):6s}  {deg} °")
            self._list.addItem(row)
        if 0 <= current < self._list.count():
            self._list.setCurrentRow(current)

    def apply_snapshot(self, snap: ArmSnapshot) -> None:
        connected = snap.connected and snap.enabled
        self._leader_btn.setEnabled(connected and not snap.playing)
        self._leader_btn.blockSignals(True)
        self._leader_btn.setChecked(snap.leader_mode)
        self._leader_btn.setText("零力拖动 · 开" if snap.leader_mode else "零力拖动")
        self._leader_btn.blockSignals(False)
        self._record_btn.setEnabled(snap.can_teach)
        self._goto_btn.setEnabled(snap.can_jog and self._list.count() > 0)
        self._play_btn.setEnabled((snap.can_jog or snap.playing) and self._list.count() > 0)
        set_widget_active(self._play_btn, snap.playing)
        self._stop_btn.setEnabled(snap.playing)
        if snap.leader_mode:
            self._mode_hint.setText("当前：零力拖动中（按钮为黄色）。手掰到位后点「记录路点」。")
        elif snap.playing:
            self._mode_hint.setText(f"当前：顺序回放第 {snap.play_index + 1} 点")
        else:
            self._mode_hint.setText("当前：未进入零力 / 回放")
        if snap.playing and 0 <= snap.play_index < self._list.count():
            self._list.setCurrentRow(snap.play_index)
