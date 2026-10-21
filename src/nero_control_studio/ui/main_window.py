"""Main teaching-pendant window."""

from __future__ import annotations

from PySide6.QtCore import QMetaObject, QThread, QTimer, Qt, Signal, Slot
from PySide6.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSlider,
    QSplitter,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from nero_control_studio.arm.status import ArmSnapshot
from nero_control_studio.constants import DEFAULT_CHANNEL, DEFAULT_CHANNELS, DEFAULT_SPEED_PERCENT
from nero_control_studio.handeye.config import HandeyeRunConfig, SOLVE_TARGET_RIGHT_BASE
from nero_control_studio.handeye.joint_names import arm_from_channel
from nero_control_studio.handeye.session import HandeyeSession
from nero_control_studio.ui.drag_teach_panel import DragTeachPanel
from nero_control_studio.ui.handeye_page import HandeyePage
from nero_control_studio.ui.joint_jog_panel import JointJogPanel
from nero_control_studio.ui.styles import ACTIVITY_BANNER_COLORS, set_widget_active
from nero_control_studio.ui.waypoint_panel import WaypointPanel
from nero_control_studio.worker import ArmWorker


class MainWindow(QMainWindow):
    _connect_requested = Signal(str, str)
    _speed_requested = Signal(int)

    def __init__(self, channel: str = DEFAULT_CHANNEL, interface: str = "socketcan") -> None:
        super().__init__()
        self.setWindowTitle("Nero Control Studio")
        self.resize(1400, 900)
        self._interface = interface
        self._closing = False
        self._last_snap: ArmSnapshot | None = None

        self._thread = QThread(self)
        self._worker = ArmWorker()
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.start)

        self._handeye = HandeyeSession(self)
        self._image_timer = QTimer(self)
        self._image_timer.setInterval(120)
        self._image_timer.timeout.connect(self._refresh_handeye_image)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        layout.addLayout(self._build_toolbar(channel))
        layout.addWidget(self._build_activity_banner())
        layout.addLayout(self._build_status_row())

        body = QHBoxLayout()
        self._nav = QListWidget()
        self._nav.setObjectName("navSidebar")
        self._nav.setFixedWidth(112)
        self._nav.addItem(QListWidgetItem("示教"))
        self._nav.addItem(QListWidgetItem("手眼标定"))
        self._nav.setCurrentRow(0)
        body.addWidget(self._nav)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        self._jog = JointJogPanel()
        self._pages = QStackedWidget()
        teach = QWidget()
        teach_layout = QVBoxLayout(teach)
        teach_layout.setContentsMargins(0, 0, 0, 0)
        self._waypoints = WaypointPanel()
        self._drag = DragTeachPanel()
        teach_layout.addWidget(self._waypoints, 3)
        teach_layout.addWidget(self._drag, 2)
        self._handeye_page = HandeyePage()
        self._pages.addWidget(teach)
        self._pages.addWidget(self._handeye_page)
        splitter.addWidget(self._jog)
        splitter.addWidget(self._pages)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)
        splitter.setSizes([420, 760])
        body.addWidget(splitter, 1)
        layout.addLayout(body, 1)

        self._log = QPlainTextEdit()
        self._log.setObjectName("logView")
        self._log.setReadOnly(True)
        self._log.setMaximumHeight(140)
        layout.addWidget(self._log)

        self._handeye_page.set_channel(channel)
        self._handeye_solve_target = "camera"
        self._connect_signals()
        self._thread.start()

    def _build_toolbar(self, channel: str) -> QHBoxLayout:
        row = QHBoxLayout()
        row.addWidget(QLabel("通道"))
        self._channel = QComboBox()
        self._channel.setEditable(True)
        for name in DEFAULT_CHANNELS:
            self._channel.addItem(name)
        if channel not in DEFAULT_CHANNELS:
            self._channel.addItem(channel)
        self._channel.setCurrentText(channel)
        row.addWidget(self._channel)

        self._connect_btn = QPushButton("连接")
        self._connect_btn.setObjectName("connectButton")
        self._disconnect_btn = QPushButton("断开")
        self._enable_btn = QPushButton("使能")
        self._enable_btn.setObjectName("enableButton")
        self._disable_btn = QPushButton("失能")
        self._reset_btn = QPushButton("复位")
        row.addWidget(self._connect_btn)
        row.addWidget(self._disconnect_btn)
        row.addWidget(self._enable_btn)
        row.addWidget(self._disable_btn)
        row.addWidget(self._reset_btn)

        row.addWidget(QLabel("速度 %"))
        self._speed = QSlider(Qt.Orientation.Horizontal)
        self._speed.setRange(1, 100)
        self._speed.setValue(DEFAULT_SPEED_PERCENT)
        self._speed.setFixedWidth(160)
        self._speed_label = QLabel(str(DEFAULT_SPEED_PERCENT))
        row.addWidget(self._speed)
        row.addWidget(self._speed_label)
        row.addStretch(1)

        self._estop = QPushButton("急停")
        self._estop.setObjectName("estopButton")
        row.addWidget(self._estop)
        return row

    def _build_activity_banner(self) -> QLabel:
        self._activity = QLabel("当前功能：未连接")
        self._activity.setObjectName("activityBanner")
        fg, bg = ACTIVITY_BANNER_COLORS["idle"]
        self._activity.setStyleSheet(f"color: {fg}; background: {bg};")
        return self._activity

    def _build_status_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        self._fw_label = QLabel("固件: —")
        self._mode_label = QLabel("控制: —")
        self._arm_label = QLabel("状态: —")
        self._teach_label = QLabel("示教: —")
        self._move_label = QLabel("运动: —")
        self._err_label = QLabel("故障: —")
        for widget in (
            self._fw_label,
            self._mode_label,
            self._arm_label,
            self._teach_label,
            self._move_label,
            self._err_label,
        ):
            widget.setObjectName("statusValue")
            row.addWidget(widget)
        self._err_label.setObjectName("errorValue")
        row.addStretch(1)
        return row

    def _connect_signals(self) -> None:
        self._connect_requested.connect(self._worker.connect_arm)
        self._speed_requested.connect(self._worker.set_speed_percent)
        self._connect_btn.clicked.connect(self._on_connect)
        self._disconnect_btn.clicked.connect(self._worker.disconnect_arm)
        self._enable_btn.clicked.connect(self._worker.enable_arm)
        self._disable_btn.clicked.connect(self._worker.disable_arm)
        self._reset_btn.clicked.connect(self._worker.reset_arm)
        self._estop.clicked.connect(self._worker.emergency_stop)
        self._speed.valueChanged.connect(self._on_speed)
        self._channel.currentTextChanged.connect(self._handeye_page.set_channel)
        self._nav.currentRowChanged.connect(self._pages.setCurrentIndex)

        self._jog.step_requested.connect(self._worker.jog_step)
        self._jog.hold_started.connect(self._worker.jog_hold_start)
        self._jog.hold_stopped.connect(self._worker.jog_hold_stop)
        self._jog.joint_target.connect(self._worker.jog_set_joint)
        self._jog.params_changed.connect(self._worker.set_jog_params)

        self._waypoints.record_requested.connect(self._worker.record_waypoint)
        self._waypoints.delete_requested.connect(self._worker.delete_waypoint)
        self._waypoints.move_requested.connect(self._worker.move_waypoint)
        self._waypoints.goto_requested.connect(self._worker.goto_waypoint)
        self._waypoints.play_requested.connect(self._worker.play_waypoints)
        self._waypoints.stop_requested.connect(self._worker.stop_playback)
        self._waypoints.save_requested.connect(self._worker.save_waypoints)
        self._waypoints.load_requested.connect(self._worker.load_waypoints)
        self._waypoints.leader_toggled.connect(self._worker.set_leader_mode)

        self._drag.command_requested.connect(self._worker.drag_teach)

        self._handeye_page.start_requested.connect(self._on_handeye_start)
        self._handeye_page.stop_requested.connect(self._handeye.stop)
        self._handeye_page.sample_requested.connect(self._handeye.sample)
        self._handeye_page.drop_requested.connect(self._handeye.drop)
        self._handeye_page.solve_requested.connect(self._handeye.solve)
        self._handeye_page.leader_toggled.connect(self._worker.set_leader_mode)

        self._handeye.log_message.connect(self._append_log)
        self._handeye.status_received.connect(self._handeye_page.apply_handeye_status)
        self._handeye.ready.connect(self._on_handeye_ready)
        self._handeye.failed.connect(self._on_handeye_failed)
        self._handeye.running_changed.connect(self._on_handeye_running)
        self._handeye.sample_finished.connect(self._on_handeye_sample)
        self._handeye.drop_finished.connect(self._on_handeye_drop)
        self._handeye.solve_finished.connect(self._on_handeye_solve)

        self._worker.status_updated.connect(self._on_status)
        self._worker.log_message.connect(self._append_log)
        self._worker.waypoints_changed.connect(self._waypoints.set_waypoints)
        self._worker.connection_finished.connect(self._on_connection_finished)

    def _on_connect(self) -> None:
        self._connect_btn.setEnabled(False)
        self._connect_requested.emit(self._channel.currentText().strip(), self._interface)

    def _on_speed(self, value: int) -> None:
        self._speed_label.setText(str(value))
        self._speed_requested.emit(value)

    @Slot(object)
    def _on_handeye_start(self, cfg: HandeyeRunConfig) -> None:
        self._nav.setCurrentRow(1)
        self._handeye_solve_target = cfg.solve_target
        self._handeye.start(cfg)

    @Slot(str)
    def _on_handeye_ready(self, path: str) -> None:
        self._handeye_page.image_view.set_image_path(path)
        self._image_timer.start()
        self._nav.setCurrentRow(1)

    @Slot(str)
    def _on_handeye_failed(self, detail: str) -> None:
        self._image_timer.stop()
        self._handeye_page.set_running(False)
        QMessageBox.warning(self, "手眼标定启动失败", detail)

    @Slot(bool)
    def _on_handeye_running(self, running: bool) -> None:
        self._handeye_page.set_running(running)
        if not running:
            self._image_timer.stop()
        if self._last_snap is not None:
            self._apply_banner(self._last_snap)

    @Slot(bool, str)
    def _on_handeye_sample(self, ok: bool, message: str) -> None:
        self._append_log(message)
        if not ok:
            QMessageBox.warning(self, "采样失败", message)

    @Slot(bool, str)
    def _on_handeye_drop(self, _ok: bool, message: str) -> None:
        self._append_log(message)

    @Slot(bool, str)
    def _on_handeye_solve(self, ok: bool, message: str) -> None:
        self._append_log(message)
        right = self._handeye_solve_target == SOLVE_TARGET_RIGHT_BASE
        if ok:
            title = "右基座求解完成" if right else "手眼求解完成"
            QMessageBox.information(self, title, message)
        else:
            title = "右基座求解失败" if right else "求解失败"
            QMessageBox.warning(self, title, message)

    def _refresh_handeye_image(self) -> None:
        self._handeye_page.image_view.refresh()

    @Slot(object)
    def _on_status(self, snap: ArmSnapshot) -> None:
        self._last_snap = snap
        fw = snap.software_version or "—"
        extra = f" ({snap.firmware})" if snap.firmware else ""
        self._fw_label.setText(f"固件: {fw}{extra}")
        self._mode_label.setText(f"控制: {snap.ctrl_mode_label}")
        status_text = snap.arm_status_label
        if snap.urdf_over_limit_text:
            status_text = f"{status_text} · {snap.urdf_over_limit_text}"
        self._arm_label.setText(f"状态: {status_text}")
        self._teach_label.setText(f"示教: {snap.teach_status_label}")
        moving = "到位" if snap.motion_status == 0 else "运动中"
        if snap.playing:
            moving = f"回放 {snap.play_index + 1}"
        self._move_label.setText(f"运动: {moving}")
        err = snap.err_text or ("无" if snap.connected else "—")
        if snap.urdf_over_limit_text:
            err = (
                f"{snap.err_text}; {snap.urdf_over_limit_text}"
                if snap.err_text
                else snap.urdf_over_limit_text
            )
        self._err_label.setText(f"故障: {err}")
        if snap.last_error and not snap.connected:
            self._err_label.setText(f"故障: {snap.last_error}")

        self._apply_banner(snap)

        connected = snap.connected
        self._connect_btn.setEnabled(not connected and not snap.connecting)
        set_widget_active(self._connect_btn, connected)
        self._connect_btn.setText("已连接" if connected else "连接")
        self._disconnect_btn.setEnabled(connected)
        self._enable_btn.setEnabled(connected and not snap.enabled)
        set_widget_active(self._enable_btn, connected and snap.enabled)
        self._enable_btn.setText("已使能" if snap.enabled else "使能")
        self._disable_btn.setEnabled(connected and snap.enabled)
        self._reset_btn.setEnabled(connected)
        self._channel.setEnabled(not connected)
        self._jog.apply_snapshot(snap)
        self._waypoints.apply_snapshot(snap)
        self._drag.apply_snapshot(snap)
        self._handeye_page.apply_snapshot(snap)

        if self._handeye.running and snap.have_joints:
            self._handeye.send_joints(arm_from_channel(snap.channel or self._channel.currentText()), snap.joints)

    def _apply_banner(self, snap: ArmSnapshot) -> None:
        kind = snap.activity_kind
        text = snap.activity_text
        if self._handeye.running and kind in ("ready", "jog", "leader"):
            right = self._handeye_solve_target == SOLVE_TARGET_RIGHT_BASE
            label = "右基座标定" if right else "手眼标定"
            kind = "handeye" if kind != "leader" else kind
            if kind == "leader":
                text = f"{label} · 零力拖动中，摆好后退出零力再采样"
            else:
                text = f"{label}中 · 可用点动或零力摆姿态"
        self._activity.setText(f"当前功能：{text}")
        fg, bg = ACTIVITY_BANNER_COLORS.get(kind, ACTIVITY_BANNER_COLORS["idle"])
        self._activity.setStyleSheet(
            "QLabel#activityBanner {"
            f" color: {fg}; background: {bg};"
            " font-size: 16px; font-weight: 700; padding: 8px 12px; border-radius: 6px;"
            "}"
        )

    @Slot(bool, str)
    def _on_connection_finished(self, ok: bool, detail: str) -> None:
        self._connect_btn.setEnabled(True)
        if not ok:
            QMessageBox.warning(self, "连接失败", detail)

    @Slot(str)
    def _append_log(self, text: str) -> None:
        self._log.appendPlainText(text)

    def closeEvent(self, event) -> None:  # type: ignore[override]
        if self._closing:
            event.accept()
            return
        self._closing = True
        self._image_timer.stop()
        self._append_log("正在断开…")
        if self._handeye.running:
            self._handeye.stop()
        if self._thread.isRunning():
            QMetaObject.invokeMethod(
                self._worker, "shutdown", Qt.ConnectionType.BlockingQueuedConnection
            )
            self._thread.quit()
            if not self._thread.wait(4000):
                self._thread.terminate()
                self._thread.wait(1000)
        else:
            self._worker.shutdown()
        event.accept()
