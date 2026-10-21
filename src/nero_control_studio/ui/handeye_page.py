"""Eye-to-hand calibration page: detection view, params, sample/solve."""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from nero_control_studio.arm.status import ArmSnapshot
from nero_control_studio.handeye.config import (
    ARUCO_DICT_NAMES,
    DEFAULT_MARKER_LENGTH,
    DEFAULT_TARGET_SAMPLES,
    MIN_SOLVE_SAMPLES,
    SOLVE_TARGET_CAMERA,
    SOLVE_TARGET_RIGHT_BASE,
    HandeyeRunConfig,
    default_camera_yaml_path,
    default_right_base_yaml_path,
    default_samples_path,
    default_yaml_path,
)
from nero_control_studio.handeye.joint_names import arm_from_channel, ee_frame_for_arm
from nero_control_studio.ui.debug_image_view import DebugImageView
from nero_control_studio.ui.styles import set_widget_active


class HandeyePage(QWidget):
    start_requested = Signal(object)
    stop_requested = Signal()
    sample_requested = Signal()
    drop_requested = Signal()
    solve_requested = Signal(str, str)
    leader_toggled = Signal(bool)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._running = False
        self._detected = False
        self._samples = 0
        self._channel = "can_left"

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)

        image_box = QGroupBox("检测画面")
        image_layout = QVBoxLayout(image_box)
        self.image_view = DebugImageView()
        image_layout.addWidget(self.image_view, 1)
        root.addWidget(image_box, 3)

        status = QHBoxLayout()
        self._detect_label = QLabel("板：—")
        self._detect_label.setObjectName("handeyeDetectIdle")
        self._count_label = QLabel(f"样本：0 / {DEFAULT_TARGET_SAMPLES}")
        self._health_label = QLabel("sidecar：未启动")
        self._health_label.setObjectName("statusValue")
        status.addWidget(self._detect_label)
        status.addWidget(self._count_label)
        status.addWidget(self._health_label, 1)
        root.addLayout(status)

        body = QHBoxLayout()
        body.addWidget(self._build_params(), 1)
        body.addWidget(self._build_actions(), 1)
        root.addLayout(body, 2)

        self._pattern.currentTextChanged.connect(self._sync_pattern_fields)
        self._solve_target.currentIndexChanged.connect(self._sync_solve_target)
        self._sync_pattern_fields()
        self._sync_solve_target()
        self._set_idle_status()

    def _build_params(self) -> QGroupBox:
        box = QGroupBox("标定参数")
        grid = QGridLayout(box)
        row = 0
        grid.addWidget(QLabel("求解目标"), row, 0)
        self._solve_target = QComboBox()
        self._solve_target.addItem("相机 (world→astra2_link)", SOLVE_TARGET_CAMERA)
        self._solve_target.addItem("右基座 (world→right_base_link)", SOLVE_TARGET_RIGHT_BASE)
        grid.addWidget(self._solve_target, row, 1)
        row += 1

        grid.addWidget(QLabel("标定板"), row, 0)
        self._pattern = QComboBox()
        self._pattern.addItem("棋盘格", "chessboard")
        self._pattern.addItem("ArUco", "aruco")
        grid.addWidget(self._pattern, row, 1)
        row += 1

        grid.addWidget(QLabel("内角点列 × 行"), row, 0)
        cr = QHBoxLayout()
        self._cols = QSpinBox()
        self._cols.setRange(3, 30)
        self._cols.setValue(11)
        self._rows = QSpinBox()
        self._rows.setRange(3, 30)
        self._rows.setValue(8)
        cr.addWidget(self._cols)
        cr.addWidget(QLabel("×"))
        cr.addWidget(self._rows)
        grid.addLayout(cr, row, 1)
        row += 1

        grid.addWidget(QLabel("格宽 (m)"), row, 0)
        self._square = QDoubleSpinBox()
        self._square.setDecimals(4)
        self._square.setRange(0.001, 0.2)
        self._square.setSingleStep(0.001)
        self._square.setValue(0.015)
        grid.addWidget(self._square, row, 1)
        row += 1

        grid.addWidget(QLabel("ArUco 字典 / id"), row, 0)
        ar = QHBoxLayout()
        self._dict = QComboBox()
        for name in ARUCO_DICT_NAMES:
            self._dict.addItem(name)
        self._marker_id = QSpinBox()
        self._marker_id.setRange(0, 250)
        ar.addWidget(self._dict, 1)
        ar.addWidget(self._marker_id)
        grid.addLayout(ar, row, 1)
        row += 1

        grid.addWidget(QLabel("码边长 (m)"), row, 0)
        self._marker_len = QDoubleSpinBox()
        self._marker_len.setDecimals(4)
        self._marker_len.setRange(0.005, 0.5)
        self._marker_len.setSingleStep(0.001)
        self._marker_len.setValue(DEFAULT_MARKER_LENGTH)
        grid.addWidget(self._marker_len, row, 1)
        row += 1

        grid.addWidget(QLabel("目标样本数"), row, 0)
        self._target = QSpinBox()
        self._target.setRange(MIN_SOLVE_SAMPLES, 80)
        self._target.setValue(DEFAULT_TARGET_SAMPLES)
        grid.addWidget(self._target, row, 1)
        row += 1

        self._start_camera = QCheckBox("若无彩色图则启动 Astra 2")
        self._start_camera.setChecked(True)
        grid.addWidget(self._start_camera, row, 0, 1, 2)
        row += 1

        self._arm_label = QLabel("法兰：left_revo2_flange（随通道）")
        self._arm_label.setWordWrap(True)
        self._arm_label.setStyleSheet("color: #9aa0a6; font-weight: 400;")
        grid.addWidget(self._arm_label, row, 0, 1, 2)
        return box

    def _build_actions(self) -> QGroupBox:
        box = QGroupBox("采样与保存")
        root = QVBoxLayout(box)

        yaml_row = QHBoxLayout()
        yaml_row.addWidget(QLabel("YAML"))
        self._yaml_edit = QLineEdit(str(default_yaml_path()))
        yaml_btn = QPushButton("浏览")
        yaml_btn.clicked.connect(self._browse_yaml)
        yaml_row.addWidget(self._yaml_edit, 1)
        yaml_row.addWidget(yaml_btn)
        root.addLayout(yaml_row)

        self._camera_yaml_row = QWidget()
        cam_row = QHBoxLayout(self._camera_yaml_row)
        cam_row.setContentsMargins(0, 0, 0, 0)
        cam_row.addWidget(QLabel("锁定相机"))
        self._camera_yaml_edit = QLineEdit(str(default_camera_yaml_path()))
        cam_btn = QPushButton("浏览")
        cam_btn.clicked.connect(self._browse_camera_yaml)
        cam_row.addWidget(self._camera_yaml_edit, 1)
        cam_row.addWidget(cam_btn)
        root.addWidget(self._camera_yaml_row)
        self._camera_yaml_row.setVisible(False)

        samp_row = QHBoxLayout()
        samp_row.addWidget(QLabel("样本"))
        self._samples_edit = QLineEdit(str(default_samples_path()))
        samp_btn = QPushButton("浏览")
        samp_btn.clicked.connect(self._browse_samples)
        samp_row.addWidget(self._samples_edit, 1)
        samp_row.addWidget(samp_btn)
        root.addLayout(samp_row)

        run_row = QHBoxLayout()
        self._start_btn = QPushButton("启动标定")
        self._stop_btn = QPushButton("停止")
        self._stop_btn.setEnabled(False)
        run_row.addWidget(self._start_btn)
        run_row.addWidget(self._stop_btn)
        root.addLayout(run_row)

        self._leader_btn = QPushButton("零力拖动")
        self._leader_btn.setObjectName("leaderButton")
        self._leader_btn.setCheckable(True)
        self._leader_btn.setToolTip("打开后电机进入零力，可手掰到位；再点一次退出")
        root.addWidget(self._leader_btn)

        act = QHBoxLayout()
        self._sample_btn = QPushButton("采样")
        self._drop_btn = QPushButton("丢掉上一组")
        self._solve_btn = QPushButton("求解")
        act.addWidget(self._sample_btn)
        act.addWidget(self._drop_btn)
        act.addWidget(self._solve_btn)
        root.addLayout(act)

        self._hint = QLabel("")
        self._hint.setWordWrap(True)
        self._hint.setStyleSheet("color: #9aa0a6; font-weight: 400;")
        root.addWidget(self._hint)

        self._start_btn.clicked.connect(self._on_start)
        self._stop_btn.clicked.connect(self.stop_requested)
        self._sample_btn.clicked.connect(self.sample_requested)
        self._drop_btn.clicked.connect(self.drop_requested)
        self._solve_btn.clicked.connect(self._on_solve)
        self._leader_btn.toggled.connect(self.leader_toggled)
        return box

    def set_channel(self, channel: str) -> None:
        self._channel = channel
        self._update_arm_label()

    def _update_arm_label(self) -> None:
        arm = arm_from_channel(self._channel)
        if self._solve_target.currentData() == SOLVE_TARGET_RIGHT_BASE:
            if arm == "right":
                self._arm_label.setText("拿板手臂：right　法兰：right_revo2_flange（右基座模式）")
            else:
                self._arm_label.setText(
                    f"当前通道是 {arm}。右基座求解请先连接 can_right，板贴 right_revo2_flange。"
                )
            return
        self._arm_label.setText(f"拿板手臂：{arm}　法兰：{ee_frame_for_arm(arm)}（随连接通道）")

    def set_running(self, running: bool) -> None:
        self._running = running
        self._start_btn.setEnabled(not running)
        set_widget_active(self._start_btn, running)
        self._start_btn.setText("标定运行中" if running else "启动标定")
        self._stop_btn.setEnabled(running)
        for widget in (
            self._solve_target,
            self._pattern,
            self._cols,
            self._rows,
            self._square,
            self._dict,
            self._marker_id,
            self._marker_len,
            self._start_camera,
            self._camera_yaml_edit,
        ):
            widget.setEnabled(not running)
        self._sync_pattern_fields()
        self._sync_solve_target_visibility()
        if not running:
            self._detected = False
            self._samples = 0
            self._set_idle_status()
            self.image_view.clear_image()
        self._update_action_buttons()

    def apply_handeye_status(self, msg: dict) -> None:
        board = str(msg.get("board", "NOT FOUND"))
        self._detected = board == "DETECTED"
        self._samples = int(msg.get("samples", 0) or 0)
        self._detect_label.setText(f"板：{board}")
        self._detect_label.setObjectName("handeyeDetectOk" if self._detected else "handeyeDetectBad")
        self._detect_label.style().unpolish(self._detect_label)
        self._detect_label.style().polish(self._detect_label)
        self._count_label.setText(f"样本：{self._samples} / {self._target.value()}")
        bits = []
        bits.append("图" if msg.get("have_image") else "无图")
        bits.append("法兰TF" if msg.get("have_tf") else "无法兰TF")
        bits.append("world TF" if msg.get("have_world_tf") else "无 world TF")
        if self._solve_target.currentData() == SOLVE_TARGET_RIGHT_BASE:
            bits.append("基座TF" if msg.get("have_base_tf") else "无基座TF")
        self._health_label.setText("sidecar：运行中 · " + " · ".join(bits))
        self._update_action_buttons()

    def apply_snapshot(self, snap: ArmSnapshot) -> None:
        connected = snap.connected and snap.enabled
        self._leader_btn.setEnabled(connected and not snap.playing)
        self._leader_btn.blockSignals(True)
        self._leader_btn.setChecked(snap.leader_mode)
        self._leader_btn.setText("零力拖动 · 开" if snap.leader_mode else "零力拖动")
        self._leader_btn.blockSignals(False)

    def current_config(self) -> HandeyeRunConfig:
        arm = arm_from_channel(self._channel)
        pattern = self._pattern.currentData()
        cfg = HandeyeRunConfig.for_channel(self._channel)
        cfg.pattern = str(pattern)
        cfg.cols = int(self._cols.value())
        cfg.rows = int(self._rows.value())
        cfg.square_size = float(self._square.value())
        cfg.aruco_dict = self._dict.currentText()
        cfg.marker_id = int(self._marker_id.value())
        cfg.marker_length = float(self._marker_len.value()) if pattern == "aruco" else None
        cfg.arm = arm
        cfg.ee_frame = ee_frame_for_arm(arm)
        cfg.target_samples = int(self._target.value())
        cfg.start_camera = self._start_camera.isChecked()
        cfg.yaml_path = self._yaml_edit.text().strip()
        cfg.samples_path = self._samples_edit.text().strip()
        cfg.solve_target = str(self._solve_target.currentData())
        cfg.camera_yaml_path = self._camera_yaml_edit.text().strip()
        if cfg.solve_target == SOLVE_TARGET_RIGHT_BASE and arm == "right":
            cfg.ee_frame = "right_revo2_flange"
        return cfg

    def _on_start(self) -> None:
        cfg = self.current_config()
        if not cfg.yaml_path or not cfg.samples_path:
            return
        try:
            Path(cfg.yaml_path).parent.mkdir(parents=True, exist_ok=True)
            Path(cfg.samples_path).parent.mkdir(parents=True, exist_ok=True)
        except OSError:
            return
        self.start_requested.emit(cfg)

    def _on_solve(self) -> None:
        self.solve_requested.emit(self._yaml_edit.text().strip(), self._samples_edit.text().strip())

    def _browse_yaml(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "求解 YAML 保存路径", self._yaml_edit.text(), "YAML (*.yaml *.yml)"
        )
        if path:
            self._yaml_edit.setText(path)

    def _browse_samples(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "样本 JSON 保存路径", self._samples_edit.text(), "JSON (*.json)"
        )
        if path:
            self._samples_edit.setText(path)

    def _browse_camera_yaml(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            "锁定相机 YAML",
            self._camera_yaml_edit.text(),
            "YAML (*.yaml *.yml)",
        )
        if path:
            self._camera_yaml_edit.setText(path)

    def _sync_solve_target(self) -> None:
        right = self._solve_target.currentData() == SOLVE_TARGET_RIGHT_BASE
        camera_default = str(default_yaml_path())
        right_default = str(default_right_base_yaml_path())
        current = self._yaml_edit.text().strip()
        if right and current in ("", camera_default):
            self._yaml_edit.setText(right_default)
        elif not right and current in ("", right_default):
            self._yaml_edit.setText(camera_default)
        self._sync_solve_target_visibility()
        self._update_arm_label()
        if right:
            self._hint.setText(
                "板贴右法兰。先完成左臂相机标定并粘贴 camera:。"
                "用点动或零力拉开平移/翻/转/高度；停稳且 DETECTED 后采样。"
                f"至少 {MIN_SOLVE_SAMPLES} 组后求解，把 right: 贴进 scene_tf.yaml，"
                "不要改 left: 或 camera:。无抱闸，零力时请托住。"
            )
        else:
            self._hint.setText(
                "用左侧点动或零力摆出平移/翻/转/高度变化。停稳且绿字 DETECTED 后再采样。"
                f"至少 {MIN_SOLVE_SAMPLES} 组才能求解。无抱闸，零力时请托住。"
            )

    def _sync_solve_target_visibility(self) -> None:
        right = self._solve_target.currentData() == SOLVE_TARGET_RIGHT_BASE
        self._camera_yaml_row.setVisible(right)
        self._camera_yaml_edit.setEnabled(right and not self._running)

    def _sync_pattern_fields(self) -> None:
        chess = self._pattern.currentData() == "chessboard"
        enabled = not self._running
        self._cols.setEnabled(enabled and chess)
        self._rows.setEnabled(enabled and chess)
        self._square.setEnabled(enabled and chess)
        self._dict.setEnabled(enabled and not chess)
        self._marker_id.setEnabled(enabled and not chess)
        self._marker_len.setEnabled(enabled and not chess)

    def _set_idle_status(self) -> None:
        self._detect_label.setText("板：—")
        self._detect_label.setObjectName("handeyeDetectIdle")
        self._detect_label.style().unpolish(self._detect_label)
        self._detect_label.style().polish(self._detect_label)
        self._count_label.setText(f"样本：0 / {self._target.value()}")
        self._health_label.setText("sidecar：未启动")

    def _update_action_buttons(self) -> None:
        self._sample_btn.setEnabled(self._running and self._detected)
        self._drop_btn.setEnabled(self._running and self._samples > 0)
        self._solve_btn.setEnabled(self._running and self._samples >= MIN_SOLVE_SAMPLES)
