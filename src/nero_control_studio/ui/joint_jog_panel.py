"""Per-joint mouse jog: step click or continuous hold."""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QButtonGroup,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QRadioButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from nero_control_studio.arm.status import ArmSnapshot
from nero_control_studio.arm.jog import joint_limit_kind
from nero_control_studio.constants import (
    DEFAULT_RATE_DEG_S,
    DEFAULT_STEP_DEG,
    DEG2RAD,
    JOINT_COUNT,
    JOINT_LIMITS,
    JOINT_NAMES,
    RAD2DEG,
)

_HINT_JOG = "Step：单击 ± 或点滑条。Continuous：按住 ± 或拖滑条，松开即停。选中项为黄色圆点。"
_HINT_LEADER = "零力拖动中：左侧继续显示当前角（不能点动）。超出 URDF 限位的轴为红色。"
_HINT_PAUSED = "点动已暂停（回放或固件示教中）。左侧仍显示当前关节角。"
_STYLE_OVER = "color: #ef5350; font-weight: 700;"
_STYLE_NEAR = "color: #ffcc80; font-weight: 600;"


class HoldButton(QPushButton):
    hold_started = Signal()
    hold_stopped = Signal()

    def __init__(self, text: str, parent: QWidget | None = None) -> None:
        super().__init__(text, parent)
        self.setAutoRepeat(False)
        self.setFixedWidth(36)
        self._held = False

    def mousePressEvent(self, event) -> None:  # type: ignore[override]
        if event.button() == Qt.MouseButton.LeftButton:
            self._held = True
            self.hold_started.emit()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # type: ignore[override]
        if self._held and event.button() == Qt.MouseButton.LeftButton:
            self._held = False
            self.hold_stopped.emit()
        super().mouseReleaseEvent(event)


class JointJogPanel(QGroupBox):
    step_requested = Signal(int, int)
    hold_started = Signal(int, int)
    hold_stopped = Signal()
    joint_target = Signal(int, float)
    params_changed = Signal(float, float)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__("关节点动", parent)
        self._dragging = [False] * JOINT_COUNT
        self._sliders: list[QSlider] = []
        self._value_labels: list[QLabel] = []
        self._name_labels: list[QLabel] = []
        self._minus: list[HoldButton] = []
        self._plus: list[HoldButton] = []
        self._input_enabled = True

        root = QVBoxLayout(self)

        mode_row = QHBoxLayout()
        self._step_radio = QRadioButton("Step")
        self._cont_radio = QRadioButton("Continuous")
        self._step_radio.setChecked(True)
        group = QButtonGroup(self)
        group.addButton(self._step_radio)
        group.addButton(self._cont_radio)
        mode_row.addWidget(self._step_radio)
        mode_row.addWidget(self._cont_radio)
        mode_row.addStretch(1)
        root.addLayout(mode_row)

        param_row = QHBoxLayout()
        param_row.addWidget(QLabel("步进 (°):"))
        self._step_spin = QDoubleSpinBox()
        self._step_spin.setRange(0.1, 15.0)
        self._step_spin.setSingleStep(0.1)
        self._step_spin.setValue(DEFAULT_STEP_DEG)
        param_row.addWidget(self._step_spin)
        param_row.addWidget(QLabel("连续 (°/s):"))
        self._rate_spin = QDoubleSpinBox()
        self._rate_spin.setRange(1.0, 45.0)
        self._rate_spin.setSingleStep(1.0)
        self._rate_spin.setValue(DEFAULT_RATE_DEG_S)
        param_row.addWidget(self._rate_spin)
        param_row.addStretch(1)
        root.addLayout(param_row)
        self._step_spin.valueChanged.connect(self._emit_params)
        self._rate_spin.valueChanged.connect(self._emit_params)

        grid = QGridLayout()
        grid.addWidget(QLabel(""), 0, 0)
        grid.addWidget(QLabel(""), 0, 1)
        grid.addWidget(QLabel("角度"), 0, 2)
        grid.addWidget(QLabel(""), 0, 3)
        grid.addWidget(QLabel("当前"), 0, 4)

        for i, name in enumerate(JOINT_NAMES):
            lo, hi = JOINT_LIMITS[i]
            minus = HoldButton("−")
            plus = HoldButton("+")
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(int(lo * RAD2DEG * 100), int(hi * RAD2DEG * 100))
            slider.setValue(0)
            value = QLabel("0.00°")
            value.setMinimumWidth(110)
            minus.clicked.connect(lambda _=False, idx=i: self._on_minus_click(idx))
            plus.clicked.connect(lambda _=False, idx=i: self._on_plus_click(idx))
            minus.hold_started.connect(lambda idx=i: self._on_hold(idx, -1))
            plus.hold_started.connect(lambda idx=i: self._on_hold(idx, 1))
            minus.hold_stopped.connect(self._on_hold_stop)
            plus.hold_stopped.connect(self._on_hold_stop)
            slider.sliderPressed.connect(lambda idx=i: self._set_dragging(idx, True))
            slider.sliderReleased.connect(lambda idx=i: self._on_slider_release(idx))
            slider.sliderMoved.connect(lambda val, idx=i: self._on_slider_moved(idx, val))
            name_label = QLabel(name)
            grid.addWidget(name_label, i + 1, 0)
            grid.addWidget(minus, i + 1, 1)
            grid.addWidget(slider, i + 1, 2)
            grid.addWidget(plus, i + 1, 3)
            grid.addWidget(value, i + 1, 4)
            self._name_labels.append(name_label)
            self._minus.append(minus)
            self._plus.append(plus)
            self._sliders.append(slider)
            self._value_labels.append(value)

        root.addLayout(grid)
        self._hint = QLabel(_HINT_JOG)
        self._hint.setWordWrap(True)
        self._hint.setStyleSheet("color: #9aa0a6; font-weight: 400;")
        root.addWidget(self._hint)

    def _emit_params(self) -> None:
        self.params_changed.emit(self._step_spin.value(), self._rate_spin.value())

    def _set_dragging(self, index: int, dragging: bool) -> None:
        self._dragging[index] = dragging

    def _on_minus_click(self, index: int) -> None:
        if self._input_enabled and not self._cont_radio.isChecked():
            self.step_requested.emit(index, -1)

    def _on_plus_click(self, index: int) -> None:
        if self._input_enabled and not self._cont_radio.isChecked():
            self.step_requested.emit(index, 1)

    def _on_hold(self, index: int, sign: int) -> None:
        if self._input_enabled and self._cont_radio.isChecked():
            self.hold_started.emit(index, sign)

    def _on_hold_stop(self) -> None:
        if self._cont_radio.isChecked():
            self.hold_stopped.emit()

    def _on_slider_moved(self, index: int, value: int) -> None:
        if self._input_enabled and self._cont_radio.isChecked() and self._dragging[index]:
            self.joint_target.emit(index, value / 100.0 * DEG2RAD)

    def _on_slider_release(self, index: int) -> None:
        self._dragging[index] = False
        if not self._input_enabled:
            return
        value = self._sliders[index].value()
        self.joint_target.emit(index, value / 100.0 * DEG2RAD)
        self.hold_stopped.emit()

    def apply_snapshot(self, snap: ArmSnapshot) -> None:
        can_jog = snap.can_jog
        self._input_enabled = can_jog
        self._step_radio.setEnabled(can_jog)
        self._cont_radio.setEnabled(can_jog)
        self._step_spin.setEnabled(can_jog)
        self._rate_spin.setEnabled(can_jog)
        for i in range(JOINT_COUNT):
            self._minus[i].setEnabled(can_jog)
            self._plus[i].setEnabled(can_jog)
            self._sliders[i].setAttribute(
                Qt.WidgetAttribute.WA_TransparentForMouseEvents, not can_jog
            )
        if snap.leader_mode:
            self._hint.setText(_HINT_LEADER)
        elif not can_jog:
            self._hint.setText(_HINT_PAUSED)
        else:
            self._hint.setText(_HINT_JOG)
        if not snap.have_joints:
            return
        for i, rad in enumerate(snap.joints):
            if self._dragging[i]:
                continue
            deg = rad * RAD2DEG
            self._sliders[i].blockSignals(True)
            self._sliders[i].setValue(int(round(deg * 100)))
            self._sliders[i].blockSignals(False)
            self._value_labels[i].setText(f"{deg:7.2f}°  ({rad:7.4f} rad)")
            kind = joint_limit_kind(i, rad)
            if kind == "over":
                style = _STYLE_OVER
            elif kind == "near":
                style = _STYLE_NEAR
            else:
                style = ""
            self._value_labels[i].setStyleSheet(style)
            self._name_labels[i].setStyleSheet(style if kind == "over" else "")
