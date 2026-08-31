"""Arm status snapshot used by the worker and UI."""

from __future__ import annotations

from dataclasses import dataclass, field

from nero_control_studio.arm.jog import joints_outside_limits
from nero_control_studio.constants import (
    ARM_STATUS_LABELS,
    CTRL_MODE_LABELS,
    JOINT_COUNT,
    MODE_FEEDBACK_LABELS,
    TEACH_EXECUTING,
    TEACH_MOVING_TO_START,
    TEACH_RECORDING,
    TEACH_STATUS_LABELS,
)


def _as_int(value: object, default: int = 0) -> int:
    if value is None:
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        raw = getattr(value, "value", None)
        if raw is None:
            return default
        return int(raw)


def label_for(mapping: dict[int, str], value: int) -> str:
    return mapping.get(value, f"0x{value:02X}")


@dataclass
class ArmSnapshot:
    connected: bool = False
    enabled: bool = False
    connecting: bool = False
    channel: str = ""
    firmware: str = ""
    software_version: str = ""
    joints: list[float] = field(default_factory=lambda: [0.0] * JOINT_COUNT)
    target: list[float] = field(default_factory=lambda: [0.0] * JOINT_COUNT)
    have_joints: bool = False
    ctrl_mode: int = 0
    arm_status: int = 0
    teach_status: int = 0
    motion_status: int = 0
    mode_feedback: int = 0
    err_code: int = 0
    err_text: str = ""
    leader_mode: bool = False
    jogging: bool = False
    playing: bool = False
    play_index: int = -1
    last_error: str = ""

    @property
    def urdf_over_limit(self) -> list[int]:
        if not self.have_joints:
            return []
        return joints_outside_limits(self.joints)

    @property
    def urdf_over_limit_text(self) -> str:
        axes = self.urdf_over_limit
        if not axes:
            return ""
        return "URDF超限 " + "、".join(f"J{i}" for i in axes)

    @property
    def ctrl_mode_label(self) -> str:
        return label_for(CTRL_MODE_LABELS, self.ctrl_mode)

    @property
    def arm_status_label(self) -> str:
        return label_for(ARM_STATUS_LABELS, self.arm_status)

    @property
    def teach_status_label(self) -> str:
        return label_for(TEACH_STATUS_LABELS, self.teach_status)

    @property
    def mode_feedback_label(self) -> str:
        return label_for(MODE_FEEDBACK_LABELS, self.mode_feedback)

    @property
    def firmware_recording(self) -> bool:
        return self.teach_status == TEACH_RECORDING

    @property
    def firmware_busy(self) -> bool:
        return self.teach_status in (
            TEACH_RECORDING,
            TEACH_EXECUTING,
            TEACH_MOVING_TO_START,
        )

    @property
    def can_jog(self) -> bool:
        return (
            self.connected
            and self.enabled
            and not self.leader_mode
            and not self.playing
            and not self.firmware_busy
        )

    @property
    def can_teach(self) -> bool:
        return self.connected and self.enabled and not self.playing

    @property
    def activity_kind(self) -> str:
        if self.connecting:
            return "warn"
        if not self.connected:
            return "idle"
        if self.arm_status == 0x01:
            return "error"
        if not self.enabled:
            return "warn"
        if self.urdf_over_limit:
            return "warn"
        if self.leader_mode:
            return "leader"
        if self.playing:
            return "play"
        if self.firmware_recording or self.teach_status == TEACH_EXECUTING:
            return "teach"
        if self.teach_status == TEACH_MOVING_TO_START:
            return "teach"
        if self.jogging:
            return "jog"
        return "ready"

    @property
    def activity_text(self) -> str:
        if self.connecting:
            return "正在连接…"
        if not self.connected:
            return "未连接"
        if self.arm_status == 0x01:
            return "急停"
        if not self.enabled:
            return "已连接 · 电机失能"
        if self.leader_mode:
            base = "零力拖动中 · 可手掰手臂，再点一次「零力拖动」退出"
            extra = self.urdf_over_limit_text
            return f"{base} · {extra}" if extra else base
        if self.playing:
            n = self.play_index + 1 if self.play_index >= 0 else 0
            return f"路点回放中 · 第 {n} 点"
        if self.firmware_recording:
            return "固件示教 · 记录中（手掰手臂）"
        if self.teach_status == TEACH_EXECUTING:
            return "固件示教 · 复现中"
        if self.teach_status == TEACH_MOVING_TO_START:
            return "固件示教 · 正在去起点"
        if self.teach_status == 0x04:
            return "固件示教 · 已暂停"
        if self.jogging:
            return "关节点动中"
        extra = self.urdf_over_limit_text
        base = "待机 · 可点动或示教"
        return f"{base} · {extra}" if extra else base


def format_err_text(msg: object) -> tuple[int, str]:
    err_code = _as_int(getattr(msg, "err_code", 0), 0)
    bits: list[str] = []
    err_status = getattr(msg, "err_status", None)
    if err_status is not None:
        for i in range(1, 8):
            if getattr(err_status, f"joint_{i}_angle_limit", False):
                bits.append(f"J{i}超限")
            if getattr(err_status, f"communication_status_joint_{i}", False):
                bits.append(f"J{i}通信")
    if err_code and not bits:
        bits.append(f"err=0x{err_code:04X}")
    return err_code, ", ".join(bits)
