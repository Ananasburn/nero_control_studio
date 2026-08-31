"""Shared constants for Nero Control Studio."""

from __future__ import annotations

import math

JOINT_COUNT = 7
JOINT_NAMES = tuple(f"J{i}" for i in range(1, JOINT_COUNT + 1))

# Match dual_nero_hardware / URDF limits (radians).
JOINT_LIMITS: tuple[tuple[float, float], ...] = (
    (-2.70526, 2.70526),
    (-1.74, 1.74),
    (-2.75, 2.75),
    (-1.01, 2.14),
    (-2.75, 2.75),
    (-0.73, 0.95),
    (-1.5707963, 1.5707963),
)
LIMIT_EPS = 1e-4
NEAR_LIMIT_DEG = 5.0

DEFAULT_STEP_DEG = 1.0
DEFAULT_RATE_DEG_S = 10.0
MAX_CONTINUOUS_DELTA_RAD = 0.05
POLL_HZ = 20.0
POLL_PERIOD_S = 1.0 / POLL_HZ
PLAYBACK_SETTLE_S = 0.25
PLAYBACK_TIMEOUT_S = 20.0
ENABLE_TIMEOUT_S = 3.0
FIRMWARE_TIMEOUT_S = 8.0
CONNECT_SETTLE_S = 0.5

DEFAULT_CHANNEL = "can_left"
DEFAULT_CHANNELS = ("can_left", "can_right", "can0", "can1")
DEFAULT_INTERFACE = "socketcan"
DEFAULT_SPEED_PERCENT = 30

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

CTRL_MODE_LABELS = {
    0x00: "待机",
    0x01: "CAN 控制",
    0x02: "示教模式",
    0x03: "以太网",
    0x04: "Wi-Fi",
    0x05: "遥控器",
    0x06: "联动示教输入",
    0x07: "离线轨迹",
    0x08: "TCP 控制",
}

ARM_STATUS_LABELS = {
    0x00: "正常",
    0x01: "急停",
    0x02: "无解",
    0x03: "奇异点",
    0x04: "目标超限",
    0x05: "关节通信异常",
    0x06: "关节未就绪",
    0x07: "碰撞",
    0x08: "拖动示教超速",
    0x09: "关节状态异常",
    0x0A: "其它异常",
    0x0B: "示教记录",
    0x0C: "示教执行",
    0x0D: "示教暂停",
    0x0E: "主控过温",
    0x0F: "释放电阻过温",
}

TEACH_STATUS_LABELS = {
    0x00: "关闭",
    0x01: "记录中",
    0x02: "结束记录",
    0x03: "复现中",
    0x04: "暂停",
    0x05: "继续",
    0x06: "已终止",
    0x07: "去起点",
}

MODE_FEEDBACK_LABELS = {
    0x00: "MOVE P",
    0x01: "MOVE J",
    0x02: "MOVE L",
    0x03: "MOVE C",
    0x04: "MOVE M",
    0x05: "MOVE CPV",
    0x06: "MOVE M",
}

MOTION_REACHED = 0x00
TEACH_RECORDING = 0x01
TEACH_EXECUTING = 0x03
TEACH_MOVING_TO_START = 0x07
