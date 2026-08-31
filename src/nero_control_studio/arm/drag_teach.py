"""Firmware drag-teach via CAN 0x150 Byte 2 (grag_teach_ctrl)."""

from __future__ import annotations

from enum import IntEnum
from typing import Any


class DragTeachCmd(IntEnum):
    CLOSE = 0x00
    START_RECORD = 0x01
    STOP_RECORD = 0x02
    EXECUTE = 0x03
    PAUSE = 0x04
    RESUME = 0x05
    TERMINATE = 0x06
    GOTO_START = 0x07


def send_drag_teach(robot: Any, command: DragTeachCmd | int) -> None:
    """Send 0x150 with only the drag-teach byte set.

    pyAgxArm's Nero driver exposes 0x150 for e-stop/reset only; this fills the
    protocol gap for firmware trajectory record/playback.
    """
    from pyAgxArm.protocols.can_protocol.msgs.nero.default import ArmMsgMotionCtrl

    value = int(command)
    if value not in range(0x00, 0x08):
        raise ValueError(f"grag_teach_ctrl out of range: {value}")
    robot._send_msg(ArmMsgMotionCtrl(grag_teach_ctrl=value))
