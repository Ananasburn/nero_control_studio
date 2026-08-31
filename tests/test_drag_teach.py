from __future__ import annotations

import pytest

from nero_control_studio.arm.drag_teach import DragTeachCmd, send_drag_teach


def test_command_values_match_protocol() -> None:
    assert int(DragTeachCmd.START_RECORD) == 0x01
    assert int(DragTeachCmd.STOP_RECORD) == 0x02
    assert int(DragTeachCmd.EXECUTE) == 0x03
    assert int(DragTeachCmd.PAUSE) == 0x04
    assert int(DragTeachCmd.RESUME) == 0x05
    assert int(DragTeachCmd.TERMINATE) == 0x06
    assert int(DragTeachCmd.GOTO_START) == 0x07


def test_send_drag_teach_sets_byte() -> None:
    pytest.importorskip("pyAgxArm")

    class FakeRobot:
        def __init__(self) -> None:
            self.sent = []

        def _send_msg(self, msg) -> None:
            self.sent.append(msg)

    robot = FakeRobot()
    send_drag_teach(robot, DragTeachCmd.START_RECORD)
    assert len(robot.sent) == 1
    assert robot.sent[0].grag_teach_ctrl == 0x01
    assert robot.sent[0].emergency_stop == 0
    assert robot.sent[0].track_ctrl == 0


def test_send_drag_teach_rejects_out_of_range() -> None:
    class FakeRobot:
        def _send_msg(self, msg) -> None:
            raise AssertionError("should not send")

    with pytest.raises(ValueError):
        send_drag_teach(FakeRobot(), 99)
