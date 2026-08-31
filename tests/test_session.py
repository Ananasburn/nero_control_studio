from __future__ import annotations

from types import SimpleNamespace

from nero_control_studio.arm.session import ArmSession, feedback_joints
from nero_control_studio.constants import JOINT_COUNT


class FakeRobot:
    def __init__(self) -> None:
        self.regular = [0.1 * (i + 1) for i in range(JOINT_COUNT)]
        self.leader = [0.2 * (i + 1) for i in range(JOINT_COUNT)]
        self.leader_available = True

    def get_joint_angles(self):
        return SimpleNamespace(msg=list(self.regular))

    def get_leader_joint_angles(self):
        if not self.leader_available:
            return None
        return SimpleNamespace(msg=list(self.leader))

    def get_arm_status(self):
        return None

    def get_joints_enable_status_list(self):
        return None


def test_feedback_joints_prefers_leader() -> None:
    robot = FakeRobot()
    assert feedback_joints(robot, prefer_leader=True) == robot.leader
    assert feedback_joints(robot, prefer_leader=False) == robot.regular


def test_feedback_joints_falls_back_when_leader_missing() -> None:
    robot = FakeRobot()
    robot.leader_available = False
    assert feedback_joints(robot, prefer_leader=True) == robot.regular


def test_poll_uses_leader_joints_in_leader_mode() -> None:
    session = ArmSession()
    robot = FakeRobot()
    session._robot = robot
    session._leader_mode = True
    session.poll()
    assert session.joints == robot.leader
    assert session._have_joints


def test_poll_uses_regular_joints_outside_leader_mode() -> None:
    session = ArmSession()
    robot = FakeRobot()
    session._robot = robot
    session.poll()
    assert session.joints == robot.regular
