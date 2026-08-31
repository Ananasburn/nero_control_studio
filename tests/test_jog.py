from __future__ import annotations

import math

import pytest

from nero_control_studio.arm.jog import (
    apply_continuous,
    apply_step,
    clamp_joint,
    clamp_joints,
    joint_limit_kind,
    joints_outside_limits,
    set_joint,
)
from nero_control_studio.constants import JOINT_COUNT, JOINT_LIMITS, MAX_CONTINUOUS_DELTA_RAD


def test_clamp_joint_at_limits() -> None:
    lo, hi = JOINT_LIMITS[0]
    assert clamp_joint(0, lo - 1.0) == lo
    assert clamp_joint(0, hi + 1.0) == hi
    assert clamp_joint(0, 0.0) == 0.0


def test_clamp_joints_rejects_wrong_length() -> None:
    with pytest.raises(ValueError):
        clamp_joints([0.0] * 6)


def test_apply_step_moves_one_joint() -> None:
    current = [0.0] * JOINT_COUNT
    target = apply_step(current, 3, 1, math.radians(1.0))
    assert target[3] == pytest.approx(math.radians(1.0))
    assert target[:3] == [0.0, 0.0, 0.0]


def test_apply_step_clamps() -> None:
    current = [0.0] * JOINT_COUNT
    lo, hi = JOINT_LIMITS[5]
    current[5] = hi
    target = apply_step(current, 5, 1, 0.5)
    assert target[5] == hi


def test_apply_continuous_caps_delta() -> None:
    current = [0.0] * JOINT_COUNT
    target = apply_continuous(current, 0, 1, rate_rad_s=10.0, dt=1.0)
    assert target[0] == pytest.approx(MAX_CONTINUOUS_DELTA_RAD)


def test_set_joint() -> None:
    current = [0.1] * JOINT_COUNT
    target = set_joint(current, 1, 0.5)
    assert target[1] == pytest.approx(0.5)
    assert target[0] == pytest.approx(0.1)


def test_joints_outside_limits() -> None:
    ok = [0.0] * JOINT_COUNT
    assert joints_outside_limits(ok) == []
    over = list(ok)
    over[5] = JOINT_LIMITS[5][1] + 0.2
    over[3] = JOINT_LIMITS[3][0] - 0.2
    assert joints_outside_limits(over) == [4, 6]


def test_joint_limit_kind_near_and_over() -> None:
    lo, hi = JOINT_LIMITS[5]
    assert joint_limit_kind(5, 0.0) in ("ok", "near")
    assert joint_limit_kind(5, hi + 0.2) == "over"
    assert joint_limit_kind(5, hi) == "near"
