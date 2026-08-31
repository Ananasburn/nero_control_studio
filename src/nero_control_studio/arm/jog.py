"""Joint limit clamp and jog target math (no hardware)."""

from __future__ import annotations

from nero_control_studio.constants import (
    JOINT_COUNT,
    JOINT_LIMITS,
    LIMIT_EPS,
    MAX_CONTINUOUS_DELTA_RAD,
    NEAR_LIMIT_DEG,
    DEG2RAD,
)


def clamp_joint(index: int, value: float) -> float:
    if index < 0 or index >= JOINT_COUNT:
        raise IndexError(f"joint index {index} out of range 0..{JOINT_COUNT - 1}")
    lo, hi = JOINT_LIMITS[index]
    return min(hi, max(lo, float(value)))


def clamp_joints(joints: list[float]) -> list[float]:
    if len(joints) != JOINT_COUNT:
        raise ValueError(f"need {JOINT_COUNT} joints, got {len(joints)}")
    return [clamp_joint(i, value) for i, value in enumerate(joints)]


def joints_outside_limits(joints: list[float]) -> list[int]:
    """Return 1-based joint indices that are outside URDF limits."""
    over: list[int] = []
    for i, value in enumerate(joints[:JOINT_COUNT]):
        lo, hi = JOINT_LIMITS[i]
        if value < lo - LIMIT_EPS or value > hi + LIMIT_EPS:
            over.append(i + 1)
    return over


def joint_limit_kind(index: int, value: float) -> str:
    """'over', 'near', or 'ok' relative to URDF limits."""
    lo, hi = JOINT_LIMITS[index]
    if value < lo - LIMIT_EPS or value > hi + LIMIT_EPS:
        return "over"
    margin = NEAR_LIMIT_DEG * DEG2RAD
    if value <= lo + margin or value >= hi - margin:
        return "near"
    return "ok"


def apply_step(
    current: list[float], joint_index: int, sign: int, step_rad: float
) -> list[float]:
    if sign not in (-1, 1):
        raise ValueError("sign must be -1 or +1")
    if step_rad <= 0.0:
        raise ValueError("step_rad must be > 0")
    target = list(current)
    target[joint_index] = clamp_joint(
        joint_index, target[joint_index] + sign * step_rad
    )
    return target


def apply_continuous(
    current: list[float],
    joint_index: int,
    sign: int,
    rate_rad_s: float,
    dt: float,
    max_delta: float = MAX_CONTINUOUS_DELTA_RAD,
) -> list[float]:
    if sign not in (-1, 1):
        raise ValueError("sign must be -1 or +1")
    if rate_rad_s <= 0.0:
        raise ValueError("rate_rad_s must be > 0")
    if dt <= 0.0:
        return list(current)
    delta = sign * min(rate_rad_s * dt, max_delta)
    target = list(current)
    target[joint_index] = clamp_joint(joint_index, target[joint_index] + delta)
    return target


def set_joint(current: list[float], joint_index: int, value_rad: float) -> list[float]:
    target = list(current)
    target[joint_index] = clamp_joint(joint_index, value_rad)
    return target
