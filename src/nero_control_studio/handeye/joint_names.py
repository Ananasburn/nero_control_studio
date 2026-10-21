"""Joint names matching dual_nero_description (left/right Nero + Revo2)."""

from __future__ import annotations

from collections.abc import Sequence

from nero_control_studio.constants import JOINT_COUNT

ARM_JOINTS = {
    "left": [f"left_joint{i}" for i in range(1, 8)],
    "right": [f"right_joint{i}" for i in range(1, 8)],
}

HAND_JOINTS: list[str] = []
for _side in ("left", "right"):
    for _finger in (
        "thumb_metacarpal",
        "thumb_proximal",
        "index_proximal",
        "middle_proximal",
        "ring_proximal",
        "pinky_proximal",
    ):
        HAND_JOINTS.append(f"{_side}_revo2_{_finger}_joint")

EE_FRAMES = {
    "left": "left_revo2_flange",
    "right": "right_revo2_flange",
}

CHANNEL_TO_ARM = {
    "can_left": "left",
    "can0": "left",
    "can_right": "right",
    "can1": "right",
}


def arm_from_channel(channel: str) -> str:
    key = (channel or "").strip().lower()
    if "right" in key:
        return "right"
    if key in CHANNEL_TO_ARM:
        return CHANNEL_TO_ARM[key]
    return "left"


def ee_frame_for_arm(arm: str) -> str:
    return EE_FRAMES.get(arm, EE_FRAMES["left"])


def joint_state_arrays(arm: str, q: Sequence[float]) -> tuple[list[str], list[float]]:
    """Names and positions for /joint_states. Other arm and fingers are 0."""
    names = list(ARM_JOINTS["left"]) + list(ARM_JOINTS["right"]) + list(HAND_JOINTS)
    positions = [0.0] * len(names)
    side = "right" if arm == "right" else "left"
    joints = ARM_JOINTS[side]
    values = [float(v) for v in q[:JOINT_COUNT]]
    if len(values) < JOINT_COUNT:
        values.extend([0.0] * (JOINT_COUNT - len(values)))
    offset = 0 if side == "left" else JOINT_COUNT
    for i, value in enumerate(values):
        positions[offset + i] = value
    return names, positions
