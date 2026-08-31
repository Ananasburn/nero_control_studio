from __future__ import annotations

from nero_control_studio.arm.firmware import pick_firmware_from_sv
from nero_control_studio.arm.status import ArmSnapshot, label_for
from nero_control_studio.constants import ARM_STATUS_LABELS, TEACH_RECORDING


def test_pick_firmware() -> None:
    import pytest

    pytest.importorskip("pyAgxArm")
    from pyAgxArm import NeroFW

    assert pick_firmware_from_sv("1.20") == NeroFW.V120
    assert pick_firmware_from_sv("1.12") == NeroFW.V112
    assert pick_firmware_from_sv("1.11") == NeroFW.V111
    assert pick_firmware_from_sv("1.10") == NeroFW.DEFAULT


def test_snapshot_can_jog_flags() -> None:
    snap = ArmSnapshot(
        connected=True,
        enabled=True,
        have_joints=True,
        teach_status=TEACH_RECORDING,
    )
    assert snap.firmware_recording
    assert not snap.can_jog
    assert label_for(ARM_STATUS_LABELS, 0) == "正常"


def test_activity_text_leader_and_idle() -> None:
    idle = ArmSnapshot()
    assert idle.activity_kind == "idle"
    assert idle.activity_text == "未连接"

    leader = ArmSnapshot(connected=True, enabled=True, leader_mode=True)
    assert leader.activity_kind == "leader"
    assert "零力拖动" in leader.activity_text

    over_joints = [0.0] * 7
    over_joints[5] = 2.0
    over = ArmSnapshot(
        connected=True,
        enabled=True,
        leader_mode=True,
        have_joints=True,
        joints=over_joints,
    )
    assert over.urdf_over_limit == [6]
    assert over.activity_kind == "warn"
    assert "URDF超限 J6" in over.activity_text

    ready = ArmSnapshot(connected=True, enabled=True)
    assert ready.activity_kind == "ready"
    assert ready.activity_text.startswith("待机")
