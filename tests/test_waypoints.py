from __future__ import annotations

import json
from pathlib import Path

import pytest

from nero_control_studio.arm.waypoints import WaypointStore
from nero_control_studio.constants import JOINT_COUNT, JOINT_LIMITS


def test_record_and_reorder(tmp_path: Path) -> None:
    store = WaypointStore()
    a = store.record([0.1] * JOINT_COUNT, name="A")
    store.record([0.2] * JOINT_COUNT, name="B")
    store.record([0.3] * JOINT_COUNT, name="C")
    assert a.name == "A"
    assert store.move(2, -1) == 1
    assert [wp.name for wp in store.waypoints] == ["A", "C", "B"]
    store.delete(0)
    assert [wp.name for wp in store.waypoints] == ["C", "B"]


def test_json_roundtrip(tmp_path: Path) -> None:
    store = WaypointStore()
    joints = [0.05 * i for i in range(JOINT_COUNT)]
    store.record(joints, name="home")
    path = tmp_path / "wp.json"
    store.save(path)

    loaded = WaypointStore()
    loaded.load(path)
    assert loaded.waypoints[0].name == "home"
    assert loaded.waypoints[0].joints == pytest.approx(joints)
    payload = json.loads(path.read_text())
    assert payload["format"] == "nero-control-studio-waypoints"


def test_record_clamps_limits() -> None:
    store = WaypointStore()
    over = [hi + 2.0 for _, hi in JOINT_LIMITS]
    wp = store.record(over)
    assert wp.joints[0] == JOINT_LIMITS[0][1]
