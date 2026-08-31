"""Software waypoint store: record, reorder, JSON save/load."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from nero_control_studio.arm.jog import clamp_joints
from nero_control_studio.constants import JOINT_COUNT


@dataclass
class Waypoint:
    name: str
    joints: list[float]

    def to_dict(self) -> dict:
        return {"name": self.name, "joints": [float(v) for v in self.joints]}

    @classmethod
    def from_dict(cls, data: dict, index: int) -> Waypoint:
        joints = clamp_joints([float(v) for v in data["joints"]])
        name = str(data.get("name") or f"WP{index + 1}")
        return cls(name=name, joints=joints)


@dataclass
class WaypointStore:
    waypoints: list[Waypoint] = field(default_factory=list)

    def record(self, joints: list[float], name: str | None = None) -> Waypoint:
        if len(joints) != JOINT_COUNT:
            raise ValueError(f"need {JOINT_COUNT} joints, got {len(joints)}")
        index = len(self.waypoints)
        wp = Waypoint(
            name=name or f"WP{index + 1}",
            joints=clamp_joints(joints),
        )
        self.waypoints.append(wp)
        return wp

    def delete(self, index: int) -> None:
        self.waypoints.pop(index)

    def move(self, index: int, delta: int) -> int:
        new_index = index + delta
        if not (0 <= index < len(self.waypoints)):
            raise IndexError(index)
        if not (0 <= new_index < len(self.waypoints)):
            return index
        item = self.waypoints.pop(index)
        self.waypoints.insert(new_index, item)
        return new_index

    def clear(self) -> None:
        self.waypoints.clear()

    def to_payload(self) -> list[dict]:
        return [wp.to_dict() for wp in self.waypoints]

    def save(self, path: str | Path) -> None:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "format": "nero-control-studio-waypoints",
            "version": 1,
            "waypoints": self.to_payload(),
        }
        target.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    def load(self, path: str | Path) -> None:
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        items = payload.get("waypoints", payload if isinstance(payload, list) else [])
        self.waypoints = [
            Waypoint.from_dict(item, i) for i, item in enumerate(items)
        ]
