"""Line-delimited JSON protocol between the GUI and the ROS sidecar."""

from __future__ import annotations

import json
from typing import Any


def encode_cmd(cmd: str, **fields: Any) -> str:
    payload = {"cmd": cmd}
    payload.update(fields)
    return json.dumps(payload, ensure_ascii=False) + "\n"


def parse_message(line: str) -> dict[str, Any] | None:
    text = line.strip()
    if not text:
        return None
    data = json.loads(text)
    if not isinstance(data, dict):
        raise ValueError("protocol message must be a JSON object")
    return data
