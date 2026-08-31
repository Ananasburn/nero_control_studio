"""Firmware detection helpers, matching dual_nero_hardware.sdk_session."""

from __future__ import annotations

import time
import warnings
from typing import Any


def pick_firmware_from_sv(software_version: str) -> str:
    from pyAgxArm import NeroFW

    if software_version >= "1.20":
        return NeroFW.V120
    if software_version >= "1.12":
        return NeroFW.V112
    if software_version >= "1.11":
        return NeroFW.V111
    return NeroFW.DEFAULT


def leave_leader_if_needed(robot: Any, firmware: str) -> None:
    from pyAgxArm import NeroFW

    try:
        if firmware in (NeroFW.V112, NeroFW.V120):
            robot.set_follower_mode()
        else:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", UserWarning)
                robot.set_normal_mode()
        time.sleep(0.1)
    except Exception as exc:
        print(f"[WARN] could not leave leader mode: {exc}")


def create_arm(interface: str, channel: str, firmware: str) -> Any:
    from pyAgxArm import AgxArmFactory, ArmModel, create_agx_arm_config

    cfg = create_agx_arm_config(
        robot=ArmModel.NERO,
        firmeware_version=firmware,
        interface=interface,
        channel=channel,
    )
    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()
    return robot


def wait_firmware(robot: Any, timeout: float) -> dict:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            fw = robot.get_firmware(timeout=0.5, min_interval=0.5)
            if fw is not None and fw.get("software_version"):
                return fw
        except Exception:
            pass
        time.sleep(0.2)
    raise TimeoutError("no firmware within timeout")
