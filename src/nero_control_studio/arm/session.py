"""Thread-safe pyAgxArm session: connect, enable, motion, status."""

from __future__ import annotations

import threading
import time
from typing import Any, Optional

from nero_control_studio.arm.drag_teach import DragTeachCmd, send_drag_teach
from nero_control_studio.arm.firmware import (
    create_arm,
    leave_leader_if_needed,
    pick_firmware_from_sv,
    wait_firmware,
)
from nero_control_studio.arm.jog import clamp_joints
from nero_control_studio.arm.status import ArmSnapshot, _as_int, format_err_text
from nero_control_studio.constants import (
    CONNECT_SETTLE_S,
    ENABLE_TIMEOUT_S,
    FIRMWARE_TIMEOUT_S,
    JOINT_COUNT,
    TEACH_RECORDING,
)


def _busy_message(exc: BaseException) -> str:
    text = str(exc)
    lowered = text.lower()
    if "busy" in lowered or "address already in use" in lowered or "16" in text:
        return (
            f"打开 CAN 失败（可能已被占用）：{text}。"
            "请先关掉 real.launch.py / publish_nero_joint_states.py 等占用该接口的进程。"
        )
    if "no such device" in lowered or "network is down" in lowered:
        return f"CAN 接口不可用：{text}。请确认已 bind 并 ip link set up、bitrate 1 Mbps。"
    return text


def _msg_to_joints(mja: Any) -> list[float] | None:
    if mja is None:
        return None
    msg = getattr(mja, "msg", None)
    if msg is None:
        return None
    try:
        joints = [float(v) for v in msg]
    except (TypeError, ValueError):
        return None
    if len(joints) != JOINT_COUNT:
        return None
    return joints


def feedback_joints(robot: Any, *, prefer_leader: bool) -> list[float] | None:
    """Read 7 joint angles. Leader mode uses a different CAN feedback stream."""
    if prefer_leader:
        getter = getattr(robot, "get_leader_joint_angles", None)
        if callable(getter):
            joints = _msg_to_joints(getter())
            if joints is not None:
                return joints
    getter = getattr(robot, "get_joint_angles", None)
    if not callable(getter):
        return None
    return _msg_to_joints(getter())


class ArmSession:
    """Owns one Nero driver instance. Call from a single worker thread."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._robot: Any = None
        self._firmware = ""
        self._software_version = ""
        self._channel = ""
        self._interface = "socketcan"
        self._enabled = False
        self._leader_mode = False
        self._joints = [0.0] * JOINT_COUNT
        self._target = [0.0] * JOINT_COUNT
        self._have_joints = False
        self._last_error = ""
        self._ctrl_mode = 0
        self._arm_status = 0
        self._teach_status = 0
        self._motion_status = 0
        self._mode_feedback = 0
        self._err_code = 0
        self._err_text = ""

    @property
    def connected(self) -> bool:
        return self._robot is not None

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def leader_mode(self) -> bool:
        return self._leader_mode

    @property
    def teach_status(self) -> int:
        return self._teach_status

    @property
    def channel(self) -> str:
        return self._channel

    @property
    def joints(self) -> list[float]:
        return list(self._joints)

    @property
    def target(self) -> list[float]:
        return list(self._target)

    def connect(
        self,
        channel: str,
        interface: str = "socketcan",
        firmware_timeout: float = FIRMWARE_TIMEOUT_S,
    ) -> str:
        with self._lock:
            if self._robot is not None:
                self._disconnect_unlocked(disable=False)
            from pyAgxArm import NeroFW

            try:
                fw_use = NeroFW.DEFAULT
                robot = create_arm(interface, channel, fw_use)
                time.sleep(CONNECT_SETTLE_S)
                fw = wait_firmware(robot, timeout=max(firmware_timeout, 5.0))
                sv = str(fw.get("software_version", "?"))
                detected = pick_firmware_from_sv(sv)
                if fw_use != detected:
                    robot.disconnect()
                    robot = create_arm(interface, channel, detected)
                    time.sleep(CONNECT_SETTLE_S)
                    fw_use = detected
                leave_leader_if_needed(robot, fw_use)
            except Exception as exc:
                message = _busy_message(exc)
                self._last_error = message
                raise RuntimeError(message) from exc

            self._robot = robot
            self._firmware = fw_use
            self._software_version = sv
            self._channel = channel
            self._interface = interface
            self._enabled = False
            self._leader_mode = False
            self._last_error = ""
            self.poll()
            if self._have_joints:
                self._target = list(self._joints)
            return sv

    def disconnect(self, disable: bool = True) -> None:
        with self._lock:
            self._disconnect_unlocked(disable=disable)

    def _disconnect_unlocked(self, disable: bool) -> None:
        robot = self._robot
        if robot is None:
            return
        try:
            if self._leader_mode:
                leave_leader_if_needed(robot, self._firmware)
                self._leader_mode = False
            if disable:
                try:
                    self._apply_enable_unlocked(False, timeout=1.5)
                except Exception:
                    pass
            robot.disconnect()
        except Exception as exc:
            self._last_error = str(exc)
        finally:
            self._robot = None
            self._enabled = False
            self._have_joints = False

    def enable(self, timeout: float = ENABLE_TIMEOUT_S) -> bool:
        with self._lock:
            ok = self._apply_enable_unlocked(True, timeout=timeout)
            if ok:
                self.poll()
                if self._have_joints:
                    self._target = list(self._joints)
            return ok

    def disable(self, timeout: float = ENABLE_TIMEOUT_S) -> bool:
        with self._lock:
            if self._leader_mode and self._robot is not None:
                leave_leader_if_needed(self._robot, self._firmware)
                self._leader_mode = False
            return self._apply_enable_unlocked(False, timeout=timeout)

    def _apply_enable_unlocked(self, enable: bool, timeout: float) -> bool:
        robot = self._require_robot()
        deadline = time.monotonic() + timeout
        last_exc: Optional[BaseException] = None
        while time.monotonic() < deadline:
            try:
                ok = bool(robot.enable() if enable else robot.disable())
                if ok:
                    self._enabled = enable
                    return True
            except Exception as exc:
                last_exc = exc
            time.sleep(0.05)
        self._last_error = (
            f"{'使能' if enable else '失能'}超时"
            + (f": {last_exc}" if last_exc else "")
        )
        return False

    def set_speed_percent(self, percent: int) -> None:
        with self._lock:
            percent = max(1, min(100, int(percent)))
            self._require_robot().set_speed_percent(percent)

    def emergency_stop(self) -> None:
        with self._lock:
            robot = self._require_robot()
            if self._leader_mode:
                leave_leader_if_needed(robot, self._firmware)
                self._leader_mode = False
            robot.electronic_emergency_stop()
            self._target = list(self._joints)

    def reset(self) -> None:
        with self._lock:
            self._require_robot().reset()

    def move_j(self, joints: list[float]) -> None:
        with self._lock:
            robot = self._require_robot()
            if not self._enabled:
                raise RuntimeError("电机未使能")
            if self._leader_mode:
                raise RuntimeError("零力拖动中，不能点动")
            target = clamp_joints(joints)
            robot.move_j(target)
            self._target = target

    def seed_target_from_feedback(self) -> list[float]:
        with self._lock:
            self.poll()
            if self._have_joints:
                self._target = list(self._joints)
            return list(self._target)

    def set_leader_mode(self, on: bool) -> None:
        with self._lock:
            robot = self._require_robot()
            if not self._enabled:
                raise RuntimeError("电机未使能")
            if on:
                robot.set_leader_mode()
                self._leader_mode = True
            else:
                leave_leader_if_needed(robot, self._firmware)
                self._leader_mode = False
                self.poll()
                if self._have_joints:
                    self._target = list(self._joints)

    def send_drag_teach(self, command: DragTeachCmd | int) -> None:
        with self._lock:
            send_drag_teach(self._require_robot(), command)

    def poll(self) -> None:
        robot = self._robot
        if robot is None:
            return
        try:
            joints = feedback_joints(
                robot,
                prefer_leader=self._leader_mode
                or self._teach_status == TEACH_RECORDING,
            )
            if joints is not None:
                self._joints = joints
                self._have_joints = True
                if not self._enabled:
                    self._target = list(joints)
        except Exception as exc:
            self._last_error = str(exc)

        try:
            status = robot.get_arm_status()
            if status is not None and status.msg is not None:
                msg = status.msg
                self._ctrl_mode = _as_int(msg.ctrl_mode)
                self._arm_status = _as_int(msg.arm_status)
                self._teach_status = _as_int(msg.teach_status)
                self._motion_status = _as_int(msg.motion_status)
                self._mode_feedback = _as_int(msg.mode_feedback)
                self._err_code, self._err_text = format_err_text(msg)
        except Exception as exc:
            self._last_error = str(exc)

        try:
            enabled_list = robot.get_joints_enable_status_list()
            if isinstance(enabled_list, (list, tuple)) and enabled_list:
                flags = [bool(v) for v in enabled_list[:JOINT_COUNT]]
                if all(flags):
                    self._enabled = True
                elif not any(flags):
                    self._enabled = False
        except Exception:
            pass

    def snapshot(
        self,
        *,
        connecting: bool = False,
        jogging: bool = False,
        playing: bool = False,
        play_index: int = -1,
    ) -> ArmSnapshot:
        with self._lock:
            if self._robot is not None:
                self.poll()
            return ArmSnapshot(
                connected=self._robot is not None,
                enabled=self._enabled,
                connecting=connecting,
                channel=self._channel,
                firmware=self._firmware,
                software_version=self._software_version,
                joints=list(self._joints),
                target=list(self._target),
                have_joints=self._have_joints,
                ctrl_mode=self._ctrl_mode,
                arm_status=self._arm_status,
                teach_status=self._teach_status,
                motion_status=self._motion_status,
                mode_feedback=self._mode_feedback,
                err_code=self._err_code,
                err_text=self._err_text,
                leader_mode=self._leader_mode,
                jogging=jogging,
                playing=playing,
                play_index=play_index,
                last_error=self._last_error,
            )

    def _require_robot(self) -> Any:
        if self._robot is None:
            raise RuntimeError("未连接机械臂")
        return self._robot
