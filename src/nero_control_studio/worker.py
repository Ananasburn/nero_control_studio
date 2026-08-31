"""QObject worker that owns ArmSession and runs all CAN I/O off the GUI thread."""

from __future__ import annotations

import time
from typing import Optional

from PySide6.QtCore import QObject, QTimer, Signal, Slot

from nero_control_studio.arm.drag_teach import DragTeachCmd
from nero_control_studio.arm.jog import apply_continuous, apply_step, set_joint, joints_outside_limits
from nero_control_studio.arm.session import ArmSession
from nero_control_studio.arm.waypoints import WaypointStore
from nero_control_studio.constants import (
    DEFAULT_RATE_DEG_S,
    DEFAULT_STEP_DEG,
    DEG2RAD,
    MOTION_REACHED,
    PLAYBACK_SETTLE_S,
    PLAYBACK_TIMEOUT_S,
    POLL_PERIOD_S,
    TEACH_EXECUTING,
    TEACH_MOVING_TO_START,
    TEACH_RECORDING,
)


class ArmWorker(QObject):
    status_updated = Signal(object)
    log_message = Signal(str)
    waypoints_changed = Signal(object)
    connection_finished = Signal(bool, str)

    def __init__(self) -> None:
        super().__init__()
        self._session = ArmSession()
        self._store = WaypointStore()
        self._timer: Optional[QTimer] = None
        self._hold: Optional[tuple[int, int]] = None
        self._step_rad = DEFAULT_STEP_DEG * DEG2RAD
        self._rate_rad_s = DEFAULT_RATE_DEG_S * DEG2RAD
        self._play_index = -1
        self._play_sent_at = 0.0
        self._stopping = False
        self._connecting = False
        self._speed_percent = 30

    @Slot()
    def start(self) -> None:
        self._timer = QTimer(self)
        self._timer.setInterval(int(POLL_PERIOD_S * 1000))
        self._timer.timeout.connect(self._tick)
        self._timer.start()
        self._emit_status()

    @Slot()
    def shutdown(self) -> None:
        self._stopping = True
        self._hold = None
        self._play_index = -1
        if self._timer is not None:
            self._timer.stop()
        try:
            self._session.disconnect(disable=True)
        except Exception as exc:
            self.log_message.emit(f"断开失败: {exc}")
        self._emit_status()

    @Slot(str, str)
    def connect_arm(self, channel: str, interface: str) -> None:
        self._connecting = True
        self._emit_status()
        self.log_message.emit(f"正在连接 {channel} …")
        try:
            sv = self._session.connect(channel, interface=interface)
            self._session.set_speed_percent(self._speed_percent)
            self.log_message.emit(f"已连接 {channel}，固件 {sv}")
            self.connection_finished.emit(True, sv)
        except Exception as exc:
            self.log_message.emit(str(exc))
            self.connection_finished.emit(False, str(exc))
        finally:
            self._connecting = False
            self._emit_status()

    @Slot()
    def disconnect_arm(self) -> None:
        self._hold = None
        self._play_index = -1
        try:
            self._session.disconnect(disable=True)
            self.log_message.emit("已断开")
        except Exception as exc:
            self.log_message.emit(f"断开失败: {exc}")
        self._emit_status()

    @Slot()
    def enable_arm(self) -> None:
        try:
            ok = self._session.enable()
            self.log_message.emit("使能成功" if ok else "使能失败或超时")
        except Exception as exc:
            self.log_message.emit(str(exc))
        self._emit_status()

    @Slot()
    def disable_arm(self) -> None:
        self._hold = None
        self._play_index = -1
        try:
            ok = self._session.disable()
            self.log_message.emit("已失能" if ok else "失能失败或超时")
        except Exception as exc:
            self.log_message.emit(str(exc))
        self._emit_status()

    @Slot()
    def emergency_stop(self) -> None:
        self._hold = None
        self._play_index = -1
        try:
            self._session.emergency_stop()
            self.log_message.emit("已急停")
        except Exception as exc:
            self.log_message.emit(f"急停失败: {exc}")
        self._emit_status()

    @Slot()
    def reset_arm(self) -> None:
        try:
            self._session.reset()
            self.log_message.emit("已复位运动状态")
        except Exception as exc:
            self.log_message.emit(str(exc))
        self._emit_status()

    @Slot(int)
    def set_speed_percent(self, percent: int) -> None:
        self._speed_percent = max(1, min(100, int(percent)))
        if not self._session.connected:
            return
        try:
            self._session.set_speed_percent(self._speed_percent)
        except Exception as exc:
            self.log_message.emit(str(exc))

    @Slot(float, float)
    def set_jog_params(self, step_deg: float, rate_deg_s: float) -> None:
        self._step_rad = max(0.001, float(step_deg)) * DEG2RAD
        self._rate_rad_s = max(0.1, float(rate_deg_s)) * DEG2RAD

    @Slot(int, int)
    def jog_step(self, joint_index: int, sign: int) -> None:
        if not self._can_jog():
            return
        try:
            current = self._session.seed_target_from_feedback()
            target = apply_step(current, joint_index, sign, self._step_rad)
            self._session.move_j(target)
        except Exception as exc:
            self.log_message.emit(str(exc))
        self._emit_status()

    @Slot(int, int)
    def jog_hold_start(self, joint_index: int, sign: int) -> None:
        if not self._can_jog():
            return
        try:
            self._session.seed_target_from_feedback()
            self._hold = (joint_index, sign)
        except Exception as exc:
            self.log_message.emit(str(exc))

    @Slot()
    def jog_hold_stop(self) -> None:
        self._hold = None
        if self._session.connected and self._session.enabled:
            try:
                self._session.move_j(self._session.target)
            except Exception:
                pass
        self._emit_status()

    @Slot(int, float)
    def jog_set_joint(self, joint_index: int, value_rad: float) -> None:
        if not self._can_jog():
            return
        try:
            target = set_joint(self._session.target, joint_index, value_rad)
            self._session.move_j(target)
        except Exception as exc:
            self.log_message.emit(str(exc))

    @Slot(bool)
    def set_leader_mode(self, on: bool) -> None:
        self._hold = None
        self._play_index = -1
        try:
            self._session.set_leader_mode(on)
            self.log_message.emit("已进入零力拖动" if on else "已退出零力拖动")
        except Exception as exc:
            self.log_message.emit(str(exc))
        self._emit_status()

    @Slot()
    def record_waypoint(self) -> None:
        try:
            self._session.poll()
            joints = self._session.joints
            over = joints_outside_limits(joints)
            wp = self._store.record(joints)
            if over:
                axes = "、".join(f"J{i}" for i in over)
                self.log_message.emit(
                    f"已记录 {wp.name}，但 {axes} 超出 URDF 限位（路点已钳位到限位内）"
                )
            else:
                self.log_message.emit(f"已记录 {wp.name}")
            self._emit_waypoints()
        except Exception as exc:
            self.log_message.emit(str(exc))

    @Slot(int)
    def delete_waypoint(self, index: int) -> None:
        try:
            self._store.delete(index)
            self._emit_waypoints()
        except Exception as exc:
            self.log_message.emit(str(exc))

    @Slot(int, int)
    def move_waypoint(self, index: int, delta: int) -> None:
        try:
            self._store.move(index, delta)
            self._emit_waypoints()
        except Exception as exc:
            self.log_message.emit(str(exc))

    @Slot()
    def clear_waypoints(self) -> None:
        self._store.clear()
        self._emit_waypoints()

    @Slot(int)
    def goto_waypoint(self, index: int) -> None:
        if not self._can_play():
            return
        try:
            wp = self._store.waypoints[index]
            self._session.seed_target_from_feedback()
            self._session.move_j(wp.joints)
            self.log_message.emit(f"运动到 {wp.name}")
        except Exception as exc:
            self.log_message.emit(str(exc))
        self._emit_status()

    @Slot()
    def play_waypoints(self) -> None:
        if not self._store.waypoints:
            self.log_message.emit("没有路点")
            return
        if not self._can_play():
            self.log_message.emit("当前不能回放（需使能，且非零力/固件示教）")
            return
        self._hold = None
        self._play_index = 0
        self._play_sent_at = 0.0
        self.log_message.emit(f"开始回放 {len(self._store.waypoints)} 个路点")
        self._emit_status()

    @Slot()
    def stop_playback(self) -> None:
        self._play_index = -1
        self.log_message.emit("已停止回放")
        self._emit_status()

    @Slot(str)
    def save_waypoints(self, path: str) -> None:
        try:
            self._store.save(path)
            self.log_message.emit(f"已保存 {path}")
        except Exception as ext:
            self.log_message.emit(str(ext))

    @Slot(str)
    def load_waypoints(self, path: str) -> None:
        try:
            self._store.load(path)
            self.log_message.emit(f"已加载 {path}（{len(self._store.waypoints)} 点）")
            self._emit_waypoints()
        except Exception as exc:
            self.log_message.emit(str(exc))

    @Slot(int)
    def drag_teach(self, command: int) -> None:
        self._hold = None
        self._play_index = -1
        try:
            cmd = DragTeachCmd(command)
            self._session.send_drag_teach(cmd)
            self.log_message.emit(f"固件示教指令: {cmd.name}")
            if cmd == DragTeachCmd.START_RECORD:
                time.sleep(0.15)
                snap = self._session.snapshot()
                if snap.teach_status != TEACH_RECORDING:
                    self.log_message.emit(
                        "示教状态未变为「记录中」。该固件可能未实现 0x150 拖动示教，请改用软件路点。"
                    )
        except Exception as exc:
            self.log_message.emit(str(exc))
        self._emit_status()

    def _tick(self) -> None:
        if self._stopping or self._connecting:
            self._emit_status()
            return
        if not self._session.connected:
            return
        try:
            self._session.poll()
            self._advance_playback()
            self._advance_hold()
        except Exception as exc:
            self.log_message.emit(str(exc))
        self._emit_status()

    def _advance_hold(self) -> None:
        if self._hold is None or not self._can_jog():
            return
        joint_index, sign = self._hold
        target = apply_continuous(
            self._session.target,
            joint_index,
            sign,
            self._rate_rad_s,
            POLL_PERIOD_S,
        )
        if target == self._session.target:
            return
        self._session.move_j(target)

    def _advance_playback(self) -> None:
        if self._play_index < 0:
            return
        if self._play_index >= len(self._store.waypoints):
            self._play_index = -1
            self.log_message.emit("回放完成")
            return
        if not self._can_play():
            self._play_index = -1
            self.log_message.emit("回放中断")
            return
        now = time.monotonic()
        if self._play_sent_at <= 0.0:
            wp = self._store.waypoints[self._play_index]
            self._session.move_j(wp.joints)
            self._play_sent_at = now
            self.log_message.emit(f"回放 {wp.name} ({self._play_index + 1}/{len(self._store.waypoints)})")
            return
        snap = self._session.snapshot(playing=True, play_index=self._play_index)
        elapsed = now - self._play_sent_at
        if elapsed >= PLAYBACK_SETTLE_S and snap.motion_status == MOTION_REACHED:
            self._play_index += 1
            self._play_sent_at = 0.0
            return
        if elapsed > PLAYBACK_TIMEOUT_S:
            self.log_message.emit(f"路点 {self._play_index + 1} 等待到位超时")
            self._play_index = -1

    def _can_jog(self) -> bool:
        teach = self._session.teach_status
        return (
            self._session.connected
            and self._session.enabled
            and not self._session.leader_mode
            and self._play_index < 0
            and teach not in (TEACH_RECORDING, TEACH_EXECUTING, TEACH_MOVING_TO_START)
        )

    def _can_play(self) -> bool:
        return (
            self._session.connected
            and self._session.enabled
            and not self._session.leader_mode
            and self._session.teach_status != TEACH_RECORDING
        )

    def _emit_status(self) -> None:
        snap = self._session.snapshot(
            connecting=self._connecting,
            jogging=self._hold is not None,
            playing=self._play_index >= 0,
            play_index=self._play_index,
        )
        self.status_updated.emit(snap)

    def _emit_waypoints(self) -> None:
        self.waypoints_changed.emit(self._store.to_payload())
