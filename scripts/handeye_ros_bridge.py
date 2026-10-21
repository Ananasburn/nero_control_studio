#!/usr/bin/env python3
"""ROS sidecar for Nero Control Studio eye-to-hand calibration.

Run with system Python after sourcing ROS 2 Jazzy. Does not open CAN.
GUI talks JSON on stdin/stdout; debug overlay is written as JPEG.
"""

from __future__ import annotations

import argparse
import json
import os
import select
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

# GUI venv is Python 3.13; this process must be /usr/bin/python3 + ROS.
_SRC = Path(__file__).resolve().parents[1] / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import numpy as np
from sensor_msgs.msg import CameraInfo, Image, JointState
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformException, TransformListener

import rclpy

from nero_control_studio.handeye.calibrator import (
    DetectorSettings,
    HandEyeCalibrator,
    write_jpeg_atomic,
)
from nero_control_studio.handeye.config import (
    DEFAULT_BASE_FRAME,
    DEFAULT_CAMERA_INFO_TOPIC,
    DEFAULT_CAMERA_LINK_FRAME,
    DEFAULT_DEBUG_IMAGE,
    DEFAULT_IMAGE_TOPIC,
    DEFAULT_OPTICAL_FRAME,
    DEFAULT_WORLD_FRAME,
    SOLVE_TARGET_CAMERA,
    SOLVE_TARGET_RIGHT_BASE,
    HandeyeRunConfig,
)
from nero_control_studio.handeye.geometry import xyz_quat_to_T
from nero_control_studio.handeye.joint_names import joint_state_arrays
from nero_control_studio.handeye.protocol import parse_message


def emit(event: dict) -> None:
    sys.stdout.write(json.dumps(event, ensure_ascii=False) + "\n")
    sys.stdout.flush()


def log(message: str) -> None:
    emit({"event": "log", "message": message})
    print(message, file=sys.stderr, flush=True)


def imgmsg_to_bgr(msg: Image) -> np.ndarray:
    import cv2

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("bgr8", "rgb8", "8UC3"):
        if msg.step == 0:
            img = raw.reshape(msg.height, msg.width, 3)
        else:
            row = msg.step
            img = raw.reshape(msg.height, row)[:, : msg.width * 3].reshape(
                msg.height, msg.width, 3
            )
        if msg.encoding == "rgb8":
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img.copy()
    if msg.encoding in ("mono8", "8UC1"):
        gray = raw.reshape(msg.height, msg.width)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    raise RuntimeError(f"unsupported encoding {msg.encoding}")


def lookup_T(buffer: Buffer, parent: str, child: str) -> np.ndarray:
    tf = buffer.lookup_transform(parent, child, rclpy.time.Time(), Duration(seconds=0.0))
    t = tf.transform.translation
    q = tf.transform.rotation
    return xyz_quat_to_T((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))


class OwnedProcess:
    def __init__(self, name: str, popen: subprocess.Popen) -> None:
        self.name = name
        self.popen = popen

    def stop(self) -> None:
        proc = self.popen
        if proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait(timeout=1.0)


def start_launch(name: str, args: list[str], log_path: Path) -> OwnedProcess:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    handle = open(log_path, "ab", buffering=0)
    popen = subprocess.Popen(
        args,
        stdout=handle,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    log(f"started {name} pid={popen.pid}")
    return OwnedProcess(name, popen)


class HandeyeBridgeNode(Node):
    def __init__(self, cfg: HandeyeRunConfig, calibrator: HandEyeCalibrator) -> None:
        super().__init__("nero_studio_handeye_bridge")
        self._cfg = cfg
        self._calibrator = calibrator
        self._arm = cfg.arm
        self._q = [0.0] * 7
        self._lock = threading.Lock()
        self._optical_frame = cfg.optical_frame
        self._have_image = False
        self._last_detect = 0.0
        self._last_jpeg = 0.0
        self._debug_path = Path(cfg.debug_image_path)
        self._buffer = Buffer()
        self._tf_listener = TransformListener(self._buffer, self)
        qos = qos_profile_sensor_data
        self._js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(CameraInfo, cfg.camera_info_topic, self._on_info, qos)
        self.create_subscription(Image, cfg.image_topic, self._on_image, qos)
        self.create_timer(0.05, self._publish_joints)
        self.create_timer(0.5, self._emit_status)

    def set_joints(self, arm: str, q: list[float]) -> None:
        with self._lock:
            self._arm = arm
            self._q = [float(v) for v in q]

    @property
    def arm(self) -> str:
        with self._lock:
            return self._arm

    def has_world_tf(self) -> bool:
        try:
            return bool(
                self._buffer.can_transform(
                    self._cfg.world_frame,
                    "left_base_link",
                    rclpy.time.Time(),
                    Duration(seconds=0.0),
                )
            )
        except Exception:
            return False

    def has_ee_tf(self) -> bool:
        try:
            return bool(
                self._buffer.can_transform(
                    self._cfg.world_frame,
                    self._cfg.ee_frame,
                    rclpy.time.Time(),
                    Duration(seconds=0.0),
                )
            )
        except Exception:
            return False

    def has_base_tf(self) -> bool:
        if self._cfg.solve_target != SOLVE_TARGET_RIGHT_BASE:
            return False
        try:
            return bool(
                self._buffer.can_transform(
                    self._cfg.base_frame,
                    self._cfg.ee_frame,
                    rclpy.time.Time(),
                    Duration(seconds=0.0),
                )
            )
        except Exception:
            return False

    def sample(self) -> str:
        try:
            T_world_ee = lookup_T(self._buffer, self._cfg.world_frame, self._cfg.ee_frame)
            optical = self._optical_frame or self._cfg.optical_frame
            T_link_optical = lookup_T(self._buffer, self._cfg.camera_link_frame, optical)
            T_base_ee = None
            if self._cfg.solve_target == SOLVE_TARGET_RIGHT_BASE:
                T_base_ee = lookup_T(self._buffer, self._cfg.base_frame, self._cfg.ee_frame)
        except TransformException as exc:
            hint_parent = (
                self._cfg.base_frame
                if self._cfg.solve_target == SOLVE_TARGET_RIGHT_BASE
                else self._cfg.world_frame
            )
            return (
                f"FAIL: TF {exc} "
                "(need display.launch with world root + /joint_states; "
                f"try: ros2 run tf2_ros tf2_echo {hint_parent} {self._cfg.ee_frame})"
            )
        return self._calibrator.sample_latest(T_world_ee, T_link_optical, T_base_ee=T_base_ee)

    def drop(self) -> str:
        return self._calibrator.drop_last()

    def solve(self, output: str, samples_output: str) -> str:
        return self._calibrator.solve(
            output,
            samples_output,
            solve_target=self._cfg.solve_target,
            camera_yaml=self._cfg.camera_yaml_path,
        )

    def status_event(self) -> dict:
        event = {
            "event": "status",
            "board": "DETECTED" if self._calibrator.detected else "NOT FOUND",
            "samples": self._calibrator.sample_count,
            "have_image": self._have_image,
            "have_tf": self.has_ee_tf(),
            "have_world_tf": self.has_world_tf(),
            "solve_target": self._cfg.solve_target,
        }
        if self._cfg.solve_target == SOLVE_TARGET_RIGHT_BASE:
            event["have_base_tf"] = self.has_base_tf()
        return event

    def _on_info(self, msg: CameraInfo) -> None:
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64).reshape(-1)
        self._calibrator.set_intrinsics(K, D)
        if msg.header.frame_id:
            self._optical_frame = msg.header.frame_id

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self._last_detect < 0.08:
            return
        self._last_detect = now
        try:
            bgr = imgmsg_to_bgr(msg)
        except Exception as exc:
            log(f"image convert failed: {exc}")
            return
        self._have_image = True
        _, display = self._calibrator.process_bgr(bgr)
        if now - self._last_jpeg >= 0.12:
            try:
                write_jpeg_atomic(self._debug_path, display)
            except Exception as exc:
                log(f"debug jpeg failed: {exc}")
            self._last_jpeg = now

    def _publish_joints(self) -> None:
        with self._lock:
            arm = self._arm
            q = list(self._q)
        names, positions = joint_state_arrays(arm, q)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self._js_pub.publish(msg)

    def _emit_status(self) -> None:
        emit(self.status_event())


def topic_names(node: Node) -> set[str]:
    return {name for name, _types in node.get_topic_names_and_types()}


def wait_topic(node: Node, name: str, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if name in topic_names(node):
            return True
        time.sleep(0.4)
    return False


def wait_tf(node: HandeyeBridgeNode, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if node.has_world_tf():
            return True
        time.sleep(0.4)
    return False


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Nero Control Studio hand-eye ROS bridge")
    p.add_argument("--arm", default="left", choices=("left", "right"))
    p.add_argument("--ee-frame", default="")
    p.add_argument("--pattern", default="chessboard", choices=("chessboard", "aruco"))
    p.add_argument("--cols", type=int, default=11)
    p.add_argument("--rows", type=int, default=8)
    p.add_argument("--square-size", type=float, default=0.015)
    p.add_argument("--dict", default="4x4_50")
    p.add_argument("--marker-id", type=int, default=0)
    p.add_argument("--marker-length", type=float, default=None)
    p.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    p.add_argument("--camera-info-topic", default=DEFAULT_CAMERA_INFO_TOPIC)
    p.add_argument("--world-frame", default=DEFAULT_WORLD_FRAME)
    p.add_argument("--camera-link-frame", default=DEFAULT_CAMERA_LINK_FRAME)
    p.add_argument("--optical-frame", default=DEFAULT_OPTICAL_FRAME)
    p.add_argument("--debug-image", default=str(DEFAULT_DEBUG_IMAGE))
    p.add_argument("--start-camera", action="store_true")
    p.add_argument("--nero-ws", default="")
    p.add_argument(
        "--solve-target",
        choices=(SOLVE_TARGET_CAMERA, SOLVE_TARGET_RIGHT_BASE),
        default=SOLVE_TARGET_CAMERA,
    )
    p.add_argument("--base-frame", default=DEFAULT_BASE_FRAME)
    p.add_argument("--camera-yaml", default="")
    args = p.parse_args(argv)
    if args.pattern == "aruco" and args.marker_length is None:
        p.error("--marker-length is required with --pattern aruco")
    if args.solve_target == SOLVE_TARGET_RIGHT_BASE:
        if args.arm != "right" or (args.ee_frame and "left" in args.ee_frame):
            p.error("right_base needs --arm right and the board on right_revo2_flange")
        args.arm = "right"
        if not args.ee_frame:
            args.ee_frame = "right_revo2_flange"
    elif not args.ee_frame:
        args.ee_frame = "right_revo2_flange" if args.arm == "right" else "left_revo2_flange"
    return args


def config_from_args(args: argparse.Namespace) -> HandeyeRunConfig:
    return HandeyeRunConfig(
        pattern=args.pattern,
        cols=args.cols,
        rows=args.rows,
        square_size=args.square_size,
        aruco_dict=args.dict,
        marker_id=args.marker_id,
        marker_length=args.marker_length,
        ee_frame=args.ee_frame,
        arm=args.arm,
        world_frame=args.world_frame,
        camera_link_frame=args.camera_link_frame,
        optical_frame=args.optical_frame,
        image_topic=args.image_topic,
        camera_info_topic=args.camera_info_topic,
        start_camera=args.start_camera,
        debug_image_path=args.debug_image,
        solve_target=args.solve_target,
        base_frame=args.base_frame,
        camera_yaml_path=args.camera_yaml,
    )


def handle_command(node: HandeyeBridgeNode, msg: dict) -> bool:
    cmd = str(msg.get("cmd", "")).strip().lower()
    if cmd in ("stop", "quit", "exit"):
        return False
    if cmd == "joints":
        arm = str(msg.get("arm") or node.arm)
        q = [float(v) for v in msg.get("q", [])]
        node.set_joints(arm, q)
        return True
    if cmd == "sample":
        text = node.sample()
        emit({"event": "sample_result", "ok": text.startswith("OK"), "message": text})
        emit(node.status_event())
        return True
    if cmd == "drop":
        text = node.drop()
        emit({"event": "drop_result", "ok": "dropped" in text, "message": text})
        emit(node.status_event())
        return True
    if cmd == "solve":
        output = str(msg.get("output") or "")
        samples_output = str(msg.get("samples_output") or "")
        if not output or not samples_output:
            emit({"event": "solve_result", "ok": False, "message": "FAIL: missing output paths"})
            return True
        text = node.solve(output, samples_output)
        ok = "best:" in text or text.startswith("OK")
        emit({"event": "solve_result", "ok": ok, "message": text})
        emit(node.status_event())
        return True
    if cmd == "status":
        emit(node.status_event())
        return True
    emit({"event": "error", "message": f"unknown cmd {cmd}"})
    return True


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    cfg = config_from_args(args)
    Path(cfg.debug_image_path).parent.mkdir(parents=True, exist_ok=True)

    ws = Path(args.nero_ws).expanduser() if args.nero_ws else Path.home() / "Projects/nero/ros2_ws"
    log_dir = Path("/tmp/nero_handeye_logs")
    owned: list[OwnedProcess] = []

    rclpy.init()
    settings = DetectorSettings.from_run(cfg)
    calibrator = HandEyeCalibrator(settings)
    node = HandeyeBridgeNode(cfg, calibrator)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    running = True

    def _on_signal(_signum, _frame) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        if cfg.image_topic not in topic_names(node):
            if cfg.start_camera:
                owned.append(
                    start_launch(
                        "camera",
                        [
                            "ros2",
                            "launch",
                            "orbbec_camera",
                            "astra2.launch.py",
                            "camera_name:=astra2",
                        ],
                        log_dir / "camera.log",
                    )
                )
                if not wait_topic(node, cfg.image_topic, 40.0):
                    emit(
                        {
                            "event": "error",
                            "message": f"相机话题 {cfg.image_topic} 未出现，见 {log_dir / 'camera.log'}",
                        }
                    )
                    return 1
            else:
                emit(
                    {
                        "event": "error",
                        "message": f"没有 {cfg.image_topic}。勾选启动相机，或先打开 Astra 驱动。",
                    }
                )
                return 1
        else:
            log(f"reusing existing {cfg.image_topic}")

        if not wait_tf(node, 3.0):
            owned.append(
                start_launch(
                    "display",
                    [
                        "ros2",
                        "launch",
                        "dual_nero_description",
                        "display.launch.py",
                        "use_rviz:=false",
                        "use_gui:=false",
                        "use_jsp:=false",
                    ],
                    log_dir / "display.log",
                )
            )
            if not wait_tf(node, 20.0):
                emit(
                    {
                        "event": "error",
                        "message": f"没有 world TF。检查 {ws} 是否已 colcon 安装 dual_nero_description。见 {log_dir / 'display.log'}",
                    }
                )
                return 1
        else:
            log("reusing existing world TF")

        emit({"event": "ready", "debug_image": cfg.debug_image_path})
        emit(node.status_event())
        log("hand-eye bridge ready")

        while running:
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not ready:
                continue
            line = sys.stdin.readline()
            if line == "":
                break
            try:
                msg = parse_message(line)
            except Exception as exc:
                emit({"event": "error", "message": f"bad JSON: {exc}"})
                continue
            if msg is None:
                continue
            if not handle_command(node, msg):
                break
        return 0
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        for proc in owned:
            try:
                proc.stop()
                log(f"stopped {proc.name}")
            except Exception as exc:
                log(f"stop {proc.name} failed: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
