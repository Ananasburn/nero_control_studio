"""Defaults and run configuration for Studio-owned eye-to-hand calibration."""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path

from nero_control_studio.handeye.joint_names import arm_from_channel, ee_frame_for_arm

DEFAULT_PATTERN = "chessboard"
DEFAULT_COLS = 11
DEFAULT_ROWS = 8
DEFAULT_SQUARE_SIZE = 0.015
DEFAULT_ARUCO_DICT = "4x4_50"
DEFAULT_MARKER_ID = 0
DEFAULT_MARKER_LENGTH = 0.08
DEFAULT_TARGET_SAMPLES = 18
MIN_SOLVE_SAMPLES = 8

DEFAULT_IMAGE_TOPIC = "/astra2/color/image_raw"
DEFAULT_CAMERA_INFO_TOPIC = "/astra2/color/camera_info"
DEFAULT_WORLD_FRAME = "world"
DEFAULT_CAMERA_LINK_FRAME = "astra2_link"
DEFAULT_OPTICAL_FRAME = "astra2_color_optical_frame"
DEFAULT_BASE_FRAME = "right_base_link"
SOLVE_TARGET_CAMERA = "camera"
SOLVE_TARGET_RIGHT_BASE = "right_base"

DEFAULT_ROS_SETUP = "/opt/ros/jazzy/setup.bash"
DEFAULT_SYSTEM_PYTHON = "/usr/bin/python3"
DEFAULT_DEBUG_IMAGE = Path("/tmp/nero_handeye_debug.jpg")


def default_output_dir() -> Path:
    return Path.home() / "nero_handeye"


def default_yaml_path() -> Path:
    return default_output_dir() / "scene_tf_camera_calibrated.yaml"


def default_samples_path() -> Path:
    return default_output_dir() / "handeye_samples.json"


def default_right_base_yaml_path() -> Path:
    return default_output_dir() / "scene_tf_right_base_calibrated.yaml"


def default_camera_yaml_path() -> Path:
    scene = nero_ws() / "src" / "dual_nero_sim" / "dual_nero_description" / "config" / "scene_tf.yaml"
    if scene.is_file():
        return scene
    return default_yaml_path()


def nero_ws() -> Path:
    raw = os.environ.get("NERO_WS", "").strip()
    if raw:
        return Path(raw).expanduser()
    return Path.home() / "Projects/nero/ros2_ws"


def ros_setup_path() -> Path:
    raw = os.environ.get("NERO_ROS_SETUP", "").strip()
    if raw:
        return Path(raw)
    return Path(DEFAULT_ROS_SETUP)


def system_python() -> Path:
    raw = os.environ.get("NERO_SYSTEM_PYTHON", "").strip()
    if raw:
        return Path(raw)
    return Path(DEFAULT_SYSTEM_PYTHON)


def repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def package_src_root() -> Path:
    return Path(__file__).resolve().parents[2]


def bridge_script_path() -> Path:
    return repo_root() / "scripts" / "handeye_ros_bridge.py"


ARUCO_DICT_NAMES = (
    "4x4_50",
    "4x4_100",
    "5x5_50",
    "5x5_100",
    "6x6_50",
    "6x6_250",
)


@dataclass
class HandeyeRunConfig:
    pattern: str = DEFAULT_PATTERN
    cols: int = DEFAULT_COLS
    rows: int = DEFAULT_ROWS
    square_size: float = DEFAULT_SQUARE_SIZE
    aruco_dict: str = DEFAULT_ARUCO_DICT
    marker_id: int = DEFAULT_MARKER_ID
    marker_length: float | None = None
    ee_frame: str = "left_revo2_flange"
    arm: str = "left"
    world_frame: str = DEFAULT_WORLD_FRAME
    camera_link_frame: str = DEFAULT_CAMERA_LINK_FRAME
    optical_frame: str = DEFAULT_OPTICAL_FRAME
    image_topic: str = DEFAULT_IMAGE_TOPIC
    camera_info_topic: str = DEFAULT_CAMERA_INFO_TOPIC
    target_samples: int = DEFAULT_TARGET_SAMPLES
    start_camera: bool = True
    yaml_path: str = field(default_factory=lambda: str(default_yaml_path()))
    samples_path: str = field(default_factory=lambda: str(default_samples_path()))
    debug_image_path: str = field(default_factory=lambda: str(DEFAULT_DEBUG_IMAGE))
    solve_target: str = SOLVE_TARGET_CAMERA
    base_frame: str = DEFAULT_BASE_FRAME
    camera_yaml_path: str = field(default_factory=lambda: str(default_camera_yaml_path()))

    @classmethod
    def for_channel(cls, channel: str) -> HandeyeRunConfig:
        arm = arm_from_channel(channel)
        return cls(arm=arm, ee_frame=ee_frame_for_arm(arm))
