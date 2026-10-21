from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

from nero_control_studio.handeye.config import (
    HandeyeRunConfig,
    MIN_SOLVE_SAMPLES,
    SOLVE_TARGET_RIGHT_BASE,
)
from nero_control_studio.handeye.calibrator import (
    HandEyeCalibrator,
    format_camera_yaml,
    load_named_pose,
    right_base_config_error,
    solve_hand_eye,
)
from nero_control_studio.handeye.geometry import (
    camera_looks_like_placeholder,
    compose_T_world_base,
    invert_T,
    rot_to_rpy,
    rpy_to_R,
    se3_delta,
    xyz_rpy_to_T,
)
from nero_control_studio.handeye.joint_names import (
    ARM_JOINTS,
    HAND_JOINTS,
    arm_from_channel,
    ee_frame_for_arm,
    joint_state_arrays,
)
from nero_control_studio.handeye.protocol import encode_cmd, parse_message


def test_arm_from_channel() -> None:
    assert arm_from_channel("can_left") == "left"
    assert arm_from_channel("can0") == "left"
    assert arm_from_channel("can_right") == "right"
    assert arm_from_channel("can1") == "right"
    assert ee_frame_for_arm("left") == "left_revo2_flange"
    assert ee_frame_for_arm("right") == "right_revo2_flange"
    cfg = HandeyeRunConfig.for_channel("can_right")
    assert cfg.arm == "right"
    assert cfg.ee_frame == "right_revo2_flange"


def test_joint_state_arrays_puts_arm_in_named_slots() -> None:
    q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    names, pos = joint_state_arrays("left", q)
    assert names[:7] == ARM_JOINTS["left"]
    assert names[7:14] == ARM_JOINTS["right"]
    assert names[14:] == HAND_JOINTS
    assert pos[:7] == pytest.approx(q)
    assert pos[7:] == pytest.approx([0.0] * (len(pos) - 7))
    names_r, pos_r = joint_state_arrays("right", q)
    assert pos_r[7:14] == pytest.approx(q)
    assert pos_r[:7] == pytest.approx([0.0] * 7)


def test_environment_error_does_not_point_at_external_calib_venv() -> None:
    from nero_control_studio.handeye.session import environment_error

    err = environment_error()
    if err:
        assert "eye_to_hand_calibration" not in err


def test_protocol_roundtrip() -> None:
    line = encode_cmd("solve", output="/tmp/a.yaml", samples_output="/tmp/b.json")
    assert line.endswith("\n")
    msg = parse_message(line)
    assert msg is not None
    assert msg["cmd"] == "solve"
    assert msg["output"] == "/tmp/a.yaml"
    assert parse_message("  ") is None
    joints = parse_message(encode_cmd("joints", arm="left", q=[0.0] * 7))
    assert joints is not None
    assert joints["arm"] == "left"
    assert len(joints["q"]) == 7


def test_invert_and_rpy_roundtrip() -> None:
    T = xyz_rpy_to_T([0.1, -0.2, 0.3], [0.2, -0.1, 0.4])
    I = invert_T(T) @ T
    assert I == pytest.approx(np.eye(4), abs=1e-9)
    rpy = [0.15, -0.25, 0.35]
    assert rot_to_rpy(rpy_to_R(rpy)) == pytest.approx(rpy, abs=1e-9)


def test_sample_drop_and_yaml_format(tmp_path: Path) -> None:
    cal = HandEyeCalibrator()
    T = np.eye(4)
    T[0, 3] = 0.25
    assert cal.sample(T, np.eye(4), np.eye(4)).startswith("OK sample #1")
    assert cal.sample_count == 1
    assert "dropped last, now 0" in cal.drop_last()
    assert cal.drop_last() == "no samples"
    text = format_camera_yaml(
        [0.1, 0.2, 0.3],
        [0.0, 1.57, 0.0],
        "astra2_link",
        "TSAI",
        "invert",
        4.2,
        12,
        [0.01, 0.0, 0.05],
    )
    assert "camera:" in text
    assert "root_frame: astra2_link" in text
    yaml_path = tmp_path / "a.yaml"
    yaml_path.write_text(text, encoding="utf-8")
    assert yaml_path.read_text(encoding="utf-8") == text


def _synthetic_samples(n: int = 12) -> tuple[list[dict], np.ndarray]:
    T_world_cam = xyz_rpy_to_T([0.42, 0.05, 0.85], [0.0, 2.6, 0.1])
    T_ee_board = xyz_rpy_to_T([0.02, -0.01, 0.06], [0.05, 0.0, 0.1])
    T_link_optical = xyz_rpy_to_T([0.0, 0.0, 0.0], [-1.5708, 0.0, -1.5708])
    samples = []
    for i in range(n):
        T_world_ee = xyz_rpy_to_T(
            [0.22 + 0.04 * (i % 4), 0.08 * ((i % 5) - 2), 0.18 + 0.03 * (i % 3)],
            [0.15 * ((i % 3) - 1), 0.2 * ((i % 4) - 1.5), 0.35 * i],
        )
        T_cam_board = invert_T(T_world_cam) @ T_world_ee @ T_ee_board
        samples.append(
            {
                "T_world_ee": T_world_ee.tolist(),
                "T_cam_board": T_cam_board.tolist(),
                "T_link_optical": T_link_optical.tolist(),
            }
        )
    return samples, T_world_cam


def test_solve_writes_independent_paths(tmp_path: Path) -> None:
    samples, T_world_cam = _synthetic_samples(12)
    assert len(samples) >= MIN_SOLVE_SAMPLES
    best, _lines = solve_hand_eye(samples)
    assert best is not None
    recovered = best["T_world_cam"]
    assert recovered[:3, 3] == pytest.approx(T_world_cam[:3, 3], abs=0.05)

    cal = HandEyeCalibrator()
    for sample in samples:
        cal.sample(
            np.array(sample["T_world_ee"]),
            np.array(sample["T_cam_board"]),
            np.array(sample["T_link_optical"]),
        )
    yaml_path = tmp_path / "out" / "camera.yaml"
    samples_path = tmp_path / "data" / "samples.json"
    text = cal.solve(yaml_path, samples_path)
    assert "best:" in text
    assert yaml_path.is_file()
    assert samples_path.is_file()
    assert yaml_path.read_text(encoding="utf-8").startswith("# Paste")
    loaded = json.loads(samples_path.read_text(encoding="utf-8"))
    assert len(loaded) == 12


def _assert_close_T(name: str, T_got: np.ndarray, T_want: np.ndarray, mm: float, deg: float) -> None:
    dt_mm, dR = se3_delta(T_want, T_got)
    n = float(np.linalg.norm(dt_mm))
    if n > mm or dR > deg:
        raise AssertionError(
            f"{name}: |dt|={n:.3f} mm (max {mm}), dR={dR:.3f} deg (max {deg})\n"
            f"  want xyz={T_want[:3, 3].tolist()} rpy={rot_to_rpy(T_want[:3, :3])}\n"
            f"  got  xyz={T_got[:3, 3].tolist()} rpy={rot_to_rpy(T_got[:3, :3])}"
        )


def _excited_base_ee(i: int) -> np.ndarray:
    xyz = [
        0.18 + 0.04 * (i % 4),
        0.08 * math.sin(0.9 * i),
        0.12 + 0.03 * (i % 5),
    ]
    rpy = [
        0.35 * math.sin(1.1 * i),
        0.40 * math.cos(0.7 * i),
        0.55 * i,
    ]
    return xyz_rpy_to_T(xyz, rpy)


def test_recover_tilted_right_base() -> None:
    T_world_base_true = xyz_rpy_to_T(
        [0.012, -0.351, 0.018],
        [-0.040, 0.025, -math.pi / 2],
    )
    T_world_optical = xyz_rpy_to_T(
        [0.274244, 0.040922, 1.041085],
        [-1.343059393, 1.538410834, -1.332287444],
    )
    T_flange_board = xyz_rpy_to_T([-0.016, 0.084, 0.013], [0.12, -0.08, 0.40])

    samples = []
    for i in range(12):
        T_base_ee = _excited_base_ee(i)
        T_world_board = T_world_base_true @ T_base_ee @ T_flange_board
        T_cam_board = invert_T(T_world_optical) @ T_world_board
        samples.append(
            {
                "T_base_ee": T_base_ee.tolist(),
                "T_cam_board": T_cam_board.tolist(),
            }
        )

    best, lines = solve_hand_eye(samples, "T_base_ee")
    assert best is not None, "\n".join(lines)
    if best["error_mm"] > 0.05:
        raise AssertionError(
            f"flange-board std {best['error_mm']:.4f} mm; expected ~0 on noiseless data\n"
            + "\n".join(lines)
        )
    T_world_base = compose_T_world_base(T_world_optical, best["T_parent_optical"])
    _assert_close_T("T_world_right_base", T_world_base, T_world_base_true, mm=0.5, deg=0.05)

    T_world_base_yaml = xyz_rpy_to_T([0.0, -0.36, 0.0], [0.0, 0.0, -math.pi / 2])
    dt_mm, dR = se3_delta(T_world_base_yaml, T_world_base_true)
    if float(np.linalg.norm(dt_mm)) < 5.0 and dR < 1.0:
        raise AssertionError("fixture is too close to the design right: pose")


def test_load_named_pose_skips_comments(tmp_path: Path) -> None:
    path = tmp_path / "scene_tf.yaml"
    path.write_text(
        "# Unified world extrinsics\n"
        "left:\n"
        "  xyz: [0.0, 0.36, 0.0]\n"
        "  rpy: [0.0, 0.0, 1.57079632679]   # yaw +pi/2\n"
        "right:\n"
        "  xyz: [0.0, -0.36, 0.0]\n"
        "  rpy: [0.0, 0.0, -1.57079632679]\n"
        "camera:\n"
        "  # xyz: [0.30, 0.0, 0.80]\n"
        "  xyz: [0.274244, 0.040922, 1.041085]\n"
        "  rpy: [-1.343059393, 1.538410834, -1.332287444]\n"
        "  root_frame: astra2_link\n",
        encoding="utf-8",
    )
    T_cam, cam_block, data = load_named_pose(path, "camera")
    assert cam_block["xyz"] == pytest.approx([0.274244, 0.040922, 1.041085])
    assert cam_block["root_frame"] == "astra2_link"
    assert T_cam[:3, 3] == pytest.approx(cam_block["xyz"])
    _, right_block, _ = load_named_pose(path, "right")
    assert right_block["xyz"] == pytest.approx([0.0, -0.36, 0.0])
    assert "right" in data
    assert not camera_looks_like_placeholder(cam_block["xyz"])
    assert camera_looks_like_placeholder([0.30, 0.0, 0.80])


def test_right_base_solve_writes_yaml(tmp_path: Path) -> None:
    T_world_base_true = xyz_rpy_to_T(
        [0.012, -0.351, 0.018],
        [-0.040, 0.025, -math.pi / 2],
    )
    T_world_optical = xyz_rpy_to_T(
        [0.274244, 0.040922, 1.041085],
        [-1.343059393, 1.538410834, -1.332287444],
    )
    T_link_optical = np.eye(4)
    T_flange_board = xyz_rpy_to_T([-0.016, 0.084, 0.013], [0.12, -0.08, 0.40])
    cam_xyz = T_world_optical[:3, 3].tolist()
    cam_rpy = rot_to_rpy(T_world_optical[:3, :3])
    cam_yaml = tmp_path / "locked_camera.yaml"
    cam_yaml.write_text(
        format_camera_yaml(cam_xyz, cam_rpy, "astra2_link", "PARK", "invert", 0.1, 12, [0.0, 0.0, 0.0])
        + "right:\n"
        "  xyz: [0.0, -0.36, 0.0]\n"
        "  rpy: [0.0, 0.0, -1.570796327]\n",
        encoding="utf-8",
    )

    cal = HandEyeCalibrator()
    for i in range(12):
        T_base_ee = _excited_base_ee(i)
        T_world_board = T_world_base_true @ T_base_ee @ T_flange_board
        T_cam_board = invert_T(T_world_optical) @ T_world_board
        T_world_ee = T_world_base_true @ T_base_ee
        cal.sample(T_world_ee, T_cam_board, T_link_optical, T_base_ee=T_base_ee, solve_target="right_base")

    yaml_path = tmp_path / "out" / "right.yaml"
    samples_path = tmp_path / "data" / "right_samples.json"
    text = cal.solve(yaml_path, samples_path, solve_target="right_base", camera_yaml=cam_yaml)
    assert "best:" in text
    written = yaml_path.read_text(encoding="utf-8")
    assert written.startswith("# Paste")
    assert "right:" in written
    assert "Do NOT change camera:" in written
    loaded = json.loads(samples_path.read_text(encoding="utf-8"))
    assert len(loaded) == 12
    assert "T_base_ee" in loaded[0]
    _, right_block, _ = load_named_pose(yaml_path, "right")
    T_got = xyz_rpy_to_T(right_block["xyz"], right_block["rpy"])
    _assert_close_T("written right:", T_got, T_world_base_true, mm=0.5, deg=0.05)


def test_right_base_config_error_rejects_left_arm() -> None:
    cfg = HandeyeRunConfig.for_channel("can_left")
    cfg.solve_target = SOLVE_TARGET_RIGHT_BASE
    err = right_base_config_error(cfg)
    assert err is not None
    assert "can_right" in err


def test_right_base_config_error_rejects_placeholder(tmp_path: Path) -> None:
    cam = tmp_path / "placeholder.yaml"
    cam.write_text("camera:\n  xyz: [0.30, 0.0, 0.80]\n  rpy: [0.0, 0.0, 0.0]\n", encoding="utf-8")
    cfg = HandeyeRunConfig.for_channel("can_right")
    cfg.solve_target = SOLVE_TARGET_RIGHT_BASE
    cfg.camera_yaml_path = str(cam)
    err = right_base_config_error(cfg)
    assert err is not None
    assert "占位" in err
