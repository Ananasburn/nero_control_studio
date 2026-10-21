"""Eye-to-hand board detection, sampling, and OpenCV HandEye solve.

ROS-free. The sidecar supplies images, camera intrinsics, and TF poses.
"""

from __future__ import annotations

import ast
import json
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from nero_control_studio.handeye.config import (
    DEFAULT_ARUCO_DICT,
    DEFAULT_CAMERA_LINK_FRAME,
    DEFAULT_COLS,
    DEFAULT_MARKER_ID,
    DEFAULT_PATTERN,
    DEFAULT_ROWS,
    DEFAULT_SQUARE_SIZE,
    DEFAULT_BASE_FRAME,
    MIN_SOLVE_SAMPLES,
    SOLVE_TARGET_CAMERA,
    SOLVE_TARGET_RIGHT_BASE,
    HandeyeRunConfig,
)
from nero_control_studio.handeye.geometry import (
    camera_looks_like_placeholder,
    compose_T_world_base,
    flange_board_consistency_mm,
    invert_T,
    mean_T_ee_board,
    park_hand_eye,
    rot_to_rpy,
    se3_delta,
    xyz_rpy_to_T,
)

try:
    import cv2
except ImportError:  # pragma: no cover - optional in the GUI venv
    cv2 = None  # type: ignore[assignment]


def _require_cv2():
    if cv2 is None:
        raise RuntimeError("OpenCV (cv2) is required for hand-eye detection and solve")
    return cv2


def aruco_dict_map() -> dict[str, int]:
    cv = _require_cv2()
    return {
        "4x4_50": cv.aruco.DICT_4X4_50,
        "4x4_100": cv.aruco.DICT_4X4_100,
        "5x5_50": cv.aruco.DICT_5X5_50,
        "5x5_100": cv.aruco.DICT_5X5_100,
        "6x6_50": cv.aruco.DICT_6X6_50,
        "6x6_250": cv.aruco.DICT_6X6_250,
    }


def hand_eye_methods() -> dict[str, int]:
    cv = _require_cv2()
    if not hasattr(cv, "calibrateHandEye"):
        return {}
    return {
        "TSAI": cv.CALIB_HAND_EYE_TSAI,
        "PARK": cv.CALIB_HAND_EYE_PARK,
        "HORAUD": cv.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv.CALIB_HAND_EYE_DANIILIDIS,
    }


def rvec_tvec_to_T(rvec, tvec) -> np.ndarray:
    cv = _require_cv2()
    Rm, _ = cv.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    T = np.eye(4)
    T[:3, :3] = Rm
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return T


@dataclass
class DetectorSettings:
    pattern: str = DEFAULT_PATTERN
    cols: int = DEFAULT_COLS
    rows: int = DEFAULT_ROWS
    square_size: float = DEFAULT_SQUARE_SIZE
    aruco_dict: str = DEFAULT_ARUCO_DICT
    marker_id: int = DEFAULT_MARKER_ID
    marker_length: float | None = None
    camera_link_frame: str = DEFAULT_CAMERA_LINK_FRAME
    solve_target: str = SOLVE_TARGET_CAMERA
    camera_yaml_path: str = ""
    base_frame: str = DEFAULT_BASE_FRAME

    @classmethod
    def from_run(cls, cfg: HandeyeRunConfig) -> DetectorSettings:
        return cls(
            pattern=cfg.pattern,
            cols=cfg.cols,
            rows=cfg.rows,
            square_size=cfg.square_size,
            aruco_dict=cfg.aruco_dict,
            marker_id=cfg.marker_id,
            marker_length=cfg.marker_length,
            camera_link_frame=cfg.camera_link_frame,
            solve_target=cfg.solve_target,
            camera_yaml_path=cfg.camera_yaml_path,
            base_frame=cfg.base_frame,
        )


@dataclass
class HandEyeCalibrator:
    settings: DetectorSettings = field(default_factory=DetectorSettings)

    def __post_init__(self) -> None:
        self._K: np.ndarray | None = None
        self._D: np.ndarray | None = None
        self._latest_T_cam_board: np.ndarray | None = None
        self._latest_debug: np.ndarray | None = None
        self._detected = False
        self._samples: list[dict[str, Any]] = []
        self._lock = threading.Lock()
        self._chess_objp: np.ndarray | None = None
        self._aruco_detector = None
        self._aruco_dictionary = None
        self._aruco_params = None
        self._setup_pattern()

    def _setup_pattern(self) -> None:
        settings = self.settings
        if settings.pattern == "chessboard":
            n = settings.cols * settings.rows
            objp = np.zeros((n, 3), np.float32)
            objp[:, :2] = np.mgrid[0 : settings.cols, 0 : settings.rows].T.reshape(-1, 2)
            objp *= settings.square_size
            self._chess_objp = objp
            return
        cv = _require_cv2()
        dictionary = cv.aruco.getPredefinedDictionary(aruco_dict_map()[settings.aruco_dict])
        params = cv.aruco.DetectorParameters()
        if hasattr(cv.aruco, "ArucoDetector"):
            self._aruco_detector = cv.aruco.ArucoDetector(dictionary, params)
        else:
            self._aruco_dictionary = dictionary
            self._aruco_params = params

    def set_intrinsics(self, K: np.ndarray, D: np.ndarray | None) -> None:
        with self._lock:
            self._K = np.asarray(K, dtype=np.float64).reshape(3, 3)
            self._D = np.zeros(5) if D is None else np.asarray(D, dtype=np.float64).reshape(-1)

    @property
    def sample_count(self) -> int:
        with self._lock:
            return len(self._samples)

    @property
    def detected(self) -> bool:
        with self._lock:
            return self._detected

    @property
    def latest_debug(self) -> np.ndarray | None:
        with self._lock:
            if self._latest_debug is None:
                return None
            return self._latest_debug.copy()

    def process_bgr(self, bgr: np.ndarray) -> tuple[np.ndarray | None, np.ndarray]:
        cv = _require_cv2()
        with self._lock:
            K = None if self._K is None else self._K.copy()
            D = None if self._D is None else self._D.copy()
            n_samples = len(self._samples)
        T_cam_board, display = self._detect(bgr, K, D, n_samples, cv)
        with self._lock:
            self._latest_T_cam_board = None if T_cam_board is None else T_cam_board.copy()
            self._detected = T_cam_board is not None
            self._latest_debug = display
        return T_cam_board, display

    def sample(
        self,
        T_world_ee: np.ndarray,
        T_cam_board: np.ndarray,
        T_link_optical: np.ndarray,
        T_base_ee: np.ndarray | None = None,
        solve_target: str | None = None,
    ) -> str:
        target = solve_target or self.settings.solve_target
        rec: dict[str, Any] = {
            "T_world_ee": np.asarray(T_world_ee, dtype=np.float64).tolist(),
            "T_cam_board": np.asarray(T_cam_board, dtype=np.float64).tolist(),
            "T_link_optical": np.asarray(T_link_optical, dtype=np.float64).tolist(),
            "solve_target": target,
        }
        if T_base_ee is not None:
            rec["T_base_ee"] = np.asarray(T_base_ee, dtype=np.float64).tolist()
        elif target == SOLVE_TARGET_RIGHT_BASE:
            return "FAIL: --base-frame is required for right_base"
        with self._lock:
            self._samples.append(rec)
            n = len(self._samples)
        if T_base_ee is not None and target == SOLVE_TARGET_RIGHT_BASE:
            tcp = np.asarray(T_base_ee, dtype=np.float64)[:3, 3]
            return (
                f"OK sample #{n}  "
                f"base→flange xyz=[{tcp[0]:.3f}, {tcp[1]:.3f}, {tcp[2]:.3f}]"
            )
        tcp = np.asarray(T_world_ee, dtype=np.float64)[:3, 3]
        return f"OK sample #{n}  flange xyz=[{tcp[0]:.3f}, {tcp[1]:.3f}, {tcp[2]:.3f}]"

    def sample_latest(
        self,
        T_world_ee: np.ndarray,
        T_link_optical: np.ndarray,
        T_base_ee: np.ndarray | None = None,
    ) -> str:
        with self._lock:
            T_cam_board = (
                None if self._latest_T_cam_board is None else self._latest_T_cam_board.copy()
            )
        if T_cam_board is None:
            return "FAIL: board not detected in current frame"
        return self.sample(T_world_ee, T_cam_board, T_link_optical, T_base_ee=T_base_ee)

    def drop_last(self) -> str:
        with self._lock:
            if not self._samples:
                return "no samples"
            self._samples.pop()
            n = len(self._samples)
        return f"dropped last, now {n}"

    def solve(
        self,
        output: str | Path,
        samples_output: str | Path,
        *,
        solve_target: str | None = None,
        camera_yaml: str | Path | None = None,
    ) -> str:
        target = solve_target or self.settings.solve_target
        cam_yaml = self.settings.camera_yaml_path if camera_yaml is None else str(camera_yaml)
        with self._lock:
            samples = [dict(s) for s in self._samples]
        if len(samples) < MIN_SOLVE_SAMPLES:
            return f"FAIL: need >= {MIN_SOLVE_SAMPLES} samples, have {len(samples)}"
        if target == SOLVE_TARGET_RIGHT_BASE:
            return self._solve_right_base(samples, output, samples_output, cam_yaml)
        return self._solve_camera(samples, output, samples_output)

    def _solve_camera(
        self,
        samples: list[dict[str, Any]],
        output: str | Path,
        samples_output: str | Path,
    ) -> str:
        best, lines = solve_hand_eye(samples, "T_world_ee")
        if best is None:
            return "FAIL: all HandEye methods failed\n" + "\n".join(lines)

        T_world_optical = best["T_parent_optical"]
        T_link_optical = np.array(samples[-1]["T_link_optical"], dtype=np.float64)
        T_world_link = T_world_optical @ invert_T(T_link_optical)
        xyz = T_world_link[:3, 3].tolist()
        rpy = rot_to_rpy(T_world_link[:3, :3])
        T_ee_board = mean_T_ee_board(samples, T_world_optical, "T_world_ee")
        board_xyz = T_ee_board[:3, 3].tolist()
        yaml_path, samples_path = _write_outputs(
            output,
            samples_output,
            format_camera_yaml(
                xyz,
                rpy,
                self.settings.camera_link_frame,
                best["method"],
                best["convention"],
                best["error_mm"],
                len(samples),
                board_xyz,
            ),
            samples,
        )
        return _format_best(best, lines, xyz, rpy, board_xyz, yaml_path, samples_path)

    def _solve_right_base(
        self,
        samples: list[dict[str, Any]],
        output: str | Path,
        samples_output: str | Path,
        camera_yaml: str,
    ) -> str:
        missing = [i for i, s in enumerate(samples, 1) if "T_base_ee" not in s]
        if missing:
            shown = missing[:8]
            return (
                "FAIL: samples lack T_base_ee "
                f"(indices {shown}). Restart calibrator with --solve-target right_base."
            )
        cam_path = Path(camera_yaml)
        try:
            T_world_link, cam_block, cam_data = load_named_pose(cam_path, "camera")
        except Exception as exc:
            return f"FAIL: camera yaml {cam_path}: {exc}"
        if camera_looks_like_placeholder(cam_block["xyz"]):
            return (
                f"FAIL: {cam_path} camera: still looks like the URDF placeholder. "
                "Finish left-arm eye-to-hand and paste camera: first."
            )
        best, lines = solve_hand_eye(samples, "T_base_ee")
        if best is None:
            return "FAIL: all HandEye methods failed\n" + "\n".join(lines)
        T_base_optical = best["T_parent_optical"]
        T_link_optical = np.array(samples[-1]["T_link_optical"], dtype=np.float64)
        T_world_optical = T_world_link @ T_link_optical
        T_world_base = compose_T_world_base(T_world_optical, T_base_optical)
        xyz = T_world_base[:3, 3].tolist()
        rpy = rot_to_rpy(T_world_base[:3, :3])
        T_ee_board = mean_T_ee_board(samples, T_base_optical, "T_base_ee")
        board_xyz = T_ee_board[:3, 3].tolist()
        prev_note = ""
        if isinstance(cam_data, dict) and "right" in cam_data:
            prev = cam_data["right"]
            if "xyz" in prev and "rpy" in prev:
                T_prev = xyz_rpy_to_T(prev["xyz"], prev["rpy"])
                dt_mm, dR = se3_delta(T_prev, T_world_base)
                prev_note = (
                    f"# previous right in {cam_path.name}: xyz={list(prev['xyz'])} "
                    f"rpy={list(prev['rpy'])}\n"
                    f"# delta vs previous: dxyz_mm={[round(float(v), 2) for v in dt_mm]} "
                    f"|dt|={float(np.linalg.norm(dt_mm)):.2f} mm  dR={dR:.3f} deg\n"
                )
        yaml_path, samples_path = _write_outputs(
            output,
            samples_output,
            format_right_base_yaml(
                xyz,
                rpy,
                best["method"],
                best["convention"],
                best["error_mm"],
                len(samples),
                board_xyz,
                cam_path,
                cam_block,
                prev_note,
            ),
            samples,
        )
        extra = (
            f"\nlocked camera {cam_path.name}  "
            f"xyz={list(cam_block['xyz'])}  rpy={list(cam_block['rpy'])}"
        )
        if prev_note:
            extra += "\n" + prev_note.rstrip()
        return _format_best(best, lines, xyz, rpy, board_xyz, yaml_path, samples_path) + extra

    def _detect(
        self,
        bgr: np.ndarray,
        K: np.ndarray | None,
        D: np.ndarray | None,
        n_samples: int,
        cv: Any,
    ) -> tuple[np.ndarray | None, np.ndarray]:
        gray = cv.cvtColor(bgr, cv.COLOR_BGR2GRAY)
        display = bgr.copy()
        T_cam_board = None
        settings = self.settings
        if settings.pattern == "chessboard":
            hint = f"expect chessboard inner {settings.cols}x{settings.rows}"
            size = (settings.cols, settings.rows)
            flags = cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE
            ok, corners = cv.findChessboardCorners(gray, size, flags)
            if not ok and hasattr(cv, "findChessboardCornersSB"):
                ok, corners = cv.findChessboardCornersSB(
                    gray, size, flags=cv.CALIB_CB_EXHAUSTIVE | cv.CALIB_CB_ACCURACY
                )
            if ok:
                criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                cv.drawChessboardCorners(display, size, corners, True)
                if K is not None:
                    pnp_ok, rvec, tvec = cv.solvePnP(
                        self._chess_objp,
                        corners,
                        K,
                        D,
                        flags=cv.SOLVEPNP_ITERATIVE,
                    )
                    if pnp_ok:
                        T_cam_board = rvec_tvec_to_T(rvec, tvec)
                        cv.drawFrameAxes(display, K, D, rvec, tvec, settings.square_size * 3)
        else:
            hint = f"expect ArUco id={settings.marker_id} dict={settings.aruco_dict}"
            if self._aruco_detector is not None:
                corners, ids, _ = self._aruco_detector.detectMarkers(gray)
            else:
                corners, ids, _ = cv.aruco.detectMarkers(
                    gray, self._aruco_dictionary, parameters=self._aruco_params
                )
            if ids is not None:
                cv.aruco.drawDetectedMarkers(display, corners, ids)
                ids_flat = ids.flatten()
                if settings.marker_id in ids_flat and K is not None:
                    idx = int(np.where(ids_flat == settings.marker_id)[0][0])
                    half = float(settings.marker_length or 0.0) / 2.0
                    obj = np.array(
                        [
                            [-half, half, 0.0],
                            [half, half, 0.0],
                            [half, -half, 0.0],
                            [-half, -half, 0.0],
                        ],
                        dtype=np.float32,
                    )
                    pnp_ok, rvec, tvec = cv.solvePnP(
                        obj,
                        corners[idx][0],
                        K,
                        D,
                        flags=cv.SOLVEPNP_IPPE_SQUARE,
                    )
                    if pnp_ok:
                        T_cam_board = rvec_tvec_to_T(rvec, tvec)
                        cv.drawFrameAxes(display, K, D, rvec, tvec, float(settings.marker_length or 0.08))
        color = (0, 255, 0) if T_cam_board is not None else (0, 0, 255)
        cv.putText(
            display,
            "DETECTED" if T_cam_board is not None else "NOT FOUND",
            (10, 30),
            cv.FONT_HERSHEY_SIMPLEX,
            1.0,
            color,
            2,
        )
        cv.putText(
            display,
            f"samples={n_samples}  {hint}",
            (10, 70),
            cv.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0) if T_cam_board is not None else (0, 200, 255),
            2,
        )
        if K is None:
            cv.putText(
                display,
                "waiting for camera_info",
                (10, 110),
                cv.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )
        return T_cam_board, display


def solve_hand_eye(
    samples: list[dict[str, Any]],
    ee_key: str = "T_world_ee",
) -> tuple[dict[str, Any] | None, list[str]]:
    """Tsai/Park AX=XB. Result T_parent_optical is optical in the same parent as ee_key.

    Camera solve: ee_key=T_world_ee → T_world_optical.
    Right-base solve: ee_key=T_base_ee → T_right_base_optical.
    """
    R_ee2parent, t_ee2parent, R_b2c, t_b2c = [], [], [], []
    R_parent2ee, t_parent2ee = [], []
    for sample in samples:
        if ee_key not in sample:
            return None, [f"sample missing {ee_key}"]
        T_parent_ee = np.array(sample[ee_key], dtype=np.float64)
        T_cam_board = np.array(sample["T_cam_board"], dtype=np.float64)
        T_ee_parent = invert_T(T_parent_ee)
        R_ee2parent.append(np.array(T_parent_ee[:3, :3], copy=True))
        t_ee2parent.append(np.array(T_parent_ee[:3, 3].reshape(3, 1), copy=True))
        R_parent2ee.append(np.array(T_ee_parent[:3, :3], copy=True))
        t_parent2ee.append(np.array(T_ee_parent[:3, 3].reshape(3, 1), copy=True))
        R_b2c.append(np.array(T_cam_board[:3, :3], copy=True))
        t_b2c.append(np.array(T_cam_board[:3, 3].reshape(3, 1), copy=True))

    conventions = {
        "invert": (R_parent2ee, t_parent2ee),
        "direct": (R_ee2parent, t_ee2parent),
    }
    best: dict[str, Any] | None = None
    lines = ["method / convention  flange-board pos std (mm)"]

    def _consider(method_name: str, conv_name: str, T_parent_optical: np.ndarray) -> None:
        nonlocal best
        err = flange_board_consistency_mm(samples, T_parent_optical, ee_key)
        lines.append(f"  {method_name:12s} {conv_name:7s}  {err:.2f}")
        if best is None or err < best["error_mm"]:
            best = {
                "method": method_name,
                "convention": conv_name,
                "T_parent_optical": T_parent_optical,
                "T_world_cam": T_parent_optical,
                "error_mm": err,
            }

    opencv_methods: dict[str, int] = {}
    cv = None
    if cv2 is not None and hasattr(cv2, "calibrateHandEye"):
        cv = cv2
        opencv_methods = hand_eye_methods()

    for conv_name, (Rg, tg) in conventions.items():
        try:
            T_np = park_hand_eye(Rg, tg, R_b2c, t_b2c)
            _consider("PARK_NP", conv_name, T_np)
        except Exception as exc:
            lines.append(f"  {'PARK_NP':12s} {conv_name:7s}  FAIL {exc}")
        if cv is None:
            continue
        for method_name, flag in opencv_methods.items():
            try:
                R_out, t_out = cv.calibrateHandEye(Rg, tg, R_b2c, t_b2c, method=flag)
            except Exception as exc:
                lines.append(f"  {method_name:12s} {conv_name:7s}  FAIL {exc}")
                continue
            T_parent_optical = np.eye(4)
            T_parent_optical[:3, :3] = R_out
            T_parent_optical[:3, 3] = t_out.reshape(3)
            _consider(method_name, conv_name, T_parent_optical)
    return best, lines


def _parse_float_list(raw: str) -> list[float]:
    text = raw.split("#", 1)[0].strip()
    values = ast.literal_eval(text)
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError(f"expected 3-vector, got {raw!r}")
    return [float(v) for v in values]


def load_named_pose(path: Path | str, key: str) -> tuple[np.ndarray, dict[str, Any], dict[str, Any]]:
    """Read xyz/rpy under a top-level YAML key without depending on PyYAML."""
    dest = Path(path)
    text = dest.read_text(encoding="utf-8")
    blocks: dict[str, dict[str, Any]] = {}
    current: str | None = None
    top: dict[str, Any] = {}
    for raw_line in text.splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indented = line.startswith(" ") or line.startswith("\t")
        if not indented:
            current = None
            stripped = line.strip()
            if stripped.endswith(":"):
                current = stripped[:-1].strip()
                blocks.setdefault(current, {})
            continue
        stripped = line.strip()
        if stripped.startswith("xyz:"):
            vec = _parse_float_list(stripped[4:])
            if current:
                blocks[current]["xyz"] = vec
            else:
                top["xyz"] = vec
        elif stripped.startswith("rpy:"):
            vec = _parse_float_list(stripped[4:])
            if current:
                blocks[current]["rpy"] = vec
            else:
                top["rpy"] = vec
        elif stripped.startswith("root_frame:"):
            value = stripped.split(":", 1)[1].strip()
            if current:
                blocks[current]["root_frame"] = value
            else:
                top["root_frame"] = value

    if key in blocks and "xyz" in blocks[key] and "rpy" in blocks[key]:
        block = blocks[key]
    elif key == "camera" and "xyz" in top and "rpy" in top:
        block = top
    else:
        raise KeyError(f"{dest} has no '{key}:'")
    data = {
        name: dict(item)
        for name, item in blocks.items()
        if "xyz" in item and "rpy" in item
    }
    return xyz_rpy_to_T(block["xyz"], block["rpy"]), dict(block), data


def right_base_config_error(cfg: HandeyeRunConfig) -> str | None:
    if cfg.solve_target != SOLVE_TARGET_RIGHT_BASE:
        return None
    if cfg.arm != "right" or "left" in cfg.ee_frame:
        return "右基座求解需要连接右臂通道（can_right），标定板贴在 right_revo2_flange。"
    cam_path = Path(cfg.camera_yaml_path)
    if not cam_path.is_file():
        return f"未找到锁定相机 YAML：{cam_path}"
    try:
        _, cam_block, _ = load_named_pose(cam_path, "camera")
    except Exception as exc:
        return f"无法读取 camera: {cam_path}: {exc}"
    if camera_looks_like_placeholder(cam_block["xyz"]):
        return (
            f"{cam_path} 的 camera: 仍像 URDF 占位。"
            "请先完成左臂眼在手外标定并粘贴 camera:。"
        )
    return None


def _write_outputs(
    output: str | Path,
    samples_output: str | Path,
    yaml_text: str,
    samples: list[dict[str, Any]],
) -> tuple[Path, Path]:
    yaml_path = Path(output)
    samples_path = Path(samples_output)
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    samples_path.parent.mkdir(parents=True, exist_ok=True)
    yaml_path.write_text(yaml_text, encoding="utf-8")
    samples_path.write_text(json.dumps(samples, indent=2), encoding="utf-8")
    return yaml_path, samples_path


def _format_best(
    best: dict[str, Any],
    lines: list[str],
    xyz: list[float],
    rpy: list[float],
    board_xyz: list[float],
    yaml_path: Path,
    samples_path: Path,
) -> str:
    return (
        "\n".join(lines)
        + f"\n\nbest: {best['method']} / {best['convention']}  "
        f"flange-board std={best['error_mm']:.2f} mm  (want < 15 mm)\n"
        f"recovered T_flange_board xyz={[round(v, 4) for v in board_xyz]} "
        f"(do not type this in; it is solved)\n"
        f"wrote {yaml_path}\n"
        f"  xyz={[round(v, 6) for v in xyz]}\n"
        f"  rpy={[round(v, 9) for v in rpy]}\n"
        f"  samples {samples_path}"
    )


def format_camera_yaml(
    xyz: list[float],
    rpy: list[float],
    camera_link_frame: str,
    method: str,
    convention: str,
    error_mm: float,
    n_samples: int,
    board_xyz: list[float],
) -> str:
    return (
        "# Paste into dual_nero_description/config/scene_tf.yaml under camera:\n"
        "camera:\n"
        f"  xyz: {[round(v, 6) for v in xyz]}\n"
        f"  rpy: {[round(v, 9) for v in rpy]}\n"
        f"  root_frame: {camera_link_frame}\n"
        f"# method={method} convention={convention} "
        f"flange_board_std_mm={error_mm:.2f} samples={n_samples}\n"
        f"# recovered T_flange_board xyz (not an input): "
        f"{[round(v, 4) for v in board_xyz]}\n"
    )


def format_right_base_yaml(
    xyz: list[float],
    rpy: list[float],
    method: str,
    convention: str,
    error_mm: float,
    n_samples: int,
    board_xyz: list[float],
    cam_path: Path,
    cam_block: dict[str, Any],
    prev_note: str = "",
) -> str:
    return (
        "# Paste into dual_nero_description/config/scene_tf.yaml under right:\n"
        "# Do NOT change camera: or left: — world is defined by left-arm eye-to-hand.\n"
        "right:\n"
        f"  xyz: {[round(v, 6) for v in xyz]}\n"
        f"  rpy: {[round(v, 9) for v in rpy]}\n"
        f"# method={method} convention={convention} "
        f"flange_board_std_mm={error_mm:.2f} samples={n_samples}\n"
        f"# locked camera from {cam_path}: xyz={list(cam_block['xyz'])} "
        f"rpy={list(cam_block['rpy'])}\n"
        f"# recovered T_flange_board xyz (not an input): "
        f"{[round(v, 4) for v in board_xyz]}\n"
        + prev_note
    )


def write_jpeg_atomic(path: str | Path, bgr: np.ndarray, quality: int = 80) -> None:
    cv = _require_cv2()
    dest = Path(path)
    ok, buf = cv.imencode(".jpg", bgr, [int(cv.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("failed to encode debug jpeg")
    tmp = dest.with_name(dest.name + ".tmp")
    tmp.write_bytes(buf.tobytes())
    tmp.replace(dest)
