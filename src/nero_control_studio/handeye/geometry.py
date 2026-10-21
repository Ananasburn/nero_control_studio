"""Homogeneous transforms and hand-eye consistency checks (numpy only)."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import numpy as np

NDArray = np.ndarray


def invert_T(T: NDArray) -> NDArray:
    Rm = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = Rm.T
    Ti[:3, 3] = -Rm.T @ t
    return Ti


def quat_to_R(x: float, y: float, z: float, w: float) -> NDArray:
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ]
    )


def xyz_quat_to_T(xyz: Sequence[float], quat_xyzw: Sequence[float]) -> NDArray:
    T = np.eye(4)
    T[:3, :3] = quat_to_R(*[float(v) for v in quat_xyzw])
    T[:3, 3] = [float(v) for v in xyz]
    return T


def rot_to_rpy(Rm: NDArray) -> list[float]:
    pitch = float(np.arcsin(np.clip(-Rm[2, 0], -1.0, 1.0)))
    if abs(np.cos(pitch)) > 1e-6:
        roll = float(np.arctan2(Rm[2, 1], Rm[2, 2]))
        yaw = float(np.arctan2(Rm[1, 0], Rm[0, 0]))
    else:
        roll = float(np.arctan2(-Rm[1, 2], Rm[1, 1]))
        yaw = 0.0
    return [roll, pitch, yaw]


def rpy_to_R(rpy: Sequence[float]) -> NDArray:
    roll, pitch, yaw = [float(v) for v in rpy]
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def xyz_rpy_to_T(xyz: Sequence[float], rpy: Sequence[float]) -> NDArray:
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(rpy)
    T[:3, 3] = [float(v) for v in xyz]
    return T


def rotation_error_deg(R_a: NDArray, R_b: NDArray) -> float:
    c = np.clip((np.trace(R_a.T @ R_b) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(c)))


# URDF / scene_tf placeholder; a real left-arm solve is far from this.
CAMERA_PLACEHOLDER_XYZ = np.array([0.30, 0.0, 0.80])


def se3_delta(T_a: NDArray, T_b: NDArray) -> tuple[NDArray, float]:
    """How T_b differs from T_a: translation mm and rotation deg."""
    dt_mm = (T_b[:3, 3] - T_a[:3, 3]) * 1000.0
    return dt_mm, rotation_error_deg(T_a[:3, :3], T_b[:3, :3])


def parent_ee_key(solve_target: str) -> str:
    if solve_target == "right_base":
        return "T_base_ee"
    return "T_world_ee"


def camera_looks_like_placeholder(xyz: Sequence[float]) -> bool:
    p = np.asarray(xyz, dtype=np.float64).reshape(3)
    return float(np.linalg.norm(p - CAMERA_PLACEHOLDER_XYZ)) < 0.05


def compose_T_world_base(T_world_optical: NDArray, T_base_optical: NDArray) -> NDArray:
    """Lock camera, recover T_world_right_base from T_right_base_optical."""
    return T_world_optical @ invert_T(T_base_optical)


def flange_board_consistency_mm(
    samples: list[dict[str, Any]],
    T_parent_optical: NDArray,
    ee_key: str = "T_world_ee",
) -> float:
    """Std of T_flange_board translation across poses; should be constant."""
    positions = []
    for sample in samples:
        T_parent_ee = np.array(sample[ee_key], dtype=np.float64)
        T_cam_board = np.array(sample["T_cam_board"], dtype=np.float64)
        T_ee_board = invert_T(T_parent_ee) @ T_parent_optical @ T_cam_board
        positions.append(T_ee_board[:3, 3])
    pos = np.array(positions)
    return float(np.linalg.norm(np.std(pos, axis=0) * 1000.0))


def mean_T_ee_board(
    samples: list[dict[str, Any]],
    T_parent_optical: NDArray,
    ee_key: str = "T_world_ee",
) -> NDArray:
    acc = np.zeros((4, 4))
    for sample in samples:
        T_parent_ee = np.array(sample[ee_key], dtype=np.float64)
        T_cam_board = np.array(sample["T_cam_board"], dtype=np.float64)
        acc += invert_T(T_parent_ee) @ T_parent_optical @ T_cam_board
    return acc / len(samples)


def _rotation_log(Rm: NDArray) -> NDArray:
    angle = float(np.arccos(np.clip((np.trace(Rm) - 1.0) / 2.0, -1.0, 1.0)))
    if angle < 1e-12:
        return np.zeros(3)
    axis = np.array([Rm[2, 1] - Rm[1, 2], Rm[0, 2] - Rm[2, 0], Rm[1, 0] - Rm[0, 1]])
    return (angle / (2.0 * np.sin(angle))) * axis


def _poses_from_rt(Rs: list[NDArray], ts: list[NDArray]) -> list[NDArray]:
    poses = []
    for Rm, t in zip(Rs, ts):
        T = np.eye(4)
        T[:3, :3] = np.asarray(Rm, dtype=np.float64)
        T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
        poses.append(T)
    return poses


def park_hand_eye(R_g2b: list[NDArray], t_g2b: list[NDArray], R_t2c: list[NDArray], t_t2c: list[NDArray]) -> NDArray:
    """Park & Martin AX=XB (same pairing as OpenCV CALIB_HAND_EYE_PARK)."""
    Hg = _poses_from_rt(R_g2b, t_g2b)
    Hc = _poses_from_rt(R_t2c, t_t2c)
    if len(Hg) < 3:
        raise ValueError("need at least 3 poses")
    M = np.zeros((3, 3))
    pairs: list[tuple[NDArray, NDArray]] = []
    for i in range(len(Hg)):
        for j in range(i + 1, len(Hg)):
            A = invert_T(Hg[j]) @ Hg[i]
            B = Hc[j] @ invert_T(Hc[i])
            pairs.append((A, B))
            a = _rotation_log(A[:3, :3]).reshape(3, 1)
            b = _rotation_log(B[:3, :3]).reshape(3, 1)
            M += b @ a.T
    eigvals, eigvecs = np.linalg.eigh(M.T @ M)
    order = np.argsort(eigvals)[::-1]
    eigvals = np.maximum(eigvals[order], 1e-15)
    eigvecs = eigvecs[:, order]
    v = np.diag(1.0 / np.sqrt(eigvals))
    Rx = eigvecs @ v @ eigvecs.T @ M.T
    u, _, vt = np.linalg.svd(Rx)
    Rx = u @ np.diag([1.0, 1.0, np.linalg.det(u @ vt)]) @ vt
    rows = []
    rhs = []
    for A, B in pairs:
        rows.append(np.eye(3) - A[:3, :3])
        rhs.append(A[:3, 3] - Rx @ B[:3, 3])
    tx, *_ = np.linalg.lstsq(np.vstack(rows), np.concatenate(rhs), rcond=None)
    T = np.eye(4)
    T[:3, :3] = Rx
    T[:3, 3] = tx.reshape(3)
    return T
