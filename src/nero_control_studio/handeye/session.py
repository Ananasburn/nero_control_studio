"""QProcess manager for the system-Python ROS hand-eye sidecar."""

from __future__ import annotations

import os
import shlex
from typing import Optional

from PySide6.QtCore import QObject, QProcess, QProcessEnvironment, Signal, Slot

from nero_control_studio.handeye.config import (
    HandeyeRunConfig,
    SOLVE_TARGET_RIGHT_BASE,
    bridge_script_path,
    nero_ws,
    package_src_root,
    ros_setup_path,
    system_python,
)
from nero_control_studio.handeye.calibrator import right_base_config_error
from nero_control_studio.handeye.protocol import encode_cmd, parse_message


def environment_error() -> str | None:
    if not ros_setup_path().is_file():
        return f"未找到 ROS setup：{ros_setup_path()}"
    if not system_python().is_file():
        return f"未找到系统 Python：{system_python()}"
    script = bridge_script_path()
    if not script.is_file():
        return f"未找到 sidecar 脚本：{script}"
    setup = nero_ws() / "install" / "setup.bash"
    if not setup.is_file():
        return (
            f"未找到 {setup}。请设置环境变量 NERO_WS 指向已 colcon 安装 "
            "dual_nero_description 的 ROS 工作空间。"
        )
    return None


def _launch_shell(cfg: HandeyeRunConfig) -> str:
    parts: list[str] = [
        str(system_python()),
        str(bridge_script_path()),
        "--arm",
        cfg.arm,
        "--ee-frame",
        cfg.ee_frame,
        "--pattern",
        cfg.pattern,
        "--cols",
        str(cfg.cols),
        "--rows",
        str(cfg.rows),
        "--square-size",
        str(cfg.square_size),
        "--dict",
        cfg.aruco_dict,
        "--marker-id",
        str(cfg.marker_id),
        "--image-topic",
        cfg.image_topic,
        "--camera-info-topic",
        cfg.camera_info_topic,
        "--world-frame",
        cfg.world_frame,
        "--camera-link-frame",
        cfg.camera_link_frame,
        "--optical-frame",
        cfg.optical_frame,
        "--debug-image",
        cfg.debug_image_path,
        "--nero-ws",
        str(nero_ws()),
        "--solve-target",
        cfg.solve_target,
        "--base-frame",
        cfg.base_frame,
    ]
    if cfg.pattern == "aruco":
        parts.extend(["--marker-length", str(cfg.marker_length)])
    if cfg.start_camera:
        parts.append("--start-camera")
    if cfg.solve_target == SOLVE_TARGET_RIGHT_BASE and cfg.camera_yaml_path:
        parts.extend(["--camera-yaml", cfg.camera_yaml_path])
    quoted = " ".join(shlex.quote(p) for p in parts)
    ros = shlex.quote(str(ros_setup_path()))
    ws = shlex.quote(str(nero_ws() / "install" / "setup.bash"))
    src = shlex.quote(str(package_src_root()))
    return (
        "set +u; "
        "unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_PYTHON_EXE CONDA_EXE; "
        "unset ROS_LOCALHOST_ONLY; "
        'export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"; '
        f"source {ros}; "
        f"source {ws}; "
        f'export PYTHONPATH={src}${{PYTHONPATH:+:$PYTHONPATH}}; '
        f"exec {quoted}"
    )


class HandeyeSession(QObject):
    log_message = Signal(str)
    status_received = Signal(dict)
    ready = Signal(str)
    failed = Signal(str)
    sample_finished = Signal(bool, str)
    drop_finished = Signal(bool, str)
    solve_finished = Signal(bool, str)
    running_changed = Signal(bool)

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._proc: Optional[QProcess] = None
        self._stdout_buf = ""
        self._ready = False
        self._stopping = False
        self._announced_fail = False
        self._debug_image = ""

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.state() != QProcess.ProcessState.NotRunning

    @property
    def debug_image_path(self) -> str:
        return self._debug_image

    def start(self, cfg: HandeyeRunConfig) -> None:
        err = environment_error()
        if err:
            self.failed.emit(err)
            return
        if cfg.pattern == "aruco" and not cfg.marker_length:
            self.failed.emit("ArUco 需要填写 marker 边长（米）")
            return
        right_err = right_base_config_error(cfg)
        if right_err:
            self.failed.emit(right_err)
            return
        if self.running:
            self.log_message.emit("手眼标定已在运行")
            return
        self._ready = False
        self._stopping = False
        self._announced_fail = False
        self._stdout_buf = ""
        self._debug_image = cfg.debug_image_path
        proc = QProcess(self)
        env = QProcessEnvironment.systemEnvironment()
        for key in (
            "CONDA_PREFIX",
            "CONDA_DEFAULT_ENV",
            "CONDA_SHLVL",
            "CONDA_PYTHON_EXE",
            "CONDA_EXE",
            "ROS_LOCALHOST_ONLY",
        ):
            env.remove(key)
        env.insert("ROS_AUTOMATIC_DISCOVERY_RANGE", os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET"))
        proc.setProcessEnvironment(env)
        proc.setProcessChannelMode(QProcess.ProcessChannelMode.SeparateChannels)
        proc.readyReadStandardOutput.connect(self._on_stdout)
        proc.readyReadStandardError.connect(self._on_stderr)
        proc.finished.connect(self._on_finished)
        proc.errorOccurred.connect(self._on_error)
        proc.setProgram("bash")
        proc.setArguments(["-lc", _launch_shell(cfg)])
        self._proc = proc
        self.log_message.emit("正在启动手眼标定 sidecar…")
        proc.start()
        self.running_changed.emit(True)

    def stop(self) -> None:
        if not self.running:
            return
        self._stopping = True
        self._write(encode_cmd("stop"))
        assert self._proc is not None
        if not self._proc.waitForFinished(2500):
            self._proc.terminate()
            if not self._proc.waitForFinished(1500):
                self._proc.kill()
                self._proc.waitForFinished(1000)

    def send_joints(self, arm: str, q: list[float]) -> None:
        if not self.running:
            return
        self._write(encode_cmd("joints", arm=arm, q=[float(v) for v in q]))

    def sample(self) -> None:
        self._write(encode_cmd("sample"))

    def drop(self) -> None:
        self._write(encode_cmd("drop"))

    def solve(self, yaml_path: str, samples_path: str) -> None:
        self._write(encode_cmd("solve", output=yaml_path, samples_output=samples_path))

    def request_status(self) -> None:
        self._write(encode_cmd("status"))

    def _write(self, payload: str) -> None:
        if self._proc is None or self._proc.state() == QProcess.ProcessState.NotRunning:
            return
        self._proc.write(payload.encode("utf-8"))

    @Slot()
    def _on_stdout(self) -> None:
        if self._proc is None:
            return
        chunk = bytes(self._proc.readAllStandardOutput()).decode("utf-8", errors="replace")
        self._stdout_buf += chunk
        while "\n" in self._stdout_buf:
            line, self._stdout_buf = self._stdout_buf.split("\n", 1)
            self._handle_line(line)

    def _handle_line(self, line: str) -> None:
        try:
            msg = parse_message(line)
        except Exception:
            text = line.strip()
            if text:
                self.log_message.emit(text)
            return
        if msg is None:
            return
        event = str(msg.get("event", ""))
        if event == "log":
            self.log_message.emit(str(msg.get("message", "")))
        elif event == "status":
            self.status_received.emit(msg)
        elif event == "ready":
            self._ready = True
            path = str(msg.get("debug_image") or self._debug_image)
            self._debug_image = path
            self.log_message.emit("手眼标定已就绪，检测画面已打开")
            self.ready.emit(path)
        elif event == "error":
            text = str(msg.get("message", "未知错误"))
            self.log_message.emit(text)
            if not self._ready:
                self._announced_fail = True
                self.failed.emit(text)
        elif event == "sample_result":
            self.sample_finished.emit(bool(msg.get("ok")), str(msg.get("message", "")))
        elif event == "drop_result":
            self.drop_finished.emit(bool(msg.get("ok")), str(msg.get("message", "")))
        elif event == "solve_result":
            self.solve_finished.emit(bool(msg.get("ok")), str(msg.get("message", "")))

    @Slot()
    def _on_stderr(self) -> None:
        if self._proc is None:
            return
        text = bytes(self._proc.readAllStandardError()).decode("utf-8", errors="replace").rstrip()
        if text:
            for line in text.splitlines():
                self.log_message.emit(line)

    @Slot(int, QProcess.ExitStatus)
    def _on_finished(self, code: int, _status: QProcess.ExitStatus) -> None:
        self.running_changed.emit(False)
        if not self._stopping and not self._ready and code != 0 and not self._announced_fail:
            self.failed.emit(f"手眼 sidecar 退出（code={code}）")
        elif not self._stopping:
            self.log_message.emit(f"手眼 sidecar 已结束（code={code}）")
        self._proc = None
        self._ready = False
        self._stopping = False

    @Slot(QProcess.ProcessError)
    def _on_error(self, error: QProcess.ProcessError) -> None:
        if error == QProcess.ProcessError.FailedToStart:
            self._announced_fail = True
            self.failed.emit("无法启动 bash / 系统 Python sidecar")
            self.running_changed.emit(False)
            self._proc = None
