"""Embedded detection-image viewer (JPEG file written by the sidecar)."""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import QLabel, QSizePolicy


class DebugImageView(QLabel):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._path = ""
        self._mtime_ns = 0
        self._source = QPixmap()
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumHeight(280)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setObjectName("debugImageView")
        self.setText("启动标定后将在此显示检测画面")

    def set_image_path(self, path: str) -> None:
        self._path = path
        self._mtime_ns = 0

    def clear_image(self) -> None:
        self._path = ""
        self._mtime_ns = 0
        self._source = QPixmap()
        self.setPixmap(QPixmap())
        self.setText("启动标定后将在此显示检测画面")

    def refresh(self) -> None:
        if not self._path:
            return
        path = Path(self._path)
        if not path.is_file():
            return
        try:
            mtime = path.stat().st_mtime_ns
        except OSError:
            return
        if mtime == self._mtime_ns and not self._source.isNull():
            self._rescale()
            return
        pix = QPixmap(str(path))
        if pix.isNull():
            return
        self._mtime_ns = mtime
        self._source = pix
        self._rescale()

    def resizeEvent(self, event) -> None:  # type: ignore[override]
        super().resizeEvent(event)
        if hasattr(self, "_source") and not self._source.isNull():
            self._rescale()

    def _rescale(self) -> None:
        pix = getattr(self, "_source", QPixmap())
        if pix.isNull():
            return
        scaled = pix.scaled(
            self.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.setPixmap(scaled)
        self.setText("")
