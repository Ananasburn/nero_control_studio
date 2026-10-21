"""Browser preview of the hand-eye debug JPEG.

The sidecar keeps writing one JPEG for the pendant. This server exposes that
same file so a browser can show it larger than the embedded view.
"""

from __future__ import annotations

import errno
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

PREFERRED_PREVIEW_PORT = 8765
_PORT_SCAN = 20

_PAGE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Nero 检测画面</title>
<style>
  html, body { margin: 0; height: 100%; background: #111; color: #d0d4dc;
    font-family: sans-serif; }
  body { display: flex; flex-direction: column; }
  img { flex: 1; width: 100%; min-height: 0; object-fit: contain; background: #000; }
  p { margin: 10px 16px; font-size: 14px; line-height: 1.45; }
</style>
</head>
<body>
<img src="/stream" alt="检测画面">
<p>与控制台「检测画面」是同一张图。启动标定后这里会显示角点和坐标轴，可以把浏览器窗口拉大或全屏查看。</p>
</body>
</html>
"""


class DebugPreviewServer:
    """Serve ``/`` and a live JPEG stream for the current debug image."""

    def __init__(
        self,
        image_path: str = "",
        host: str = "0.0.0.0",
        port: int | None = None,
    ) -> None:
        self._path = image_path
        self._host = host
        self._preferred_port = PREFERRED_PREVIEW_PORT if port is None else port
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._httpd: _PreviewHTTPServer | None = None
        self._thread: threading.Thread | None = None
        self._port = 0

    @property
    def port(self) -> int:
        return self._port

    @property
    def stopping(self) -> bool:
        return self._stop.is_set()

    def set_image_path(self, path: str) -> None:
        with self._lock:
            self._path = path

    def read_jpeg(self) -> tuple[bytes, int]:
        with self._lock:
            path = self._path
        if not path:
            return b"", 0
        file = Path(path)
        try:
            mtime = file.stat().st_mtime_ns
            data = file.read_bytes()
        except OSError:
            return b"", 0
        if len(data) < 4 or not data.startswith(b"\xff\xd8"):
            return b"", 0
        return data, mtime

    def wait_stopped(self, timeout: float) -> bool:
        return self._stop.wait(timeout)

    def endpoints(self) -> list[tuple[str, str]]:
        if self._port <= 0:
            return []
        return browse_endpoints(self._port, advertise_lan=self._host not in {"127.0.0.1", "localhost"})

    def start(self) -> list[tuple[str, str]]:
        if self._httpd is not None:
            return self.endpoints()
        self._stop.clear()
        httpd = _bind_server(self._host, self._preferred_port)
        httpd.owner = self
        self._httpd = httpd
        self._port = int(httpd.server_address[1])
        thread = threading.Thread(
            target=httpd.serve_forever,
            name="handeye-preview",
            daemon=True,
        )
        thread.start()
        self._thread = thread
        return self.endpoints()

    def stop(self) -> None:
        self._stop.set()
        httpd = self._httpd
        thread = self._thread
        self._httpd = None
        self._thread = None
        self._port = 0
        if httpd is not None:
            httpd.shutdown()
            httpd.server_close()
        if thread is not None:
            thread.join(timeout=2.0)


def browse_endpoints(port: int, *, advertise_lan: bool) -> list[tuple[str, str]]:
    """Labels and URLs a person can open. Localhost is always first."""
    items = [("本机", f"http://127.0.0.1:{port}/")]
    if advertise_lan:
        lan = _primary_lan_ipv4()
        if lan:
            items.append(("局域网", f"http://{lan}:{port}/"))
    return items


def _primary_lan_ipv4() -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        ip = sock.getsockname()[0]
    except OSError:
        return ""
    finally:
        sock.close()
    if not ip or ip.startswith("127."):
        return ""
    return ip


def _bind_server(host: str, preferred: int) -> _PreviewHTTPServer:
    if preferred == 0:
        return _PreviewHTTPServer((host, 0), _PreviewHandler)
    last_error: OSError | None = None
    for port in range(preferred, preferred + _PORT_SCAN):
        try:
            return _PreviewHTTPServer((host, port), _PreviewHandler)
        except OSError as exc:
            last_error = exc
            if exc.errno not in (errno.EADDRINUSE, errno.EACCES):
                raise
    try:
        return _PreviewHTTPServer((host, 0), _PreviewHandler)
    except OSError:
        if last_error is not None:
            raise last_error
        raise


class _PreviewHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    owner: DebugPreviewServer


class _PreviewHandler(BaseHTTPRequestHandler):
    server: _PreviewHTTPServer

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            self._send(200, "text/html; charset=utf-8", _PAGE.encode("utf-8"))
            return
        if path == "/image.jpg":
            data, _mtime = self.server.owner.read_jpeg()
            if not data:
                self._send(404, "text/plain; charset=utf-8", "检测画面尚未开始".encode())
                return
            self._send(200, "image/jpeg", data, cache="no-store")
            return
        if path == "/stream":
            self._stream()
            return
        self._send(404, "text/plain; charset=utf-8", b"not found")

    def log_message(self, fmt: str, *args) -> None:
        return

    def _send(self, code: int, content_type: str, body: bytes, cache: str = "") -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        if cache:
            self.send_header("Cache-Control", cache)
        self.end_headers()
        self.wfile.write(body)

    def _stream(self) -> None:
        try:
            self.send_response(200)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last_mtime = -1
            while not self.server.owner.stopping:
                data, mtime = self.server.owner.read_jpeg()
                if data and mtime != last_mtime:
                    last_mtime = mtime
                    packet = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Content-Length: " + str(len(data)).encode("ascii") + b"\r\n\r\n"
                        + data
                        + b"\r\n"
                    )
                    self.wfile.write(packet)
                if self.server.owner.wait_stopped(0.08):
                    break
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            return
