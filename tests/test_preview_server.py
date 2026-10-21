from __future__ import annotations

import socket
import urllib.error
import urllib.request
from pathlib import Path

from nero_control_studio.handeye.preview_server import (
    DebugPreviewServer,
    browse_endpoints,
)

_OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def _open(url: str):
    return _OPENER.open(url, timeout=2)


def test_browse_endpoints_puts_localhost_first() -> None:
    items = browse_endpoints(8765, advertise_lan=False)
    assert items == [("本机", "http://127.0.0.1:8765/")]

    advertised = browse_endpoints(8765, advertise_lan=True)
    assert advertised[0] == ("本机", "http://127.0.0.1:8765/")
    if len(advertised) == 2:
        label, url = advertised[1]
        assert label == "局域网"
        assert url.startswith("http://")
        assert url.endswith(":8765/")
        assert "127.0.0.1" not in url


def test_preview_serves_page_and_jpeg(tmp_path: Path) -> None:
    path = tmp_path / "debug.jpg"
    payload = b"\xff\xd8preview-jpeg\xff\xd9"
    path.write_bytes(payload)
    server = DebugPreviewServer(str(path), host="127.0.0.1", port=0)
    endpoints = server.start()
    try:
        base = endpoints[0][1]
        assert base.startswith("http://127.0.0.1:")
        with _open(base) as resp:
            page = resp.read().decode("utf-8")
        assert "/stream" in page
        assert "检测画面" in page
        with _open(base + "image.jpg") as resp:
            assert resp.read() == payload

        other = tmp_path / "next.jpg"
        other_payload = b"\xff\xd8next-jpeg\xff\xd9"
        other.write_bytes(other_payload)
        server.set_image_path(str(other))
        with _open(base + "image.jpg") as resp:
            assert resp.read() == other_payload
    finally:
        server.stop()
        server.stop()


def test_preview_image_missing_is_404(tmp_path: Path) -> None:
    server = DebugPreviewServer(str(tmp_path / "missing.jpg"), host="127.0.0.1", port=0)
    endpoints = server.start()
    try:
        with _open(endpoints[0][1] + "image.jpg") as resp:
            raise AssertionError(f"expected 404, got {resp.status}")
    except urllib.error.HTTPError as exc:
        assert exc.code == 404
    finally:
        server.stop()


def test_preview_stream_sends_current_jpeg(tmp_path: Path) -> None:
    path = tmp_path / "debug.jpg"
    payload = b"\xff\xd8stream-jpeg\xff\xd9"
    path.write_bytes(payload)
    server = DebugPreviewServer(str(path), host="127.0.0.1", port=0)
    endpoints = server.start()
    try:
        url = endpoints[0][1]
        port = int(url.rsplit(":", 1)[1].rstrip("/"))
        sock = socket.create_connection(("127.0.0.1", port), timeout=2)
        sock.sendall(b"GET /stream HTTP/1.0\r\nHost: 127.0.0.1\r\n\r\n")
        sock.settimeout(2)
        blob = b""
        while payload not in blob and len(blob) < 8192:
            chunk = sock.recv(256)
            assert chunk
            blob += chunk
        assert b"multipart/x-mixed-replace" in blob
        assert payload in blob
    finally:
        server.stop()
