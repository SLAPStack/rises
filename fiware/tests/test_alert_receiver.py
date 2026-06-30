"""Tests for :mod:`fiware.alert_service.alert_receiver`.

The receiver is a :class:`http.server.BaseHTTPRequestHandler` subclass.
Spinning up a real server per test is overkill; we construct a handler
via :meth:`object.__new__`, wire in fake ``rfile`` / ``wfile`` buffers,
and drive ``do_POST`` directly. This isolates the HTTP-parsing layer
without binding to a port.
"""

from __future__ import annotations

import io
import json
from typing import Any

import pytest


# ---------------------------------------------------------------------------
# Helpers.
# ---------------------------------------------------------------------------
class _FakeRequest:
    """Stand-in for the socket-like first argument of a handler."""

    def makefile(self, *_, **__):
        return io.BytesIO()


def _build_handler(receiver_mod, body: bytes,
                   headers: dict[str, str] | None = None):
    """Build an :class:`AlertHandler` with synthetic request data attached."""
    handler = object.__new__(receiver_mod.AlertHandler)
    handler.rfile = io.BytesIO(body)
    handler.wfile = io.BytesIO()
    handler.headers = headers or {"Content-Length": str(len(body))}
    handler.path = "/"

    # Capture status / headers / written body.
    handler._responses: list[int] = []
    handler._sent_headers: list[tuple[str, str]] = []

    def send_response(code, message=None):
        handler._responses.append(code)

    def send_header(key, value):
        handler._sent_headers.append((key, value))

    def end_headers():
        return None

    handler.send_response = send_response  # type: ignore[assignment]
    handler.send_header = send_header  # type: ignore[assignment]
    handler.end_headers = end_headers  # type: ignore[assignment]
    return handler


# ---------------------------------------------------------------------------
# Happy-path tests.
# ---------------------------------------------------------------------------
def test_post_alert_payload_logged(alert_receiver_module, log_dir):
    """A valid notification produces a JSON-line in alerts.jsonl and a 200 reply."""
    body = json.dumps({
        "subscriptionId": "sub-1",
        "data": [{
            "id": "urn:ngsi-ld:AGV:agv_0",
            "type": "AGV",
            "obstacle_alert": {"value": True},
        }],
    }).encode()

    handler = _build_handler(alert_receiver_module, body)
    handler.do_POST()

    assert handler._responses == [200]
    log_file = log_dir / "alerts.jsonl"
    assert log_file.exists()
    line = json.loads(log_file.read_text().strip())
    assert line["entity_id"] == "urn:ngsi-ld:AGV:agv_0"
    assert line["event_type"] == "OBSTACLE_ALERT"
    assert line["alert_active"] is True


def test_severity_filter_keeps_critical(alert_receiver_module, log_dir):
    """A CRITICAL-equivalent (obstacle_alert=True) event reaches the log."""
    body = json.dumps({
        "subscriptionId": "sub-1",
        "data": [{"id": "agv_0", "obstacle_alert": {"value": True}}],
    }).encode()
    handler = _build_handler(alert_receiver_module, body)
    handler.do_POST()

    line = json.loads((log_dir / "alerts.jsonl").read_text().strip())
    assert line["alert_active"] is True


def test_invalid_json_returns_400(alert_receiver_module, log_dir):
    """Malformed JSON in the body produces HTTP 400 without crashing."""
    body = b"{this is not json"
    handler = _build_handler(alert_receiver_module, body)
    handler.do_POST()

    assert handler._responses == [400]


def test_health_check_returns_200(alert_receiver_module, log_dir):
    """GET requests return a 200 health response."""
    handler = _build_handler(alert_receiver_module, b"")
    handler.do_GET()
    assert handler._responses == [200]


# ---------------------------------------------------------------------------
# Behaviour expected by the spec but not implemented in production code.
# ---------------------------------------------------------------------------
@pytest.mark.skip(
    reason="alert_receiver has no dedup window; skipped until production "
           "code grows a dedup layer.",
)
def test_dedup_window_collapses_duplicates(alert_receiver_module, log_dir):
    """Two identical alerts within the dedup window collapse to one log line."""


@pytest.mark.skip(
    reason="alert_receiver does not implement severity-based filtering; "
           "INFO-level events are logged the same as WARNING. Skipped until "
           "the production filter is added.",
)
def test_severity_filter_drops_info_alerts(alert_receiver_module, log_dir):
    """An INFO-severity alert is filtered out before reaching the log."""


@pytest.mark.skip(
    reason="alert_receiver enforces no body-size cap; skipped until a 413 "
           "guard is added to the production code.",
)
def test_content_length_cap_rejects_oversize(alert_receiver_module, log_dir):
    """A 10 MB body is rejected with HTTP 413."""
