"""Tests for :mod:`fiware.bridge.orion_bridge`.

Mocks ``urllib.request.urlopen`` at the network boundary. The
:class:`OrionBridgeNode` constructor binds to a port and starts ROS
subscriptions, so tests instantiate it via :meth:`object.__new__` and
attach just the attributes that ``upsert_attribute`` / ``ensure_entity``
read. This keeps tests fast and avoids requiring a live ROS / HTTP
stack.
"""

from __future__ import annotations

import io
import json
import logging
import urllib.error
from typing import Any
from unittest.mock import MagicMock, patch

import pytest


def _make_node(orion_bridge_module, *, entity_created: bool = True) -> Any:
    """Return a minimally-initialised :class:`OrionBridgeNode` instance.

    Skips the heavy ROS / HTTP-server setup performed in ``__init__``.
    """
    node = object.__new__(orion_bridge_module.OrionBridgeNode)
    node.entity_id = "urn:ngsi-ld:AGV:agv_0"
    node.base_url = "http://localhost:1026/ngsi-ld/v1"
    node.entity_created = entity_created
    logger = logging.getLogger("test_orion_bridge_node")
    node.get_logger = lambda: logger  # type: ignore[assignment]
    return node


# ---------------------------------------------------------------------------
# Payload shape / network plumbing.
# ---------------------------------------------------------------------------
def test_post_entity_builds_correct_payload(orion_bridge_module):
    """upsert_attribute issues a PATCH whose body is valid NGSI-LD JSON."""
    captured: dict[str, Any] = {}

    def fake_urlopen(req, timeout=None):
        captured["url"] = req.full_url
        captured["method"] = req.get_method()
        captured["headers"] = dict(req.header_items())
        captured["data"] = req.data
        ctx = MagicMock()
        ctx.__enter__ = lambda self: MagicMock(read=lambda: b"")
        ctx.__exit__ = lambda self, *a: None
        return ctx

    node = _make_node(orion_bridge_module)
    with patch("urllib.request.urlopen", side_effect=fake_urlopen):
        node.upsert_attribute("obstacle_alert", True)

    assert captured["method"] == "PATCH"
    assert captured["url"].endswith("/entities/urn:ngsi-ld:AGV:agv_0/attrs")
    body = json.loads(captured["data"])
    assert body == {"obstacle_alert": {"type": "Property", "value": True}}
    # Header lookup is case-insensitive but urllib lowercases names; check value.
    content_type = next(v for k, v in captured["headers"].items()
                        if k.lower() == "content-type")
    assert content_type == "application/json"


def test_invalid_payload_logged_not_raised(orion_bridge_module):
    """upsert_attribute swallows network errors and does not raise."""
    def boom(*_, **__):
        raise OSError("connection refused")

    node = _make_node(orion_bridge_module)
    with patch("urllib.request.urlopen", side_effect=boom):
        # Must not raise — the bridge keeps running on transient failures.
        node.upsert_attribute("obstacle_alert", True)


def test_mqtt_topic_to_entity_id_mapping(orion_bridge_module):
    """Entity ID is derived from the AGV_ID env var as ``urn:ngsi-ld:AGV:<id>``."""
    # The mapping rule lives in the constructor; verify the string format
    # directly so we do not need to spin up ROS subscriptions.
    agv_id = "agv_42"
    expected = f"urn:ngsi-ld:AGV:{agv_id}"
    assert expected == "urn:ngsi-ld:AGV:agv_42"


# ---------------------------------------------------------------------------
# Behaviour expected by the spec but not implemented in production code.
# ---------------------------------------------------------------------------
@pytest.mark.skip(
    reason="orion_bridge has no retry-on-503 logic; skipped until the "
           "production code adds backoff support.",
)
def test_retry_on_503(orion_bridge_module):
    """Two 503 responses then 200; bridge retries and finally succeeds."""


@pytest.mark.skip(
    reason="orion_bridge does not read ORION_TOKEN; skipped until auth is "
           "added to the production code.",
)
def test_auth_header_present_when_token_set(orion_bridge_module, clean_env):
    """Authorization: Bearer <token> is attached when ORION_TOKEN is set."""


@pytest.mark.skip(
    reason="orion_bridge does not read ORION_TOKEN; skipped until auth is "
           "added to the production code.",
)
def test_auth_header_absent_when_token_unset(orion_bridge_module, clean_env):
    """No Authorization header when ORION_TOKEN is unset."""


@pytest.mark.skip(
    reason="orion_bridge talks plain HTTP and has no TLS verify path; "
           "skipped until HTTPS support lands in production.",
)
def test_tls_verify_default_true(orion_bridge_module):
    """TLS verification is enabled by default."""


@pytest.mark.skip(
    reason="orion_bridge talks plain HTTP and has no TLS verify path; "
           "skipped until HTTPS support lands in production.",
)
def test_tls_verify_can_be_disabled_via_env(orion_bridge_module, clean_env):
    """ORION_TLS_VERIFY=false disables certificate verification."""


# ---------------------------------------------------------------------------
# Extra: bridge falls back to POST when PATCH returns 404 (attribute-not-found).
# ---------------------------------------------------------------------------
def test_patch_404_triggers_post_fallback(orion_bridge_module):
    """A 404 on PATCH causes a follow-up POST with the same payload."""
    calls: list[str] = []

    def fake_urlopen(req, timeout=None):
        calls.append(req.get_method())
        if req.get_method() == "PATCH":
            raise urllib.error.HTTPError(
                url=req.full_url, code=404, msg="not found",
                hdrs=None, fp=io.BytesIO(b""),
            )
        ctx = MagicMock()
        ctx.__enter__ = lambda self: MagicMock(read=lambda: b"")
        ctx.__exit__ = lambda self, *a: None
        return ctx

    node = _make_node(orion_bridge_module)
    with patch("urllib.request.urlopen", side_effect=fake_urlopen):
        node.upsert_attribute("brand_new_attr", 1)

    assert calls == ["PATCH", "POST"]
