"""
Lightweight NGSI-LD notification receiver for ARISE/RISES.

Receives webhook notifications from Orion-LD subscriptions and:
  1. Logs all events to stdout with structured formatting
  2. Logs to a JSON-lines file for historical analysis
  3. Can optionally forward to external systems (Slack, email, etc.)

Usage:
  python3 alert_receiver.py                         # default port 9090
  python3 alert_receiver.py --port 9090 --log-dir /var/log/rises

This service is intentionally minimal. For production, replace with a proper
event processing system (Node-RED, Apache NiFi, custom microservice).
"""

import argparse
import json
import logging
import os
import sys
from datetime import datetime, timezone
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path


LOG_DIR = Path("logs")
logger = logging.getLogger("alert_receiver")


class AlertHandler(BaseHTTPRequestHandler):
    """Handles POST notifications from Orion-LD subscriptions."""

    def do_POST(self):
        content_length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_length)

        try:
            notification = json.loads(body)
        except json.JSONDecodeError:
            logger.error("Invalid JSON in notification body: %s", body[:200])
            self.send_response(400)
            self.end_headers()
            return

        self._process_notification(notification)

        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(b'{"status": "ok"}')

    def do_GET(self):
        """Health check endpoint."""
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(b'{"status": "healthy"}')

    def _process_notification(self, notification):
        """Process an NGSI-LD notification from Orion-LD."""
        timestamp = datetime.now(timezone.utc).isoformat()
        subscription_id = notification.get("subscriptionId", "unknown")
        data = notification.get("data", [])

        for entity in data:
            entity_id = entity.get("id", "unknown")
            entity_type = entity.get("type", "unknown")

            event = {
                "timestamp": timestamp,
                "subscription_id": subscription_id,
                "entity_id": entity_id,
                "entity_type": entity_type,
            }

            # Extract relevant attributes based on entity type
            if "obstacle_alert" in entity:
                alert_val = entity["obstacle_alert"]
                is_alert = self._extract_value(alert_val)
                event["event_type"] = "OBSTACLE_ALERT"
                event["alert_active"] = is_alert
                severity = "WARNING" if is_alert else "INFO"
                logger.log(
                    logging.WARNING if is_alert else logging.INFO,
                    "[%s] %s obstacle_alert=%s",
                    entity_id, severity, is_alert,
                )

            if "geofence_ready" in entity:
                ready_val = entity["geofence_ready"]
                is_ready = self._extract_value(ready_val)
                event["event_type"] = "GEOFENCE_READY"
                event["ready"] = is_ready
                logger.info("[%s] geofence_ready=%s", entity_id, is_ready)

            if "mission_status" in entity:
                status_val = entity["mission_status"]
                status = self._extract_value(status_val)
                event["event_type"] = "MISSION_STATUS"
                event["status"] = status
                logger.info("[%s] mission_status=%s", entity_id, status)

            self._write_log(event)

    @staticmethod
    def _extract_value(attr):
        """Extract the value from an NGSI-LD attribute (handles both normalized and simplified)."""
        if isinstance(attr, dict):
            return attr.get("value", attr)
        return attr

    @staticmethod
    def _write_log(event):
        """Append event as JSON line to the log file."""
        log_file = LOG_DIR / "alerts.jsonl"
        with open(log_file, "a") as f:
            f.write(json.dumps(event) + "\n")

    def log_message(self, format, *args):
        """Suppress default HTTP request logging (we log our own way)."""
        pass


def main():
    parser = argparse.ArgumentParser(description="NGSI-LD alert notification receiver")
    parser.add_argument("--port", type=int, default=9090, help="Listen port (default: 9090)")
    parser.add_argument("--log-dir", type=str, default="logs", help="Directory for JSON log files")
    args = parser.parse_args()

    global LOG_DIR
    LOG_DIR = Path(args.log_dir)
    LOG_DIR.mkdir(parents=True, exist_ok=True)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        stream=sys.stdout,
    )

    server = HTTPServer(("0.0.0.0", args.port), AlertHandler)
    logger.info("Alert receiver listening on port %d, logging to %s/", args.port, LOG_DIR)
    logger.info("Health check: GET http://localhost:%d/", args.port)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logger.info("Shutting down alert receiver")
        server.server_close()


if __name__ == "__main__":
    main()
