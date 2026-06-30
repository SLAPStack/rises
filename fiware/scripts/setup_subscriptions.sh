#!/bin/bash
# Sets up NGSI-LD subscriptions on Orion-LD for automated alert generation.
# Run this after Orion-LD is healthy: docker compose up -d && sleep 10 && ./scripts/setup_subscriptions.sh

ORION_HOST="${ORION_HOST:-localhost}"
ORION_PORT="${ORION_PORT:-1026}"
ALERT_SERVICE="${ALERT_SERVICE:-http://localhost:9090/notify}"
BASE_URL="http://${ORION_HOST}:${ORION_PORT}/ngsi-ld/v1/subscriptions"

echo "Setting up NGSI-LD subscriptions on ${ORION_HOST}:${ORION_PORT}"
echo "Alert notifications will be sent to: ${ALERT_SERVICE}"

# Subscription 1: Obstacle alert state changes
echo ""
echo "Creating subscription: obstacle_alert changes..."
curl -s -o /dev/null -w "  HTTP %{http_code}\n" -X POST "${BASE_URL}" \
  -H "Content-Type: application/json" \
  -d '{
    "id": "urn:ngsi-ld:Subscription:obstacle_alert",
    "type": "Subscription",
    "description": "Notify when any AGV obstacle alert state changes",
    "entities": [{"type": "AGV"}],
    "watchedAttributes": ["obstacle_alert"],
    "notification": {
      "attributes": ["obstacle_alert", "obstacle_report"],
      "endpoint": {
        "uri": "'"${ALERT_SERVICE}"'",
        "accept": "application/json"
      }
    }
  }'

# Subscription 2: Geofence ready state changes
echo "Creating subscription: geofence_ready changes..."
curl -s -o /dev/null -w "  HTTP %{http_code}\n" -X POST "${BASE_URL}" \
  -H "Content-Type: application/json" \
  -d '{
    "id": "urn:ngsi-ld:Subscription:geofence_ready",
    "type": "Subscription",
    "description": "Notify when any AGV geofence node changes readiness",
    "entities": [{"type": "AGV"}],
    "watchedAttributes": ["geofence_ready"],
    "notification": {
      "attributes": ["geofence_ready"],
      "endpoint": {
        "uri": "'"${ALERT_SERVICE}"'",
        "accept": "application/json"
      }
    }
  }'

# Subscription 3: Mission status changes
echo "Creating subscription: mission_status changes..."
curl -s -o /dev/null -w "  HTTP %{http_code}\n" -X POST "${BASE_URL}" \
  -H "Content-Type: application/json" \
  -d '{
    "id": "urn:ngsi-ld:Subscription:mission_status",
    "type": "Subscription",
    "description": "Notify when mission controller status changes",
    "entities": [{"type": "MissionController"}],
    "watchedAttributes": ["mission_status"],
    "notification": {
      "attributes": ["mission_status"],
      "endpoint": {
        "uri": "'"${ALERT_SERVICE}"'",
        "accept": "application/json"
      }
    }
  }'

echo ""
echo "Verifying subscriptions..."
curl -s "http://${ORION_HOST}:${ORION_PORT}/ngsi-ld/v1/subscriptions" | python3 -m json.tool 2>/dev/null || \
  curl -s "http://${ORION_HOST}:${ORION_PORT}/ngsi-ld/v1/subscriptions"

echo ""
echo "Done. Subscriptions are active."
