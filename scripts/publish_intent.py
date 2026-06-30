#!/usr/bin/env python3
"""Publish a single ARISE intent on /intents to drive the mission controller.

Reviewer helper for the full-ARISE bag demo (orchestration/configs/rises_demo.yaml).
The mission controller subscribes to the GLOBAL /intents topic and routes
START_ACTIVITY intents by a substring in the `data` field: "lock_area",
"unlock_area" or "update_map". `area_id` is parsed as the first integer after
"area_id" (or "goal") in the JSON, so we emit that exact shape.

Run it INSIDE the agv container (hri_actions_msgs is available there), e.g.:

    docker exec -it agv_0 python3 /workspace/scripts/publish_intent.py lock_area --area 1

If the repo is not mounted into the container, copy this file in first
(`docker cp scripts/publish_intent.py agv_0:/tmp/`) and run it from /tmp.
"""
import argparse
import json
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from hri_actions_msgs.msg import Intent


GOALS = ("lock_area", "unlock_area", "update_map")


def build_intent(goal: str, area_id: int) -> Intent:
    msg = Intent()
    # Intent.intent is a string field; use the message constant, never a literal.
    msg.intent = Intent.START_ACTIVITY
    payload = {"goal": goal}
    if goal in ("lock_area", "unlock_area"):
        payload["area_id"] = area_id
    msg.data = json.dumps(payload)
    msg.source = "reviewer"
    msg.modality = "cli"
    msg.confidence = 1.0
    msg.priority = 0
    return msg


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("goal", choices=GOALS, help="mission to trigger")
    parser.add_argument("--area", type=int, default=1,
                        help="area_id for (un)lock_area (default: 1)")
    parser.add_argument("--topic", default="/intents",
                        help="intent topic (default: global /intents)")
    args = parser.parse_args()

    rclpy.init()
    node = Node("reviewer_intent_publisher")
    # Mission controller subscribes reliable, depth 10.
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(Intent, args.topic, qos)

    msg = build_intent(args.goal, args.area)

    # Wait briefly for the subscription to be matched so the message is not dropped.
    deadline = node.get_clock().now().nanoseconds + 5_000_000_000
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds > deadline:
            node.get_logger().warn(
                f"No subscriber on {args.topic} after 5s; publishing anyway "
                "(is rises_mission_controller running?).")
            break

    pub.publish(msg)
    node.get_logger().info(
        f"Published intent={msg.intent} data={msg.data} on {args.topic}")
    # Give the middleware a moment to flush before shutdown.
    rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
