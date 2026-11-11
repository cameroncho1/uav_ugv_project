#!/usr/bin/env python3
"""Wait for actual ROS observations with a bounded wall clock."""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=40.0)
    parser.add_argument("topics", nargs="+", help="TOPIC=ROS_MESSAGE_TYPE")
    args = parser.parse_args()
    rclpy.init()
    node = Node("verification_readiness")
    observed = {}
    subscriptions = []
    started = time.monotonic()
    try:
        for spec in args.topics:
            topic, kind = spec.split("=", 1)
            subscriptions.append(
                node.create_subscription(
                    get_message(kind),
                    topic,
                    lambda _message, name=topic: observed.setdefault(name, time.monotonic()),
                    qos_profile_sensor_data,
                )
            )
        while len(observed) < len(subscriptions) and time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(
            json.dumps(
                {
                    "ready": len(observed) == len(subscriptions),
                    "duration_wall_s": time.monotonic() - started,
                    "observed": sorted(observed),
                }
            )
        )
        return 0 if len(observed) == len(subscriptions) else 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
