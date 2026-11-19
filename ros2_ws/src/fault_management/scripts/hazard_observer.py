#!/usr/bin/env python3
"""Score obstacle visibility from camera maps; truth never enters navigation."""

import json
import math
from collections import deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from hazard_assessment import assess_hazard
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from simulation_support.obstacle_scenarios import load_scenario
from std_msgs.msg import String
from uav_ugv_msgs.msg import PlannedRoute


def stamp(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


class HazardObserver(Node):
    def __init__(self):
        super().__init__("hazard_observer")
        self.scenario = load_scenario(Path(self.startup_parameter("scenario_file", "").value))
        self.radius = self.startup_parameter("robot_radius", 0.5).value
        self.clearance = self.startup_parameter("obstacle_clearance", 0.05).value
        self.max_speed = self.startup_parameter("maximum_speed", 0.5).value
        self.response_s = self.startup_parameter("response_bound", 2.0).value
        if any(
            not math.isfinite(v) or v <= 0
            for v in (self.radius, self.clearance, self.max_speed, self.response_s)
        ):
            raise ValueError("hazard evaluation limits must be finite and positive")
        output = Path(self.startup_parameter("output_directory", "").value)
        output.mkdir(parents=True, exist_ok=True)
        self.log = (output / "observations.jsonl").open("x")
        self.output = output
        self.state = None
        self.last_clock = None
        self.error = None
        self.grids = deque(maxlen=20)
        self.routes = deque(maxlen=50)
        self.truth = deque(maxlen=100)
        self.pending = {}
        self.observed = {}
        self.retired = {}
        self.event_sequence = 0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.event_pub = self.create_publisher(String, "/evaluation/hazards", 50)
        self.create_subscription(
            String,
            "/simulation/obstacle_scenario",
            self.on_state,
            QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(OccupancyGrid, "/local_grid", self.on_grid, latched)
        self.create_subscription(PlannedRoute, "/ugv/navigation/route", self.on_route, latched)
        self.create_subscription(
            PoseStamped, "/evaluation/ugv_truth", self.on_truth, qos_profile_sensor_data
        )
        from rclpy.clock import Clock, ClockType

        self.create_timer(0.1, self.check_clock, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def startup_parameter(self, name, default):
        return self.declare_parameter(name, default, descriptor=ParameterDescriptor(read_only=True))

    def emit(self, stream, time_ns, data, sequence, valid=True):
        value = dict(
            schema_version=1,
            stream=stream,
            time_ns=time_ns,
            clock="sim",
            epoch=0,
            source="hazard_observer",
            sequence=sequence,
            valid=valid,
            frame="map",
            data=data,
        )
        self.event_pub.publish(String(data=json.dumps(value)))

    def on_state(self, message):
        value = json.loads(message.data)
        if value["schema_version"] != 1 or value["scenario_id"] != self.scenario.id:
            self.error = "scenario identity mismatch"
            return
        self.state = value
        active_operations = {v["operation"]["id"] for v in value["applied"]}
        removing = (
            value["pending"]
            if value["pending"] and value["pending"]["action"] == "remove"
            else None
        )
        for operation_id, case in list(self.pending.items()):
            if operation_id not in active_operations or (
                removing and removing["obstacle"] == case["applied"]["operation"]["obstacle"]
            ):
                self.retired[operation_id] = "removed_before_eligible_observation"
                self.pending.pop(operation_id)
        for applied in value["applied"]:
            operation_id = applied["operation"]["id"]
            if removing and removing["obstacle"] == applied["operation"]["obstacle"]:
                self.retired[operation_id] = "removed_before_eligible_observation"
                continue
            if (
                operation_id in self.pending
                or operation_id in self.observed
                or operation_id in self.retired
            ):
                continue
            # The obstacle must have been on a fresh pre-mutation planned route.
            request = applied["requested_ns"]
            baseline = next((g for g in reversed(self.grids) if g["time_ns"] < request), None)
            route = next((r for r in reversed(self.routes) if r["time_ns"] < request), None)
            self.pending[operation_id] = dict(applied=applied, baseline=baseline, route=route)
        if self.grids:
            self.assess(self.grids[-1])

    def on_route(self, message):
        if message.header.frame_id != "map":
            return
        self.routes.append(
            dict(
                time_ns=stamp(message),
                goal_revision=str(stamp(message.goal)),
                points=[[p.pose.position.x, p.pose.position.y] for p in message.path.poses],
            )
        )

    def on_truth(self, message):
        if message.header.frame_id == "map":
            self.truth.append((stamp(message), message.pose.position.x, message.pose.position.y))

    def on_grid(self, message):
        now = self.get_clock().now().nanoseconds
        q = message.info.origin.orientation
        if (
            message.header.frame_id != "map"
            or stamp(message) <= 0
            or not 0 <= now - stamp(message) <= 500_000_000
            or abs(q.x) + abs(q.y) + abs(q.z) > 1e-6
            or abs(abs(q.w) - 1) > 1e-6
        ):
            return
        grid = dict(
            time_ns=stamp(message),
            width=message.info.width,
            height=message.info.height,
            resolution=message.info.resolution,
            origin=[message.info.origin.position.x, message.info.origin.position.y],
            data=message.data,
        )
        if self.grids and grid["time_ns"] <= self.grids[-1]["time_ns"]:
            return
        self.grids.append(grid)
        self.assess(grid)

    def assess(self, grid):
        if self.error:
            return
        for operation_id, case in list(self.pending.items()):
            applied, baseline, route = case["applied"], case["baseline"], case["route"]
            truth = next((v for v in reversed(self.truth) if v[0] <= grid["time_ns"]), None)
            current = next(
                (r for r in reversed(self.routes) if r["time_ns"] <= grid["time_ns"]), None
            )
            data = assess_hazard(
                applied,
                baseline,
                route,
                grid,
                truth,
                current["goal_revision"] if current else None,
                radius=self.radius,
                clearance_margin=self.clearance,
                maximum_speed=self.max_speed,
                response_s=self.response_s,
            )
            if data is None:
                continue
            self.emit("events", grid["time_ns"], data, self.event_sequence)
            self.event_sequence += 1
            self.log.write(json.dumps(data) + "\n")
            self.log.flush()
            self.observed[operation_id] = data
            self.pending.pop(operation_id)
            self.get_logger().info(
                f"Observed {operation_id}: qualifying={data['qualifying']}, "
                f"cells={data['occupied_cells']}"
            )

    def check_clock(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock is not None and now < self.last_clock:
            self.error = "hazard observer clock rewound"
        self.last_clock = now

    def close(self):
        self.log.close()
        (self.output / "observation_result.json").write_text(
            json.dumps(
                dict(
                    scenario_id=self.scenario.id,
                    error=self.error,
                    observations=list(self.observed.values()),
                    retired_operations=self.retired,
                    unobserved_operations=list(self.pending),
                ),
                indent=2,
            )
            + "\n"
        )


def main():
    rclpy.init()
    node = None
    try:
        node = HazardObserver()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
