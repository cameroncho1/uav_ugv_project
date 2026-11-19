#!/usr/bin/env python3
"""Exercise the real ROS controller with declared pose/route/detector test inputs.

This fixture checks command gating and protocol behavior, not detector accuracy or
physical stopping. Guard/watchdog and camera trials provide separate evidence.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import subprocess
import time
from contextlib import ExitStack
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from uav_ugv_msgs.msg import FaultStatus, PlannedRoute
from verification_experiments import process_groups, stop
from verification_recording import Recording

SECOND = 1_000_000_000


def stamp(value):
    from builtin_interfaces.msg import Time

    return Time(sec=value // SECOND, nanosec=value % SECOND)


class SupervisorTrial(Node):
    def __init__(self, output):
        super().__init__("navigation_supervisor_trial")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/ugv/localization/pose", 10
        )
        self.goal_pub = self.create_publisher(PoseStamped, "/global_goal_pose", latched)
        self.route_pub = self.create_publisher(
            PlannedRoute, "/ugv/navigation/route", latched
        )
        self.fault_pub = self.create_publisher(
            FaultStatus, "/fault_detection/status", 20
        )
        self.create_subscription(String, "/evaluation/control", self.control, 100)
        self.commands = []
        self.create_subscription(Twist, "/ugv/cmd_vel", self.command, 100)
        self.trace = (output / "control.jsonl").open("w")
        self.requests = (output / "fault_requests.jsonl").open("w")
        self.sim_ns = 10 * SECOND
        self.revision = self.sim_ns
        self.goal_x = 3.0
        self.pose_x = 0.0
        self.route_obsolete = False
        self.pose_valid = True
        self.advance_clock = True
        self.sequence = 0
        self.session = "w10-controlled-detector"
        self.heartbeat = None
        self.next_heartbeat = 0
        self.duplicate = None
        self.observations = []
        self.cases = []

    def control(self, message):
        value = json.loads(message.data)
        self.observations.append(value)
        self.trace.write(json.dumps(value, sort_keys=True) + "\n")
        self.trace.flush()

    def command(self, message):
        self.commands.append(
            (time.monotonic(), abs(message.linear.x) + abs(message.angular.z))
        )

    def pump(self):
        started = time.monotonic()
        if self.advance_clock:
            self.sim_ns += SECOND // 20
        self.clock_pub.publish(Clock(clock=stamp(self.sim_ns)))
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = stamp(self.revision)
        goal.pose.position.x = self.goal_x
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map" if self.pose_valid else "odom"
        pose.header.stamp = stamp(self.sim_ns)
        pose.pose.pose.position.x = self.pose_x
        pose.pose.pose.orientation.w = 1.0
        pose.pose.covariance[0] = pose.pose.covariance[7] = 0.04
        pose.pose.covariance[35] = 0.01
        self.pose_pub.publish(pose)
        route = PlannedRoute()
        route.header = copy.deepcopy(pose.header)
        route.header.frame_id = "map"
        route.goal = copy.deepcopy(goal)
        if self.route_obsolete:
            route.goal.header.stamp = stamp(max(1, self.revision - SECOND))
        route.path.header = copy.deepcopy(route.header)
        for x in (0.0, 0.6, self.goal_x):
            point = PoseStamped()
            point.header = copy.deepcopy(route.header)
            point.pose.position.x = x
            point.pose.orientation.w = 1.0
            route.path.poses.append(point)
        grid = OccupancyGrid()
        grid.header = copy.deepcopy(route.header)
        grid.info.resolution = 0.25
        grid.info.width, grid.info.height = 40, 16
        grid.info.origin.position.x = grid.info.origin.position.y = -2.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 640
        route.clearance_grid = grid
        self.route_pub.publish(route)
        if self.duplicate is not None:
            self.fault_pub.publish(self.duplicate)
        elif self.heartbeat and self.sim_ns >= self.next_heartbeat:
            self.send_fault(self.heartbeat)
            self.next_heartbeat = self.sim_ns + SECOND // 2
        rclpy.spin_once(self, timeout_sec=0.005)
        # Drain ready telemetry; command callbacks must not delay state acknowledgments.
        for _ in range(4):
            rclpy.spin_once(self, timeout_sec=0)
        time.sleep(max(0, 0.05 - (time.monotonic() - started)))

    def drive(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.pump()

    def wait(self, predicate, timeout=3):
        start = len(self.observations)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            for value in self.observations[start:]:
                if predicate(value):
                    return value
        latest = self.observations[-1] if self.observations else {}
        raise AssertionError(
            f"Controller observation timed out: {latest}; "
            f"graph={self.get_node_names_and_namespaces()}; "
            f"domain={self.context.get_domain_id()}"
        )

    def case(self, name, mode, *, moving=False, predicate=lambda _: True):
        observed = self.wait(lambda v: v["supervisor_state"] == mode and predicate(v))
        mark = len(self.observations)
        self.drive(0.35)
        recent = self.observations[mark:]
        assert recent, f"{name}: no subsequent command telemetry"
        assert all(v["supervisor_state"] == mode for v in recent), (name, recent)
        magnitudes = [abs(v["linear"]) + abs(v["angular"]) for v in recent]
        physical_messages = [v for t, v in self.commands if t > time.monotonic() - 0.2]
        assert physical_messages, f"{name}: no actual Twist publications"
        if moving:
            assert max(magnitudes) > 0.05 and max(physical_messages) > 0.05, name
        else:
            assert max(magnitudes) == 0 and max(physical_messages) == 0, name
        self.cases.append(
            dict(
                case=name,
                expected_mode=mode,
                passed=True,
                observation=observed,
                subsequent_ticks=len(recent),
            )
        )

    def send_fault(self, kind="clear", stamp_ns=None):
        message = FaultStatus()
        message.header.stamp = stamp(self.sim_ns if stamp_ns is None else stamp_ns)
        message.schema_version = 1
        message.source_id = self.session
        message.sequence = self.sequence
        self.sequence += 1
        message.window_valid = kind != "invalid"
        message.odometry_scale = kind == "alarm"
        self.fault_pub.publish(message)
        self.requests.write(
            json.dumps(
                dict(
                    kind=kind,
                    stamp_ns=message.header.stamp.sec * SECOND
                    + message.header.stamp.nanosec,
                    sequence=message.sequence,
                    source=self.session,
                )
            )
            + "\n"
        )
        self.requests.flush()
        return message

    def recover(self, name, expected="NAVIGATE"):
        self.heartbeat, self.duplicate = None, None
        alarm = self.send_fault("alarm")
        self.wait(
            lambda v: v["fault_sequence"] == str(alarm.sequence) and v["fault_latched"]
        )
        first = self.sim_ns
        for index in range(10):
            if index:
                while self.sim_ns < first + index * SECOND // 2:
                    self.pump()
            message = self.send_fault()
            self.wait(
                lambda v, sequence=message.sequence, count=index + 1: (
                    v["fault_sequence"] == str(sequence)
                    and v["clear_windows"] == count
                ),
                timeout=0.3,
            )
        value = self.wait(
            lambda v: v["clear_windows"] == 10 and v["fault_latched"], timeout=0.3
        )
        assert value["sim_ns"] - first < 5 * SECOND, (
            "ten-window observation arrived after maturity"
        )
        self.cases.append(
            dict(
                case=name + "_ten_windows_before_five_seconds",
                passed=True,
                observation=value,
                first_clear_ns=first,
            )
        )
        while self.sim_ns < first + 5 * SECOND:
            self.pump()
        self.heartbeat = "clear"
        self.next_heartbeat = self.sim_ns
        self.case(name, expected, moving=expected == "NAVIGATE")

    def new_goal(self, x):
        self.goal_x = x
        self.revision = self.sim_ns

    def readonly_parameters(self):
        client = self.create_client(
            SetParameters, "/ugv_goal_controller/set_parameters"
        )
        assert client.wait_for_service(timeout_sec=5)
        request = SetParameters.Request(
            parameters=[
                Parameter("detector_enabled", value=False).to_parameter_msg(),
                Parameter("fault_status_topic", value="/unused").to_parameter_msg(),
            ]
        )
        future = client.call_async(request)
        deadline = time.monotonic() + 5
        while not future.done() and time.monotonic() < deadline:
            self.pump()
        assert future.done() and all(not r.successful for r in future.result().results)
        self.cases.append(dict(case="startup_configuration_is_readonly", passed=True))

    def run(self):
        # Require actual controller observations before starting the cases.
        self.wait(
            lambda v: v["have_goal"] and v["guard_valid"] and v["route_valid"],
            timeout=20,
        )
        self.wait(
            lambda _: any(
                i.node_name == "ugv_goal_controller"
                for i in self.get_subscriptions_info_by_topic("/fault_detection/status")
            ),
            timeout=20,
        )
        self.case("missing_enabled_detector", "HOLD_FAULT")
        self.readonly_parameters()
        self.recover("initial_recovery")
        self.heartbeat = "alarm"
        self.next_heartbeat = self.sim_ns
        self.case("critical_alarm", "HOLD_FAULT")
        self.new_goal(4.0)
        self.case("new_goal_cannot_clear_fault", "HOLD_FAULT")
        self.heartbeat = None
        self.duplicate = self.send_fault()
        self.wait(lambda v: v["clear_windows"] == 1)
        self.drive(0.4)
        assert self.observations[-1]["clear_windows"] == 1
        self.cases.append(dict(case="duplicate_windows_do_not_count", passed=True))
        self.drive(1.1)
        self.case(
            "duplicates_do_not_renew_freshness",
            "HOLD_FAULT",
            predicate=lambda v: v["clear_windows"] == 0,
        )
        self.duplicate = None
        self.recover("recovery_after_stale_input")
        self.heartbeat = None
        self.send_fault("invalid")
        self.case("invalid_window_holds", "HOLD_FAULT")
        self.recover("recovery_after_invalid_window")
        self.pose_x = self.goal_x
        self.case("estimated_goal_latches", "GOAL")
        self.pose_x = 0.0
        self.case("same_goal_drift_cannot_resume", "GOAL")
        self.heartbeat = "alarm"
        self.next_heartbeat = self.sim_ns
        self.case("fault_after_completed_goal", "HOLD_FAULT")
        self.recover("completion_survives_fault_recovery", "GOAL")
        self.pose_valid = False
        self.case("completion_survives_guard_failure", "HOLD_GUARD")
        self.pose_valid = True
        self.case("guard_recovery_cannot_restart_completed_goal", "GOAL")
        self.new_goal(5.0)
        self.case("new_revision_authorizes_new_mission", "NAVIGATE", moving=True)
        self.route_obsolete = True
        self.case("old_goal_route_rejected", "HOLD_GUARD")
        self.route_obsolete = False
        self.case("matching_route_recovers", "NAVIGATE", moving=True)
        self.heartbeat = None
        # Deliver an alarm/clear burst before the next periodic command decision.
        self.send_fault("alarm")
        self.send_fault(stamp_ns=self.sim_ns + 1)
        self.case(
            "alarm_clear_burst_does_not_bypass_latch",
            "HOLD_FAULT",
            predicate=lambda v: v["clear_windows"] == 1,
        )
        self.advance_clock = False
        self.drive(1.2)
        self.case("paused_clock_cannot_complete_recovery", "HOLD_FAULT")
        self.advance_clock = True
        self.sim_ns = 2 * SECOND
        self.session = "w10-controlled-detector-new-epoch"
        self.sequence = 0
        self.case("rewind_rejects_pre_epoch_goal", "INIT")
        self.new_goal(5.0)
        self.case("new_epoch_still_requires_fault_recovery", "HOLD_FAULT")
        self.recover("new_epoch_recovery")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    os.environ.setdefault(
        "FASTRTPS_DEFAULT_PROFILES_FILE", str(root / "config/fastdds_fixture_udp.xml")
    )
    status, error, node = "aborted", None, None
    started = time.monotonic()
    try:
        with ExitStack() as cleanup:
            rclpy.init()
            cleanup.callback(rclpy.try_shutdown)
            node = SupervisorTrial(args.output)
            cleanup.callback(node.destroy_node)
            cleanup.callback(node.trace.close)
            cleanup.callback(node.requests.close)
            binary = (
                Path(get_package_prefix("ugv_control"))
                / "lib/ugv_control/goal_controller"
            )
            log = cleanup.enter_context((args.output / "controller.log").open("w"))
            recording = Recording(
                args.output / "verification", extra_topics=("/fault_detection/status",)
            )
            cleanup.callback(lambda: recording.close(status))
            process = subprocess.Popen(
                [
                    str(binary),
                    "--ros-args",
                    "-p",
                    "use_sim_time:=true",
                    "-p",
                    "evaluation_trace:=true",
                    "-p",
                    "detector_enabled:=true",
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            cleanup.callback(lambda: stop(process, process_groups(process.pid)))
            node.run()
            status = "completed"
    except BaseException as exc:
        status, error = "aborted", f"{type(exc).__name__}: {exc}"
        raise
    finally:
        result = dict(
            kind="controlled_ROS_supervisor_fixture",
            status=status,
            error=error,
            detector_accuracy_evaluated=False,
            physical_stopping_evaluated=False,
            ros_domain_id=os.environ.get("ROS_DOMAIN_ID", "0"),
            transport_profile=os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"),
            duration_wall_s=time.monotonic() - started,
            cases=node.cases if node else [],
        )
        (args.output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(dict(status=status, cases=len(node.cases))))


if __name__ == "__main__":
    main()
