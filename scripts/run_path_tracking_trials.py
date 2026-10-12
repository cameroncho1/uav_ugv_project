#!/usr/bin/env python3
"""Compare closed-loop baseline/assisted tracking on the declared free-grid fixture."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock

ROOT = Path(__file__).resolve().parents[1]


class NavigationFixture(Node):
    def __init__(self):
        super().__init__("tracking_map_goal_fixture")
        self.revision = None
        self.last_publish = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid = self.create_publisher(OccupancyGrid, "/local_grid", qos)
        self.goal = self.create_publisher(PoseStamped, "/global_goal_pose", qos)
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)

    def on_clock(self, message):
        stamp = message.clock.sec + message.clock.nanosec * 1e-9
        if stamp <= 0 or (self.last_publish is not None and stamp - self.last_publish < .1):
            return
        self.last_publish = stamp
        self.revision = self.revision or message.clock
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.revision
        goal.pose.position.x = 4.0
        goal.pose.orientation.w = 1.0
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = message.clock
        grid.info.width = grid.info.height = 80
        grid.info.resolution = .25
        grid.info.origin.position.x = grid.info.origin.position.y = -5.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 6400
        self.goal.publish(goal)
        self.grid.publish(grid)


def cleanup(processes):
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        for process in processes:
            try: os.killpg(process.pid, sig)
            except ProcessLookupError: pass
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and any(p.poll() is None for p in processes):
            time.sleep(.05)
    for process in processes:
        if process.poll() is not None: process.wait()


def run_case(output, seed, strategy, timeout):
    directory = output / (str(seed) + "_" + strategy)
    directory.mkdir(parents=True, exist_ok=True)
    os.environ["GZ_PARTITION"] = "tracking_" + str(os.getpid()) + "_" + str(seed) + "_" + strategy
    os.environ["GZ_IP"] = "127.0.0.1"
    processes, logs = [], []

    def launch(label, arguments):
        log = (directory / (label + ".log")).open("w")
        logs.append(log)
        process = subprocess.Popen(arguments, cwd=ROOT, env=os.environ.copy(),
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    rclpy.init()
    fixture = NavigationFixture()
    topic = "/evaluation/baseline_pose" if strategy == "baseline" else "/ugv/localization/pose"
    try:
        launch("world", ["bash", str(ROOT / "scripts/world.sh")])
        launch("evaluation", ["ros2", "launch", "fault_management",
            "ugv_cooperative_evaluation.launch.py", "seed:=" + str(seed), "measurement_profile:=drift"])
        launch("command_bridge", ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
            "-p", "config_file:=" + str(ROOT / "ros2_ws/src/mission_manager/config/ugv_command_bridge.yaml")])
        for package, executable in (("route_planning", "local_astar_planner"),
                                    ("ugv_control", "goal_controller")):
            launch(package, ["ros2", "run", package, executable, "--ros-args", "-p", "use_sim_time:=true",
                             "-r", "/ugv/localization/pose:=" + topic])
        scorer = launch("score", ["python3", str(ROOT / "scripts/score_goal_navigation.py"),
            str(directory / "result.json"), "--seed", str(seed), "--timeout", str(timeout),
            "--estimate-topic", topic, "--relative-source", "simulated", "--initial-gps-enabled",
            "--gps-outage-after-motion", "2.0"])
        deadline = time.monotonic() + timeout + 20
        while scorer.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(fixture, timeout_sec=.1)
        result_path = directory / "result.json"
        assert result_path.exists(), "Tracking scorer did not produce bounded evidence"
        result = json.loads(result_path.read_text())
        assert result["path_following"]["samples"] >= 50, "Insufficient matched route/truth samples"
        assert result["gps_disable_confirmed"] is True, "GPS outage was not applied"
        return {"seed": seed, "strategy": strategy, "arrival_passed": result["passed"],
                "physical_goal_distance_m": result["true_goal_distance_m"],
                **result["path_following"]}
    finally:
        cleanup(processes)
        fixture.destroy_node()
        rclpy.try_shutdown()
        for log in logs: log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seeds", nargs="+", type=int, default=[513, 514, 515])
    parser.add_argument("--timeout", type=float, default=120.)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rows = [run_case(args.output, seed, strategy, args.timeout)
            for seed in args.seeds for strategy in ("baseline", "assisted")]
    conditions = {"world": "uav_ugv", "map": "declared free 20x20 m grid, 0.25 m cells",
                  "start_map_xy": [1., 0.], "goal_map_xy": [4., 0.],
                  "measurement_profile": "drift", "minimum_gps_outage_s_after_first_motion": 2.,
                  "minimum_received_gps_fixes_before_outage": 11,
                  "relative_source": "fixed-observer simulated", "results": rows,
                  "comparison": "Fresh worlds with identical configured seeds/map/goal; trajectories and sensor sequences differ under closed-loop control",
                  "limits": "Flat, obstacle-free tracking fixture; no camera-map accuracy or hardware claims"}
    (args.output / "summary.json").write_text(json.dumps(conditions, indent=2) + "\n")
    print(json.dumps(rows), flush=True)


if __name__ == "__main__":
    main()
