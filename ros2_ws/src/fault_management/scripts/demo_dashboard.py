#!/usr/bin/env python3
"""Live demo dashboard: serves a web page of the running estimators and sensor controls.

The page shows simulator truth next to the cooperative and the GPS/wheel/IMU-only
estimates, plus the navigation state and detector flags. Truth is used for display and
scoring only. Controls call the existing evaluation services and set the live
parameters of `sim_ugv_sensors`; a click on the map sets the goal through the live
parameters of `goal_manager`, and the block button adds a box on the route through the
simulation_support Gazebo adapter. Nothing here publishes measurements or commands.
"""

import json
import math
import queue
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool
from uav_ugv_msgs.msg import FaultStatus, PlannedRoute

from dashboard_core import (ErrorStats, block_site, classify_layers, encode_layers,
                            goal_error, position_error, stddev_from_covariance,
                            validate_control, world_geometry)
from localization_common.geodesy import yaw_from_quaternion

MAX_BLOCKS = 8
BLOCK_SIZE = (0.6, 0.6, 0.4)
NOMINAL = dict(gps_enabled=True, uav_assistance=True, noise_scale=1.0,
               gps_horizontal_stddev=1.5, wheel_speed_scale=1.0, imu_yaw_bias_rate=0.0)
PARAMETER_CONTROLS = ("noise_scale", "gps_horizontal_stddev", "wheel_speed_scale",
                      "imu_yaw_bias_rate")
SAMPLE_PERIOD_S = 0.25
ROUTE_POINTS = 80


class Dashboard(Node):
    def __init__(self):
        super().__init__("demo_dashboard")
        self.declare_parameter("world_file", "")
        self.declare_parameter("port", 8765)
        self.declare_parameter("host", "127.0.0.1")
        world_file = self.get_parameter("world_file").value
        self.world = world_geometry(world_file) if world_file else dict(
            obstacles=[], goal=None, start=None)

        self.lock = threading.Lock()
        self.commands = queue.Queue()
        self.controls = dict(NOMINAL)
        self.state = dict(truth=None, fused=None, baseline=None, gps=None, uav=None, route=[], goal=None,
                          partial=False, mode=None, reason="", flags={}, hazards=[], speed=0.0)
        self.trail = dict(truth=[], fused=[], baseline=[])
        self.series = []  # (t, fused error, baseline error)
        self.stats = dict(fused=ErrorStats(), baseline=ErrorStats())
        self.last_sample = None
        self.start_time = None
        self.block_lock = threading.Lock()
        self.blocks = []        # Box objects added from the page, newest last
        self.block_count = 0
        self.gz_world = None
        self.recording = False  # trails and errors start with the first drive, not the startup transient
        self.grid = None      # latest /local_grid message
        self.safety = None    # latest inflated grid from the planner
        self.layers = None
        self.layers_seq = 0
        self.layers_dirty = False

        best_effort = qos_profile_sensor_data
        self.create_subscription(PoseStamped, "/evaluation/ugv_truth", self.on_truth, best_effort)
        self.create_subscription(PoseWithCovarianceStamped, "/ugv/localization/pose",
                                 lambda m: self.on_pose("fused", m), best_effort)
        self.create_subscription(PoseWithCovarianceStamped, "/evaluation/baseline_pose",
                                 lambda m: self.on_pose("baseline", m), best_effort)
        self.create_subscription(PoseWithCovarianceStamped, "/ugv/measurements/gps_pose",
                                 self.on_gps, best_effort)
        self.create_subscription(PoseWithCovarianceStamped, "/uav/localization/pose",
                                 self.on_uav, best_effort)
        self.create_subscription(PlannedRoute, "/ugv/navigation/route", self.on_route, best_effort)
        self.create_subscription(FaultStatus, "/fault_detection/status", self.on_faults, 20)
        self.create_subscription(
            OccupancyGrid, "/local_grid", self.on_grid,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, "/evaluation/control", self.on_control, best_effort)
        self.create_subscription(
            PoseStamped, "/global_goal_pose", self.on_goal,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            String, "/simulation/obstacle_scenario", self.on_scenario,
            QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))

        self.service_clients = {
            "gps_enabled": self.create_client(SetBool, "/evaluation/set_gps_enabled"),
            "uav_assistance": self.create_client(SetBool, "/ugv/localization/set_uav_enabled"),
        }
        self.parameter_client = self.create_client(SetParameters, "/sim_ugv_sensors/set_parameters")
        self.goal_client = self.create_client(SetParameters, "/goal_manager/set_parameters")
        self.create_timer(0.05, self.drain_commands)
        self.create_timer(0.5, self.encode_grid)

        web = Path(get_package_share_directory("fault_management")) / "web/dashboard.html"
        self.page = web.read_bytes()
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def reply(self, code, body, kind="application/json"):
                self.send_response(code)
                self.send_header("Content-Type", kind)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                url = urlparse(self.path)
                if url.path == "/":
                    self.reply(200, node.page, "text/html; charset=utf-8")
                elif url.path == "/world":
                    self.reply(200, json.dumps(node.world).encode())
                elif url.path == "/state":
                    query = {k: int(v[0]) for k, v in parse_qs(url.query).items() if v[0].isdigit()}
                    self.reply(200, json.dumps(node.snapshot(query)).encode())
                else:
                    self.reply(404, b"{}")

            def do_POST(self):
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    body = json.loads(self.rfile.read(min(length, 4096)) or b"{}")
                    if self.path == "/reset":
                        node.commands.put(("reset", None))
                    elif self.path == "/block":
                        node.add_block()
                    elif self.path == "/goal":
                        reason = goal_error(node.world, body["x"], body["y"])
                        if reason:
                            raise ValueError(reason)
                        node.commands.put(("goal", (float(body["x"]), float(body["y"]))))
                    elif self.path == "/control":
                        node.commands.put((body["name"], validate_control(
                            body["name"], body["value"])))
                    else:
                        return self.reply(404, b"{}")
                    self.reply(200, b'{"ok":true}')
                except (ValueError, KeyError, TypeError, RuntimeError, ImportError) as error:
                    self.reply(400, json.dumps({"ok": False, "error": str(error)}).encode())

        self.server = ThreadingHTTPServer(
            (self.get_parameter("host").value, self.get_parameter("port").value), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info(
            f"Dashboard at http://localhost:{self.server.server_address[1]}")

    # Measurements -------------------------------------------------------------------
    def now_s(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = now
        return now - self.start_time

    def on_truth(self, message):
        position = message.pose.position
        yaw = yaw_from_quaternion(message.pose.orientation)
        with self.lock:
            self.state["truth"] = [position.x, position.y, yaw]
            time_s = self.now_s()
            if not self.recording:
                return
            if self.last_sample is not None and 0 <= time_s - self.last_sample < SAMPLE_PERIOD_S:
                return
            self.last_sample = time_s
            truth = (position.x, position.y)
            errors = {}
            for name in ("fused", "baseline"):
                estimate = self.state[name]
                errors[name] = position_error(estimate[:2] if estimate else None, truth)
                self.stats[name].add(time_s, errors[name])
            self.trail["truth"].append([round(position.x, 3), round(position.y, 3)])
            for name in ("fused", "baseline"):
                estimate = self.state[name]
                if estimate:
                    self.trail[name].append([round(estimate[0], 3), round(estimate[1], 3)])
            self.series.append([round(time_s, 2), errors["fused"], errors["baseline"]])

    def on_pose(self, name, message):
        position = message.pose.pose.position
        sigma = stddev_from_covariance(list(message.pose.covariance))
        with self.lock:
            self.state[name] = [position.x, position.y, sigma]

    def on_gps(self, message):
        position = message.pose.pose.position
        with self.lock:
            self.state["gps"] = [position.x, position.y, self.now_s()]

    def on_uav(self, message):
        position = message.pose.pose.position
        with self.lock:
            self.state["uav"] = [position.x, position.y]

    def on_route(self, message):
        poses = message.path.poses
        step = max(1, len(poses) // ROUTE_POINTS)
        points = [[p.pose.position.x, p.pose.position.y] for p in poses[::step]]
        if poses and (len(poses) - 1) % step:
            points.append([poses[-1].pose.position.x, poses[-1].pose.position.y])
        with self.lock:
            self.state["route"] = points
            self.state["partial"] = bool(message.partial)
            self.safety = message.clearance_grid
            self.layers_dirty = True

    def on_faults(self, message):
        with self.lock:
            self.state["flags"] = dict(
                valid=bool(message.window_valid), gps=bool(message.gps_outage),
                wheels=bool(message.odometry_scale), gyro=bool(message.imu_drift),
                uav=bool(message.uav_loss))

    def on_control(self, message):
        try:
            tick = json.loads(message.data)
        except ValueError:
            return
        mode = tick.get("supervisor_state")
        with self.lock:
            self.state["mode"] = mode
            self.state["reason"] = tick.get("hold_reason", "")
            if not self.recording and mode and "NAV" in mode.upper():
                self.recording = True
                for trail in self.trail.values():
                    trail.clear()
                self.series.clear()
                for stats in self.stats.values():
                    stats.reset()

    def on_grid(self, message):
        with self.lock:
            self.grid = message
            self.layers_dirty = True

    def encode_grid(self):
        """Build the map layers at 2 Hz: seen ground, detected obstacles, planner halo."""
        with self.lock:
            grid, safety, dirty = self.grid, self.safety, self.layers_dirty
            self.layers_dirty = False
        if grid is None or not dirty:
            return
        info = grid.info
        same = (safety is not None and safety.info.width == info.width
                and safety.info.height == info.height
                and safety.info.resolution == info.resolution)
        classes = classify_layers(grid.data, info.width, info.height,
                                  safety.data if same else None, info.resolution)
        layers = encode_layers(classes, info.origin.position.x, info.origin.position.y,
                               info.resolution)
        with self.lock:
            self.layers = layers
            self.layers_seq += 1

    def gazebo_world(self):
        from simulation_support.obstacle_world import GazeboWorld
        if self.gz_world is None:
            self.gz_world = GazeboWorld("simple_goal_demo")
        return self.gz_world

    def add_block(self):
        """Drop a dark box on the route ahead of the vehicle, leaving room to pass it."""
        from simulation_support.obstacle_scenarios import Box, Operation
        with self.block_lock:
            if len(self.blocks) >= MAX_BLOCKS:
                raise ValueError("block limit reached")
            with self.lock:
                route = [list(p) for p in self.state["route"]]
                here = self.state["fused"] or self.state["truth"]
                goal = self.state["goal"] or self.world["goal"]
            footprints = self.world["obstacles"] + [
                dict(kind="box", x=b.pose[0], y=b.pose[1], half_x=b.size[0] / 2,
                     half_y=b.size[1] / 2) for b in self.blocks]
            site = block_site(route, here[:2] if here else None, footprints, goal)
            if site is None:
                raise ValueError("no room ahead on the route")
            self.block_count += 1
            box = Box(id=f"live_block_{self.block_count}", size=BLOCK_SIZE,
                      pose=(round(site[0], 2), round(site[1], 2), BLOCK_SIZE[2] / 2, 0.0))
            self.gazebo_world().apply(
                Operation(id=f"add_{box.id}", at_ns=0, action="spawn", obstacle=box.id), box)
            self.blocks.append(box)

    def remove_blocks(self):
        from simulation_support.obstacle_scenarios import Operation
        with self.block_lock:
            while self.blocks:
                box = self.blocks[-1]
                try:
                    self.gazebo_world().apply(
                        Operation(id=f"remove_{box.id}", at_ns=0, action="remove",
                                  obstacle=box.id), box)
                except RuntimeError as error:
                    self.get_logger().warning(f"{box.id}: {error}")
                self.blocks.pop()

    def on_goal(self, message):
        with self.lock:
            self.state["goal"] = [message.pose.position.x, message.pose.position.y]

    def on_scenario(self, message):
        try:
            hazards = json.loads(message.data).get("active", [])
        except ValueError:
            return
        with self.lock:
            self.state["hazards"] = hazards

    # Controls -----------------------------------------------------------------------
    def drain_commands(self):
        while True:
            try:
                name, value = self.commands.get_nowait()
            except queue.Empty:
                return
            if name == "goal":
                self.apply_goal(*value)
            elif name == "reset":
                threading.Thread(target=self.remove_blocks, daemon=True).start()
                with self.lock:
                    for stats in self.stats.values():
                        stats.reset()
                    self.series.clear()
                for control, nominal in NOMINAL.items():
                    self.apply(control, nominal)
            else:
                self.apply(name, value)

    def apply_goal(self, x, y):
        if not self.goal_client.service_is_ready():
            return
        request = SetParameters.Request()
        for name, value in (("goal_x", x), ("goal_y", y)):
            parameter = Parameter(name=name)
            parameter.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                             double_value=float(value))
            request.parameters.append(parameter)
        self.goal_client.call_async(request).add_done_callback(
            lambda result: self.move_goal_marker(x, y) if self.goal_accepted(result) else None)

    @staticmethod
    def goal_accepted(result):
        try:
            return all(r.successful for r in result.result().results)
        except Exception:
            return False

    def move_goal_marker(self, x, y):
        """Best effort: keep the bullseye in the Gazebo world under the new goal."""
        request = (f'name: "global_goal_marker" position {{x: {x} y: {y} z: 0}} '
                   'orientation {w: 1}')
        command = ["gz", "service", "-s", "/world/simple_goal_demo/set_pose",
                   "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                   "--timeout", "1000", "--req", request]
        threading.Thread(target=lambda: subprocess.run(
            command, capture_output=True, timeout=5, check=False), daemon=True).start()

    def apply(self, name, value):
        if name in self.service_clients:
            client = self.service_clients[name]
            if not client.service_is_ready():
                return
            request = SetBool.Request()
            request.data = bool(value)
            future = client.call_async(request)
        else:
            if not self.parameter_client.service_is_ready():
                return
            request = SetParameters.Request()
            parameter = Parameter(name=name)
            parameter.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                             double_value=float(value))
            request.parameters = [parameter]
            future = self.parameter_client.call_async(request)

        def done(result):
            try:
                response = result.result()
            except Exception as error:  # service vanished; leave the shown value unchanged
                self.get_logger().warning(f"{name}: {error}")
                return
            ok = response.success if hasattr(response, "success") else response.results[0].successful
            if ok:
                with self.lock:
                    self.controls[name] = value

        future.add_done_callback(done)

    # Output -------------------------------------------------------------------------
    def snapshot(self, counts):
        with self.lock:
            state = self.state
            available = {name: client.service_is_ready()
                         for name, client in self.service_clients.items()}
            available.update({name: self.parameter_client.service_is_ready()
                              for name in PARAMETER_CONTROLS})
            gps = state["gps"]
            now = self.now_s() if self.start_time is not None else 0.0
            return dict(
                t=round(now, 2), started=self.recording, truth=state["truth"],
                fused=state["fused"], goal=state["goal"],
                baseline=state["baseline"], uav=state["uav"], route=state["route"],
                partial=state["partial"], mode=state["mode"], reason=state["reason"],
                flags=state["flags"],
                hazards=state["hazards"] + [b.shape() for b in list(self.blocks)],
                gps=gps if gps and now - gps[2] < 2.0 else None,
                controls=self.controls, available=available,
                rms={k: v.rms for k, v in self.stats.items()},
                trail={k: v[counts.get(k, 0):] for k, v in self.trail.items()},
                trail_count={k: len(v) for k, v in self.trail.items()},
                series=self.series[counts.get("series", 0):],
                series_count=len(self.series), layers_seq=self.layers_seq,
                layers=self.layers if counts.get("layers", -1) != self.layers_seq else None)


def main():
    rclpy.init()
    node = Dashboard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
