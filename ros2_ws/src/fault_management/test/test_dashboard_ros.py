"""Exercise HTTP actions through real ROS parameter services and the dashboard executor."""

import json
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from demo_dashboard import Dashboard


@pytest.fixture
def dashboard():
    rclpy.init(args=["--ros-args", "-p", "port:=0"])
    goal = Node("goal_manager")
    goal.declare_parameter("goal_x", 4.0)
    goal.declare_parameter("goal_y", 0.0)
    revisions = []

    def goal_change(parameters):
        values = {p.name: p.value for p in parameters}
        if values.get("goal_x") == 13.0:
            return SetParametersResult(successful=False, reason="test goal rejected")
        revisions.append(values)
        return SetParametersResult(successful=True)

    goal.add_on_set_parameters_callback(goal_change)
    sensor = Node("sim_ugv_sensors")
    for name, value in (("noise_scale", 1.0), ("gps_horizontal_stddev", 1.5),
                        ("wheel_speed_scale", 1.0), ("imu_yaw_bias_rate", 0.0)):
        sensor.declare_parameter(name, value)
    sensor.add_on_set_parameters_callback(lambda parameters: SetParametersResult(
        successful=all(p.name != "wheel_speed_scale" or p.value < 1.4 for p in parameters)))
    node = Dashboard()
    node.move_goal_marker = lambda x, y: None
    executor = MultiThreadedExecutor(num_threads=2)
    for item in (goal, sensor, node):
        executor.add_node(item)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        deadline = time.monotonic() + 5.0
        while not (node.goal_client.service_is_ready() and node.parameter_client.service_is_ready()):
            assert time.monotonic() < deadline, "test parameter services did not appear"
            time.sleep(0.02)
        yield node, goal, sensor, revisions
    finally:
        node.server.shutdown()
        node.server.server_close()
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=3.0)
        for item in (node, goal, sensor):
            item.destroy_node()
        rclpy.shutdown()


def post(node, path, value):
    request = Request(f"http://127.0.0.1:{node.server.server_port}{path}",
                      data=json.dumps(value).encode(), headers={"Content-Type": "application/json"})
    try:
        with urlopen(request, timeout=10) as response:
            return response.status, json.load(response)
    except HTTPError as response:
        return response.code, json.load(response)


def test_goal_is_one_atomic_revision_and_rejection_reaches_browser(dashboard):
    node, goal, _, revisions = dashboard
    assert post(node, "/goal", dict(x=1.0, y=2.0))[0] == 200
    assert revisions == [dict(goal_x=1.0, goal_y=2.0)]
    assert goal.get_parameter("goal_y").value == 2.0
    code, body = post(node, "/goal", dict(x=13.0, y=5.0))
    assert code == 400 and "rejected" in body["error"]
    assert goal.get_parameter("goal_x").value == 1.0
    assert goal.get_parameter("goal_y").value == 2.0
    assert len(revisions) == 1


def test_sensor_action_reports_service_result_and_preserves_rejected_value(dashboard):
    node, _, sensor, _ = dashboard
    assert post(node, "/control", dict(name="noise_scale", value=2.0))[0] == 200
    assert sensor.get_parameter("noise_scale").value == 2.0
    code, body = post(node, "/control", dict(name="wheel_speed_scale", value=1.45))
    assert code == 400 and "rejected" in body["error"]
    assert node.controls["wheel_speed_scale"] == 1.0
