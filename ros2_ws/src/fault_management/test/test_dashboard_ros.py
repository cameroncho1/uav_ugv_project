"""Exercise HTTP actions through real ROS parameter services and the dashboard executor."""

import json
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from uav_ugv_msgs.msg import FaultStatus

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
    event = node.snapshot({})["events"][-1]
    assert event["kind"] == "control" and event["label"] == "Sensor noise 2.0x"
    assert event["control"] == "noise_scale" and event["value"] == 2.0
    before = len(node.events)
    code, body = post(node, "/control", dict(name="wheel_speed_scale", value=1.45))
    assert code == 400 and "rejected" in body["error"]
    assert node.controls["wheel_speed_scale"] == 1.0
    assert len(node.events) == before


def test_event_trace_preserves_navigation_reason_and_actual_detector_flags(dashboard):
    node, _, _, _ = dashboard
    node.on_control(String(data=json.dumps(dict(
        supervisor_state="HOLD_GUARD", hold_reason="Position uncertainty exceeds the navigation limit"))))
    node.on_faults(FaultStatus(window_valid=True, gps_outage=True, imu_drift=False))
    events = node.snapshot({})["events"]
    assert events[-2]["mode"] == "HOLD_GUARD"
    assert events[-2]["reason"] == "Position uncertainty exceeds the navigation limit"
    assert events[-1]["flags"]["gps"] is True
    assert events[-1]["flags"]["gyro"] is False
    assert events[-1]["t"] >= events[-2]["t"]


def test_ml_display_keeps_raw_score_separate_from_confirmed_decision(dashboard):
    node, _, _, _ = dashboard
    node.on_ml(DiagnosticArray(status=[
        DiagnosticStatus(name="ugv_sensor_fault/wheel_odom/vx", level=DiagnosticStatus.ERROR,
                         values=[KeyValue(key=k, value=v) for k, v in {
                             "measured": "0.8", "reference": "0.5", "fault_probability": "0.9",
                             "threshold": "0.5", "window_start": "1.0", "window_end": "6.0"}.items()]),
        DiagnosticStatus(name="ugv_sensor_fault/decision_policy", level=DiagnosticStatus.OK,
                         values=[KeyValue(key=k, value=v) for k, v in {
                             "required": "3", "wheel_count": "1", "wheel_flag": "False"}.items()]),
    ]))
    snapshot = node.snapshot({})
    assert snapshot["ml"]["wheel"]["fault_probability"] == 0.9
    assert snapshot["ml"]["policy"]["valid"] is True
    assert snapshot["ml"]["policy"]["wheel_flag"] == "False"
    assert snapshot["ml_count"] == 1
    node.on_ml(DiagnosticArray(status=[DiagnosticStatus(
        name="ugv_sensor_fault/wheel_odom/vx", level=DiagnosticStatus.STALE)]))
    assert node.snapshot({})["ml"]["wheel"] is None
