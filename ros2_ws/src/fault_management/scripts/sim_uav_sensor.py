#!/usr/bin/env python3
"""Synthesize noisy body-frame observations and optional PX4-format anchor data."""

import math
import random

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from px4_msgs.msg import VehicleOdometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock

from localization_common.frames import FRD_TO_FLU, NED_TO_ENU, quaternion, rotation
from simulation_faults import common_position_bias


def vector(position):
    return np.array([position.x, position.y, position.z])


class SimUavSensor(Node):
    def __init__(self):
        super().__init__("sim_uav_sensor")

        def parameter(name, default):
            return self.declare_parameter(name, default).value

        seed = parameter("random_seed", 513)
        self.shared_std = parameter("shared_position_stddev", 0.0)
        self.shared_bias = common_position_bias(seed, self.shared_std)
        self.relative_random = random.Random(seed + 10)
        self.anchor_random = random.Random(seed + 11)
        self.relative_std = np.array(parameter("relative_stddev", [.25, .15, .25]))
        self.anchor_std = np.array(parameter("anchor_stddev", [.3, .3, .5]))
        self.attitude_std = np.array(parameter("attitude_stddev", [.01, .01, .02]))
        self.origin = np.array(parameter("map_origin", [2., -2., .3]))
        self.chassis_offset = np.array(parameter("ugv_chassis_offset", [0., 0., .25]))
        frequency = parameter("frequency", 5.)
        self.publish_anchor = parameter("publish_anchor", True)
        self.outlier_every = parameter("outlier_every", 0)
        self.dropout_start = parameter("dropout_start", -1.)
        self.dropout_end = parameter("dropout_end", -1.)
        if (any(values.shape != (3,) or not np.all(np.isfinite(values)) or np.any(values < 0)
                for values in (self.relative_std, self.anchor_std, self.attitude_std))
                or np.any(self.relative_std <= 0) or not np.isfinite(frequency) or frequency <= 0
                or self.origin.shape != (3,) or not np.all(np.isfinite(self.origin))):
            raise ValueError("Sensor deviations must be finite and nonnegative; "
                             "relative noise and frequency must be positive; origin needs xyz")
        if self.chassis_offset.shape != (3,) or not np.isfinite(self.chassis_offset).all():
            raise ValueError("UGV chassis offset requires finite xyz")
        if (self.outlier_every < 0 or not np.isfinite(self.dropout_start)
                or not np.isfinite(self.dropout_end)
                or (self.dropout_start >= 0 and self.dropout_end <= self.dropout_start)):
            raise ValueError(
                "Outlier interval must be nonnegative and dropout end must follow start")
        self.period = 1 / frequency
        self.last_stamp = None
        self.start = None
        self.count = 0
        self.relative_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                  "/ugv/sensors/uav_relative_pose", 20)
        self.anchor_pub = self.create_publisher(VehicleOdometry, "/simulation/uav_odometry", 20)
        self.create_subscription(PoseArray, "/simulation/vehicle_poses", self.on_poses,
                                 qos_profile_sensor_data)
        self.create_subscription(Clock, "/evaluation/trial_start", self.on_start,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_start(self, message):
        self.start = message.clock.sec + message.clock.nanosec*1e-9

    @staticmethod
    def noise(generator, deviations):
        return np.array([generator.gauss(0., float(value)) for value in deviations])

    def on_poses(self, message):
        if message.header.frame_id != "map" or len(message.poses) != 2:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.start = stamp
            self.last_stamp = None
        if self.last_stamp is not None and stamp-self.last_stamp < self.period-1e-6:
            return
        self.last_stamp = stamp
        uav, ugv = message.poses
        q = uav.orientation
        r = rotation([q.w, q.x, q.y, q.z])
        ugv_q = ugv.orientation
        chassis = vector(ugv.position) + rotation(
            [ugv_q.w, ugv_q.x, ugv_q.y, ugv_q.z]) @ self.chassis_offset
        relative = r.T @ (chassis-vector(uav.position))
        relative += self.noise(self.relative_random, self.relative_std)
        self.count += 1
        if self.outlier_every and self.count % self.outlier_every == 0:
            relative[0] += 8.
        dropout = (self.start is not None and
                   self.dropout_start <= stamp-self.start < self.dropout_end)
        if not dropout:
            output = PoseWithCovarianceStamped()
            output.header.stamp = message.header.stamp
            output.header.frame_id = "uav/base_link"
            p = output.pose.pose.position
            p.x, p.y, p.z = (float(value) for value in relative)
            output.pose.pose.orientation.w = 1.
            c = np.diag([*(self.relative_std**2), 1e6, 1e6, 1e6])
            output.pose.covariance = c.flatten().tolist()
            self.relative_pub.publish(output)
        if not self.publish_anchor:
            return
        position = vector(uav.position) + self.noise(self.anchor_random, self.anchor_std)
        position += self.shared_bias
        angles = self.noise(self.anchor_random, self.attitude_std)
        angle = np.linalg.norm(angles)
        error_q = (np.array([math.cos(angle/2), *(angles/angle*math.sin(angle/2))])
                   if angle else np.array([1., 0., 0., 0.]))
        measured_rotation = r @ rotation(error_q)
        output = VehicleOdometry()
        output.timestamp_sample = int(round(stamp*1e6))
        output.timestamp = output.timestamp_sample
        output.pose_frame = VehicleOdometry.POSE_FRAME_NED
        output.position = (NED_TO_ENU @ (position-self.origin)).astype(np.float32)
        output.q = quaternion(NED_TO_ENU @ measured_rotation @ FRD_TO_FLU).astype(np.float32)
        marginal_variance = self.anchor_std**2 + np.array(
            [self.shared_std**2, self.shared_std**2, 0.0])
        output.position_variance = (NED_TO_ENU**2 @ marginal_variance).astype(np.float32)
        output.orientation_variance = (self.attitude_std**2).astype(np.float32)
        self.anchor_pub.publish(output)


def main():
    rclpy.init()
    node = SimUavSensor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # ROS can close the context while the executor creates its wait set.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
