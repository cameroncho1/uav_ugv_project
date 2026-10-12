#!/usr/bin/env python3
"""Apply a stated shared position fault to the actual PX4 anchor for evaluation."""

import copy

import numpy as np
from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from localization_common.frames import NED_TO_ENU
from simulation_faults import common_position_bias


def biased_anchor(message, bias_enu, standard_deviation):
    if message.pose_frame != VehicleOdometry.POSE_FRAME_NED:
        raise ValueError('Anchor fault requires a NED pose')
    if not np.isfinite(standard_deviation) or standard_deviation < 0:
        raise ValueError('Shared deviation must be finite and nonnegative')
    bias_enu = np.asarray(bias_enu, dtype=float)
    if bias_enu.shape != (3,) or not np.isfinite(bias_enu).all():
        raise ValueError('Shared bias requires finite ENU xyz')
    output = copy.deepcopy(message)
    output.position = (np.array(message.position) + NED_TO_ENU @ bias_enu).astype(np.float32)
    variance = np.array(message.position_variance)
    variance[:2] += standard_deviation**2
    output.position_variance = variance.astype(np.float32)
    return output


class AnchorFault(Node):
    def __init__(self):
        super().__init__('sim_anchor_fault')
        seed = self.declare_parameter('random_seed', 513).value
        self.stddev = self.declare_parameter('shared_position_stddev', 0.0).value
        self.bias = common_position_bias(seed, self.stddev)
        self.publisher = self.create_publisher(
            VehicleOdometry, '/simulation/biased_uav_odometry', 10)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.on_anchor,
                                 qos_profile_sensor_data)

    def on_anchor(self, message):
        try:
            self.publisher.publish(biased_anchor(message, self.bias, self.stddev))
        except ValueError:
            return


def main():
    rclpy.init()
    node = AnchorFault()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
