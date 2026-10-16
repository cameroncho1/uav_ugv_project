"""Bridge camera pixels and calibration, then detect the calibrated UGV tag."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('uav_sensing'))
    prefix = ['/world/', LaunchConfiguration('world'), '/model/',
              LaunchConfiguration('uav_model'), '/link/camera_link/sensor/camera/']
    return LaunchDescription([
        DeclareLaunchArgument('detect_ugv', default_value='true'),
        DeclareLaunchArgument('world', default_value='simple_goal_demo'),
        DeclareLaunchArgument('uav_model', default_value='cooperative_camera_uav_0'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='uav_camera_bridge',
             arguments=[prefix + ['image@sensor_msgs/msg/Image[gz.msgs.Image'],
                        prefix + ['camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo']],
             remappings=[(prefix + ['image'], '/uav/camera/image'),
                         (prefix + ['camera_info'], '/uav/camera/camera_info')],
             additional_env={'GZ_PARTITION': EnvironmentVariable(
                 'GZ_PARTITION', default_value='uav_ugv_sim'), 'GZ_IP': '127.0.0.1'},
             output='screen'),
        Node(package='uav_sensing', executable='uav_fiducial_sensor.py',
             name='uav_fiducial_sensor', condition=IfCondition(LaunchConfiguration('detect_ugv')),
             parameters=[str(share / 'config/camera_calibration.yaml')], output='screen'),
    ])
