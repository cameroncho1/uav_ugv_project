"""Bridge camera pixels and calibration, then detect the calibrated UGV tag."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('cooperative_localization'))
    prefix = ['/world/', LaunchConfiguration('world'), '/model/',
              LaunchConfiguration('uav_model'), '/link/camera_link/sensor/camera/']
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='simple_goal_demo'),
        DeclareLaunchArgument('uav_model', default_value='cooperative_camera_uav_0'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='uav_camera_bridge',
             arguments=[prefix + ['image@sensor_msgs/msg/Image[gz.msgs.Image'],
                        prefix + ['camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo']],
             remappings=[(prefix + ['image'], '/uav/camera/image'),
                         (prefix + ['camera_info'], '/uav/camera/camera_info')],
             output='screen'),
        Node(package='cooperative_localization', executable='uav_fiducial_sensor.py',
             name='uav_fiducial_sensor',
             parameters=[str(share / 'config/camera_calibration.yaml')], output='screen'),
    ])
