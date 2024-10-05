import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_name = "ackermann_steering_vehicle"

    vehicle_params_path = os.path.join(get_package_share_directory(robot_name),
                                       'config', 'parameters.yaml')

    return LaunchDescription([Node(package=robot_name,
                                   executable='joystick_controller',
                                   parameters=[vehicle_params_path],
                                   output='screen')])
