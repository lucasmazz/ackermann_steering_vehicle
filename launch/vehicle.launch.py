import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            RegisterEventHandler, ExecuteProcess)

from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node


def load_robot_description(robot_description_path, vehicle_params_path):
    """
    Loads the robot description from a Xacro file, using parameters from a YAML file.

    @param robot_description_path: Path to the robot's Xacro file.
    @param vehicle_params_path: Path to the YAML file containing the vehicle parameters.
    @return: A string containing the robot's URDF XML description.
    """
    # Load parameters from YAML file
    with open(vehicle_params_path, 'r') as file:
        vehicle_params = yaml.safe_load(file)['/**']['ros__parameters']

    # Process the Xacro file to generate the URDF representation of the robot
    robot_description = xacro.process_file(
        robot_description_path,
        mappings={key: str(value) for key, value in vehicle_params.items()})

    return robot_description.toxml()


def start_vehicle_control():
    """
    Starts the necessary controllers for the vehicle's operation in ROS 2.

    @return: A tuple containing ExecuteProcess actions for the joint state, forward velocity, 
             and forward position controllers.
    """
    joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'joint_state_broadcaster'],
        output='screen')

    forward_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'forward_velocity_controller'],
        output='screen')

    forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_position_controller'],
        output='screen')

    return (joint_state_controller,
            forward_velocity_controller,
            forward_position_controller)


def generate_launch_description():
    # Define a launch argument for the world file, defaulting to "empty.sdf"
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='Specify the world file for Gazebo (e.g., empty.sdf)')

    # Define launch arguments for initial pose
    x_arg = DeclareLaunchArgument('x', default_value='0.0',
                                  description='Initial X position')

    y_arg = DeclareLaunchArgument('y', default_value='0.0',
                                  description='Initial Y position')

    z_arg = DeclareLaunchArgument('z', default_value='0.1',
                                  description='Initial Z position')

    roll_arg = DeclareLaunchArgument('R', default_value='0.0',
                                     description='Initial Roll')

    pitch_arg = DeclareLaunchArgument('P', default_value='0.0',
                                      description='Initial Pitch')

    yaw_arg = DeclareLaunchArgument('Y', default_value='0.0',
                                    description='Initial Yaw')

    # Retrieve launch configurations
    world_file = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('R')
    pitch = LaunchConfiguration('P')
    yaw = LaunchConfiguration('Y')

    # Define the robot's and package name
    robot_name = "ackermann_steering_vehicle"
    robot_pkg_path = get_package_share_directory(robot_name)

    # Set paths to Xacro model and configuration files
    robot_description_path = os.path.join(robot_pkg_path, 'model',
                                          'vehicle.xacro')

    gz_bridge_params_path = os.path.join(robot_pkg_path, 'config',
                                         'ros_gz_bridge.yaml')

    vehicle_params_path = os.path.join(robot_pkg_path, 'config',
                                       'parameters.yaml')
    # Load URDF
    robot_description = load_robot_description(robot_description_path,
                                               vehicle_params_path)

    # Prepare to include the Gazebo simulation launch file
    gazebo_pkg_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'),
                     'launch',
                     'gz_sim.launch.py'))

    # Include the Gazebo launch description
    gazebo_launch = IncludeLaunchDescription(
        gazebo_pkg_launch,
        launch_arguments={'gz_args': [f'-r -v 4 ', world_file],
                          'on_exit_shutdown': 'true'}.items())

    # Create node to spawn robot model in the Gazebo world
    spawn_model_gazebo_node = Node(package='ros_gz_sim',
                                   executable='create',
                                   arguments=['-name', robot_name,
                                              '-string', robot_description,
                                              '-x', x,
                                              '-y', y,
                                              '-z', z,
                                              '-R', roll,
                                              '-P', pitch,
                                              '-Y', yaw,
                                              '-allow_renaming', 'false'],
                                   output='screen')

    # Create node to publish the robot state
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description,
                                          'use_sim_time': True}],
                                      output='screen')

    # Create a node for the ROS-Gazebo bridge to handle message passing
    gz_bridge_node = Node(package='ros_gz_bridge',
                          executable='parameter_bridge',
                          arguments=['--ros-args', '-p',
                                     f'config_file:={gz_bridge_params_path}'],
                          output='screen')

    # Start controllers
    joint_state, forward_velocity, forward_position = start_vehicle_control()

    # Load vehicle controller node
    vehicle_controller_node = Node(package='ackermann_steering_vehicle',
                                   executable='vehicle_controller',
                                   parameters=[vehicle_params_path],
                                   output='screen')

    # Load joystick controller node
    joystick_node = Node(package="joy", executable="joy_node")

    # Create the launch description
    launch_description = LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=spawn_model_gazebo_node,
                                        on_exit=[joint_state])),
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=joint_state,
                                        on_exit=[forward_velocity,
                                                 forward_position])),
        world_arg,
        gazebo_launch,
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        spawn_model_gazebo_node,
        robot_state_publisher_node,
        vehicle_controller_node,
        gz_bridge_node,
        joystick_node])

    return launch_description
