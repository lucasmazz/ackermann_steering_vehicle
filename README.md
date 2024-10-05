# Ackermann Steering Vehicle Simulation

## About

This package, built with ROS 2 Jazzy Jalisco and Gazebo Harmonic, launches a simulation of an Ackermann steering vehicle. The vehicle model includes steering angle and velocity control, along with an embedded front camera that streams live images for vision-based tasks. This setup could be used for developing and testing autonomous driving algorithms in a simulated environment.

## Requirements

To use this package, you'll need the following:

- [Linux Ubuntu 24.04](https://ubuntu.com/blog/tag/ubuntu-24-04-lts)
- [ROS2 Jazzy Jalisco](https://docs.ros.org/en/rolling/Releases/Release-Jazzy-Jalisco.html)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/)

**Make sure to install the following ROS 2 Jazzy Jalisco packages:**

```bash
sudo apt install -y                         \
    ros-jazzy-ros2-controllers              \
    ros-jazzy-ros-gz                        \
    ros-jazzy-ros-gz-bridge                 \
    ros-jazzy-joint-state-publisher         \
    ros-jazzy-robot-state-publisher         \
    ros-jazzy-xacro                         \
```

## Usage

### Clone the Repository

Clone this repository into your ```workspace/src``` folder. If you don't have a workspace set up, you can learn more about creating one in the [ROS 2 workspace tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

```bash
cd <path_to_your_workspace>/src
git clone git@github.com:lucasmazz/ackermann_steering_vehicle.git
```

### Build the Package
Source the ROS 2 environment and build the package:

```bash
source /opt/ros/jazzy/setup.bash
cd <path_to_your_workspace>
colcon build
```

### Launch the Vehicle

After building the package, launch the ```vehicle.launch.py``` file from the ```ackermann_steering_vehicle``` package:

```bash
source /opt/ros/jazzy/setup.bash
cd <path_to_your_workspace>
source install/setup.bash
ros2 launch ackermann_steering_vehicle vehicle.launch.py
```

#### Arguments

To launch the robot in a specified world with a custom initial pose, run the ```vehicle.launch.py``` file and specify the world path and robot pose arguments.

- **world**: Path to the world file
- **x**: Initial x-coordinate of the robot
- **y**: Initial y-coordinate of the robot
- **z**: Initial z-coordinate of the robot
- **R**: Initial roll orientation
- **P**: Initial pitch orientation
- **Y**: Initial yaw orientation

In the following example, the robot starts at position (x, y, z) = (1.0, 2.0, 0.5) with a yaw of 1.57 radians in the specified world:

```bash
ros2 launch ackermann_steering_vehicle vehicle.launch.py world:=/path_to_world/world.sdf x:=1.0 y:=2.0 z:=0.5 R:=0.0 P:=0.0 Y:=1.57
```

### Control the Vehicle

#### Topics

The vehicle can be controlled by publishing the steering angle in radians and velocity in meters per second to the respective topics:

```bash
/steering_angle
/velocity
```

The following topics can be subscribed to access the camera image and retrieve its information:

```bash
/camera/image_raw
/camera/info
```

#### Joystick

To control the vehicle, you can use a video game joystick by launching ```joystick.launch.py```. This launch file starts the joystick_controller node, designed specifically for compatibility with an Xbox One joystick, to interface with the ```velocity``` and ```steering_angle``` topics.

To launch the ```joystick.launch.py```, run the following commands:

```bash
source /opt/ros/jazzy/setup.bash
cd <path_to_your_workspace>
source install/setup.bash
ros2 launch ackermann_steering_vehicle joystick.launch.py
```

## Parameters

The parameters for the vehicle model, control, and camera can be configured in the ```ackermann_steering_vehicle/config/parameters.yaml``` file. This file includes the following settings with their default values for the simulation:

```yaml
# Body params
body_length: 0.3 # Length of the vehicle's body [m]
body_width: 0.18 # Width of the vehicle's body [m]
body_height: 0.05 # Height of the vehicle's body [m]
body_density: 7850.0 # Density of the vehicle's body material, e.g., steel [kg/m^3]

# Wheel params
wheel_radius: 0.04 # Radius of each wheel [m]
wheel_width: 0.02 # Width of each wheel [m]
wheel_density: 900.0 # Density of the wheel material, e.g., rubber [kg/m^3]

# Kinematics and dynamics params
max_steering_angle: 0.6108652 # Maximum steering angle of the vehicle [rad]
max_steering_angular_velocity: 1.570796 # Maximum steering angular velocity [rad/s]
max_steering_effort: 1.0 # Maximum steering torque [Nm]
max_velocity: 0.5 # Maximum wheel velocity [m/s]
max_effort: 10.0 # Maximum wheel torque [Nm]

# Camera and image params
camera_box_size: 0.05 # Size of the camera enclosure [m]
camera_stick_size: 0.02 # Size of the camera stick [m]
camera_height: 0.2 # Height of the camera above the body_link [m]
camera_pitch: 0.6108652 # Pitch angle of the camera relative to body_link [rad]
camera_fov: 1.3962634 # Field of view of the camera [rad]
camera_fps: 30 # Frames per second for camera capture [Hz]
image_width: 640 # Width of the camera's image output [pixels]
image_height: 480 # Height of the camera's image output [pixels]
```