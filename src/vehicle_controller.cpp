#include "vehicle_controller.hpp"

VehicleController::VehicleController(const double timer_period, const double timeout_duration) :
  Node{"vehicle_controller"},
  timeout_duration_{timeout_duration},
  last_message_time_{get_clock()->now()},
  body_width_{0.0},
  body_length_{0.0},
  wheel_radius_{0.0},
  wheel_width_{0.0},
  max_steering_angle_{0.0},
  max_velocity_{0.0},
  wheel_base_{0.0},
  track_width_{0.0},
  steering_angle_{0.0},
  velocity_{0.0},
  wheel_angular_velocity_{0.0, 0.0},
  wheel_steering_angle_{0.0, 0.0}
{
  // Declare the used parameters
  declare_parameter<double>("body_width", 0.0);
  declare_parameter<double>("body_length", 0.0);
  declare_parameter<double>("wheel_radius", 0.0);
  declare_parameter<double>("wheel_width", 0.0);
  declare_parameter<double>("max_steering_angle", 0.0);
  declare_parameter<double>("max_velocity", 0.0);

  // Get parameters on startup
  get_parameter("body_width", body_width_);
  get_parameter("body_length", body_length_);
  get_parameter("wheel_radius", wheel_radius_);
  get_parameter("wheel_width", wheel_width_);
  get_parameter("max_steering_angle", max_steering_angle_);
  get_parameter("max_velocity", max_velocity_);

  // Set the track width and wheel base
  track_width_ = body_width_ + (2 * wheel_width_ / 2);
  wheel_base_ = body_length_ - (2 * wheel_radius_);

  // Subscribers
  steering_angle_subscriber_ = create_subscription<std_msgs::msg::Float64>(
    "/steering_angle", 10,
    std::bind(&VehicleController::steering_angle_callback, this, std::placeholders::_1));

  velocity_subscriber_ = create_subscription<std_msgs::msg::Float64>(
    "/velocity", 10, 
    std::bind(&VehicleController::velocity_callback, this, std::placeholders::_1));

  // Publishers
  position_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);

  velocity_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/forward_velocity_controller/commands", 10);

  // Timer loop
  timer_ = create_wall_timer(std::chrono::duration<double>(timer_period),
                             std::bind(&VehicleController::timer_callback, this));
}

std::pair<double, double> VehicleController::ackermann_steering_angle()
{
  double left_wheel_angle{0.0};
  double right_wheel_angle{0.0};

  // Steering angle is not zero nor too small
  if (abs(steering_angle_) > 1e-3) {
    const double sin_angle = sin(abs(steering_angle_));
    const double cos_angle = cos(abs(steering_angle_));

    if (steering_angle_ > 0.0) {
      // Left and right wheel angles when steering left
      left_wheel_angle = atan((2 * wheel_base_ * sin_angle) /
                              (2 * wheel_base_ * cos_angle - track_width_ * sin_angle));

      right_wheel_angle = atan((2 * wheel_base_ * sin_angle) /
                               (2 * wheel_base_ * cos_angle + track_width_ * sin_angle));
    } else {
      // Left and right wheel angles when steering right (mirror left with negative signs)
      left_wheel_angle = -atan((2 * wheel_base_ * sin_angle) /
                               (2 * wheel_base_ * cos_angle + track_width_ * sin_angle));

      right_wheel_angle = -atan((2 * wheel_base_ * sin_angle) /
                                (2 * wheel_base_ * cos_angle - track_width_ * sin_angle));
    }
  }

  return std::make_pair(left_wheel_angle, right_wheel_angle);
}

std::pair<double, double> VehicleController::rear_differential_velocity()
{
  double left_wheel_velocity{velocity_};
  double right_wheel_velocity{velocity_};

  // Steering angle is not zero nor too small
  if (abs(steering_angle_) > 1e-3) {
    // Calculate turning radius and angular velocity
    const double turning_radius = wheel_base_ / tan(abs(steering_angle_));  // [m]
    const double vehicle_angular_velocity = velocity_ / turning_radius;     // [rad/s]

    // Compute inner and outer wheel radius
    const double inner_radius = turning_radius - (track_width_ / 2.0);
    const double outer_radius = turning_radius + (track_width_ / 2.0);

    if (steering_angle_ > 0.0) {
      // Vehicle turning left
      left_wheel_velocity = vehicle_angular_velocity * inner_radius;
      right_wheel_velocity = vehicle_angular_velocity * outer_radius;
    } else {
      // Vehicle turning right
      left_wheel_velocity = vehicle_angular_velocity * outer_radius;
      right_wheel_velocity = vehicle_angular_velocity * inner_radius;
    }

    // Determine the maximum wheel velocity
    const double max_wheel_velocity = std::max(abs(left_wheel_velocity), abs(right_wheel_velocity));

    // Scale both wheel velocities proportionally if the maximum is exceeded
    if (max_wheel_velocity > max_velocity_) {
      const double scaling_factor = max_velocity_ / max_wheel_velocity;
      left_wheel_velocity *= scaling_factor;
      right_wheel_velocity *= scaling_factor;
    }
  }

  return std::make_pair(left_wheel_velocity, right_wheel_velocity);
}

void VehicleController::timer_callback()
{
  const auto current_time{get_clock()->now()};
  const auto elapsed_time{(current_time - last_message_time_).nanoseconds()};

  // Reset position and velocity to zero if timeout
  if (elapsed_time > timeout_duration_) {
    wheel_steering_angle_ = {0.0, 0.0};
    wheel_angular_velocity_ = {0.0, 0.0};
  }

  // Publish steering position
  std_msgs::msg::Float64MultiArray position_msg;
  position_msg.data = wheel_steering_angle_;
  position_publisher_->publish(position_msg);

  // Publish wheels velocity
  std_msgs::msg::Float64MultiArray velocity_msg;
  velocity_msg.data = wheel_angular_velocity_;
  velocity_publisher_->publish(velocity_msg);
}

void VehicleController::steering_angle_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  last_message_time_ = get_clock()->now();  // Update timestamp

  if (msg->data > max_steering_angle_) {
    steering_angle_ = max_steering_angle_;
  } else if (msg->data < -max_steering_angle_) {
    steering_angle_ = -max_steering_angle_;
  } else {
    steering_angle_ = msg->data;
  }

  const auto wheel_angles{ackermann_steering_angle()};

  wheel_steering_angle_ = {wheel_angles.first, wheel_angles.second};
}

void VehicleController::velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  last_message_time_ = get_clock()->now();  // Update timestamp

  if (msg->data > max_velocity_) {
    velocity_ = max_velocity_;
  } else if (msg->data < -max_velocity_) {
    velocity_ = -max_velocity_;
  } else {
    velocity_ = msg->data;
  }

  const auto wheel_velocity{rear_differential_velocity()};

  // Convert wheel linear velocity to wheel angular velocity
  wheel_angular_velocity_ = {(wheel_velocity.first / wheel_radius_),
                             (wheel_velocity.second / wheel_radius_)};
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleController>());
  rclcpp::shutdown();
  return 0;
}
