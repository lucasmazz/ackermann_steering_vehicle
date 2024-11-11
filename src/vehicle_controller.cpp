#include "vehicle_controller.hpp"

VehicleController::VehicleController(const double timer_period = 0.01,
                                     const double timeout_duration = 1e9) :
  Node("vehicle_controller"),
  timeout_duration_(timeout_duration),
  last_message_time_(get_clock()->now()),
  body_width_(0.0),
  body_length_(0.0),
  wheel_radius_(0.0),
  wheel_width_(0.0),
  max_steering_angle_(0.0),
  max_velocity_(0.0),
  wheel_base_(0.0),
  track_width_(0.0),
  angular_velocity_{0.0, 0.0},
  position_{0.0, 0.0}
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
  desired_angle_subscriber_ = create_subscription<std_msgs::msg::Float64>(
    "/desired_angle", 10,
    std::bind(&VehicleController::desired_angle_callback, this, std::placeholders::_1));

  desired_velocity_subscriber_ = create_subscription<std_msgs::msg::Float64>(
    "/desired_velocity", 10,
    std::bind(&VehicleController::desired_velocity_callback, this, std::placeholders::_1));

  // Publishers
  position_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/forward_position_controller/commands", 10);

  velocity_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/forward_velocity_controller/commands", 10);

  // Timer loop
  timer_ = create_wall_timer(std::chrono::duration<double>(timer_period),
                             std::bind(&VehicleController::timer_callback, this));
}

std::pair<double, double> VehicleController::ackermann_steering_angles(const double angle)
{
  double left_wheel_angle = 0.0;
  double right_wheel_angle = 0.0;

  if (angle > 0) {
    left_wheel_angle = atan((2 * wheel_base_ * sin(angle)) /
                            (2 * wheel_base_ * cos(angle) - track_width_ * sin(angle)));

    right_wheel_angle = atan((2 * wheel_base_ * sin(angle)) /
                             (2 * wheel_base_ * cos(angle) + track_width_ * sin(angle)));
  } else if (angle < 0) {
    left_wheel_angle = -atan((2 * wheel_base_ * sin(-angle)) /
                             (2 * wheel_base_ * cos(-angle) + track_width_ * sin(-angle)));

    right_wheel_angle = -atan((2 * wheel_base_ * sin(-angle)) /
                              (2 * wheel_base_ * cos(-angle) - track_width_ * sin(-angle)));
  }

  return std::make_pair(left_wheel_angle, right_wheel_angle);
}

void VehicleController::timer_callback()
{
  auto current_time = get_clock()->now();
  auto elapsed_time = (current_time - last_message_time_).nanoseconds();

  // Reset position and velocity to zero if timeout
  if (elapsed_time > timeout_duration_) {
    position_ = {0.0, 0.0};
    angular_velocity_ = {0.0, 0.0};
  }

  // Publish steering position
  std_msgs::msg::Float64MultiArray position_msg;
  position_msg.data = position_;
  position_publisher_->publish(position_msg);

  // Publish wheels velocity
  std_msgs::msg::Float64MultiArray velocity_msg;
  velocity_msg.data = angular_velocity_; 
  velocity_publisher_->publish(velocity_msg);
}

void VehicleController::desired_angle_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  last_message_time_ = get_clock()->now();  // Update timestamp

  double steering_angle = msg->data;

  if (steering_angle > max_steering_angle_) {
    steering_angle = max_steering_angle_;
  } else if (steering_angle < -max_steering_angle_) {
    steering_angle = -max_steering_angle_;
  }

  auto angles = ackermann_steering_angles(steering_angle);
  position_ = {angles.first, angles.second};  // Left and right steering angles
}

void VehicleController::desired_velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  last_message_time_ = get_clock()->now();  // Update timestamp

  double velocity = msg->data;

  if (velocity > max_velocity_) {
    velocity = max_velocity_;
  } else if (velocity < -max_velocity_) {
    velocity = -max_velocity_;
  }

  // Convert linear speed to angular speed
  angular_velocity_ = {(velocity / wheel_radius_), (velocity / wheel_radius_)};
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleController>());
  rclcpp::shutdown();
  return 0;
}
