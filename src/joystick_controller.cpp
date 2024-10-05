#include "joystick_controller.hpp"

JoystickController::JoystickController(const double timer_period) :
  Node("joystick_controller"),
  speed_lock_enabled_(false),
  desired_angle_(0.0),
  desired_speed_(0.0),
  max_steering_angle_(0.0),
  max_velocity_(0.0)
{
  // Declare the used parameters
  declare_parameter<double>("max_steering_angle", 0.0);
  declare_parameter<double>("max_velocity", 0.0);

  // Get parameters on startup
  get_parameter("max_steering_angle", max_steering_angle_);
  get_parameter("max_velocity", max_velocity_);

  // Subscription to the 'joy' topic
  subscriber_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 1, std::bind(&JoystickController::listener_callback, this, std::placeholders::_1));

  // Publishers for /desired_angle and /desired_speed topics
  angle_publisher_ = create_publisher<std_msgs::msg::Float64>("/desired_angle", 1);
  speed_publisher_ = create_publisher<std_msgs::msg::Float64>("/desired_speed", 1);

  // Timer to periodically publish the desired angle and speed
  timer_ = create_wall_timer(std::chrono::duration<double>(timer_period),
                             std::bind(&JoystickController::timer_callback, this));
}

void JoystickController::listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  const bool button_state = static_cast<bool>(msg->buttons[0]);

  // Toggle the speed lock status each time the button is pressed
  if (button_state && button_state != last_button_state_) {
    speed_lock_enabled_ = !speed_lock_enabled_;
  }

  last_button_state_ = button_state;

  // Set the desired angle and desired speed based on the joystick's axis
  desired_angle_ = msg->axes[0] * max_steering_angle_;
  desired_speed_ = msg->axes[3] * max_velocity_;
}

void JoystickController::timer_callback()
{
  // Publish the desired angle
  auto angle_msg = std::make_shared<std_msgs::msg::Float64>();
  angle_msg->data = desired_angle_;
  angle_publisher_->publish(*angle_msg);

  // Publish the desired speed
  auto speed_msg = std::make_shared<std_msgs::msg::Float64>();

  if (speed_lock_enabled_) {
    // If the speed lock is active, set the speed message to the maximum speed
    speed_msg->data = max_velocity_;
  } else {
    // If the speed lock is not active, set the speed message to the desired speed
    speed_msg->data = desired_speed_;
  }

  speed_publisher_->publish(*speed_msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickController>());
  rclcpp::shutdown();
  return 0;
}
