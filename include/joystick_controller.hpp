#ifndef JOY_CONTROLLER_HPP
#define JOY_CONTROLLER_HPP

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"

class JoystickController : public rclcpp::Node
{

public:
  JoystickController(double timer_period = 0.01);

private:
  void listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void timer_callback();

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool speed_lock_enabled_;
  bool last_button_state_;
  double desired_angle_;
  double desired_velocity_;
  double max_steering_angle_;
  double max_velocity_;
};

#endif  // JOY_CONTROLLER_HPP
