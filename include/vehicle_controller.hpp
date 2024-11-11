#ifndef VEHICLE_CONTROLLER_HPP
#define VEHICLE_CONTROLLER_HPP

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class VehicleController : public rclcpp::Node
{

public:
  VehicleController(const double timer_period, const double timeout_duration);

private:
  double timeout_duration_;
  rclcpp::Time last_message_time_;

  double body_width_;
  double body_length_;
  double wheel_radius_;
  double wheel_width_;
  double max_steering_angle_;
  double max_velocity_;

  double wheel_base_;
  double track_width_;

  std::vector<double> angular_velocity_;
  std::vector<double> position_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr desired_angle_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr desired_velocity_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::pair<double, double> ackermann_steering_angles(const double angle);

  void timer_callback();

  void desired_angle_callback(const std_msgs::msg::Float64::SharedPtr msg);

  void desired_velocity_callback(const std_msgs::msg::Float64::SharedPtr msg);
};

#endif  // VEHICLE_CONTROLLER_HPP