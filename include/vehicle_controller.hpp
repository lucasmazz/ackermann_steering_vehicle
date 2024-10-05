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
  VehicleController(const double timer_period = 1e-2, const double timeout_duration = 1e9);

private:
  /**
   * @brief Calculates the steering angles for the left and right wheels using the 
   * Ackermann steering geometry.
   * 
   * @details This function computes the angles for the left and right front wheels based on the 
   * input steering angle and the vehicle's dimensions:
   * - `wheel_base_` (distance between front and rear axles).
   * - `track_width_` (distance between the left and right wheels).
   * 
   * @return A pair of left and right steering angles [rad].
   */
  std::pair<double, double> ackermann_steering_angle();

  /**
   * @brief Calculates the rear differential wheel velocities for a vehicle with a rear 
   * differential axle, considering the steering angle and limiting the velocities to a maximum 
   * allowable value.
   * 
   * @details This function computes rear wheel velocities based on Ackermann steering geometry:
   * - Both wheels have the same velocity for straight-line motion.
   * - For turning, it computes individual wheel velocities based on the turning radius.
   * - If any wheel velocity exceeds the maximum velocity, both are scaled proportionally.
   * 
   * @return A pair of left and right wheel velocities [m/s].
   */
  std::pair<double, double> rear_differential_velocity();

  /**
   * @brief Periodically checks for timeout and publishes steering angles and wheel velocities.
   * 
   * @details This timer callback checks if a timeout has occurred since the last received 
   * message for desired steering angle or velocity. If a timeout is detected, it resets 
   * the wheel steering angles and angular velocities to zero. The function also 
   * publishes the current steering positions and wheel velocities to the appropriate topics.
   */
  void timer_callback();

  /**
   * @brief Callback function for receiving and handling desired steering angle.
   * 
   * @details This callback updates the steering angle based on the input message and clamps 
   * the value to the maximum allowable steering angle. The function calculates the 
   * corresponding Ackermann steering angles for the left and right wheels and updates 
   * the internal state of the controller.
   * 
   * @param msg A shared pointer to the incoming message containing the steering angle [rad].
   */
  void steering_angle_callback(const std_msgs::msg::Float64::SharedPtr msg);

  /**
   * @brief Callback function for receiving and handling desired velocity.
   * 
   * @details This callback updates the vehicle velocity based on the input message and clamps 
   * the value to the maximum allowable velocity. The function calculates the wheel 
   * velocities based on the rear differential model and updates the internal state 
   * with the corresponding angular velocities of the wheels.
   * 
   * @param msg A shared pointer to the incoming message containing the velocity [m/s].
   */
  void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg);

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

  double steering_angle_;
  double velocity_;

  std::vector<double> wheel_angular_velocity_;
  std::vector<double> wheel_steering_angle_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_angle_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // VEHICLE_CONTROLLER_HPP