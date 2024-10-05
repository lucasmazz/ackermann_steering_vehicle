#ifndef JOY_CONTROLLER_HPP
#define JOY_CONTROLLER_HPP

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"

class JoystickController : public rclcpp::Node
{

public:
  JoystickController(const double timer_period = 0.01);

private:
  /**
   * @brief Callback function to process joystick inputs and update control states.
   * 
   * @details This function listens to joystick messages and updates the desired steering angle,
   * desired velocity, and the velocity lock state based on the joystick inputs.
   * - Button 0 toggles the velocity lock state each time it is pressed.
   * - Axis 0 is used to calculate the desired steering angle as a fraction of the maximum steering angle.
   * - Axis 3 is used to calculate the desired velocity as a fraction of the maximum velocity.
   * 
   * @param msg Pointer to the received Joy message containing button and axis states.
   */
  void listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  /**
   * @brief Timer callback function to publish desired steering angle and velocity.
   * 
   * @details This function periodically publishes the desired steering angle and velocity 
   * to their respective topics. If the velocity lock is enabled, the published 
   * velocity is set to the maximum velocity.
   */
  void timer_callback();

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_angle_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool velocity_lock_enabled_;
  bool last_button_state_;
  double max_steering_angle_;
  double max_velocity_;

  double steering_angle_;
  double velocity_;
};

#endif  // JOY_CONTROLLER_HPP
