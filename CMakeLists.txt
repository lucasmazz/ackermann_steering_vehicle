cmake_minimum_required(VERSION 3.8)
project(ackermann_steering_vehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(vehicle_controller src/vehicle_controller.cpp)
ament_target_dependencies(vehicle_controller rclcpp std_msgs)

add_executable(joystick_controller src/joystick_controller.cpp)
ament_target_dependencies(joystick_controller rclcpp std_msgs sensor_msgs)

install(
  DIRECTORY launch model config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  vehicle_controller
  joystick_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
