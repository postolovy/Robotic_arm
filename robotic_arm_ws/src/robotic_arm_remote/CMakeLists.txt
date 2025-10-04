cmake_minimum_required(VERSION 3.8)
project(robotic_arm_remote)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robotic_arm_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(task_servo SHARED src/task_servo.cpp)
target_include_directories(task_servo PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(task_servo
  "robotic_arm_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "trajectory_msgs")
rclcpp_components_register_node(task_servo
  PLUGIN "robotic_arm_remote::TaskServo"
  EXECUTABLE task_servo_node
)

install(TARGETS
  task_servo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/alexa_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
