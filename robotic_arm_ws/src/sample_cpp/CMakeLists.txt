cmake_minimum_required(VERSION 3.8)
project(sample_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(robotic_arm_msgs REQUIRED)

add_executable(sample_publisher src/sample_publisher.cpp)
ament_target_dependencies(sample_publisher rclcpp std_msgs)

add_executable(sample_subsciber src/sample_subscriber.cpp)
ament_target_dependencies(sample_subsciber rclcpp std_msgs)

add_executable(sample_parameter src/sample_parameter.cpp)
ament_target_dependencies(sample_parameter rclcpp rcl_interfaces) 

add_executable(sample_service_server src/sample_service_server.cpp)
ament_target_dependencies(sample_service_server rclcpp robotic_arm_msgs) 

install(TARGETS
  sample_publisher
  sample_subsciber
  sample_parameter
  sample_service_server
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
