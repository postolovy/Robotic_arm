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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_executable(sample_publisher src/sample_publisher.cpp)
ament_target_dependencies(sample_publisher rclcpp std_msgs)

add_executable(sample_subsciber src/sample_subscriber.cpp)
ament_target_dependencies(sample_subsciber rclcpp std_msgs)

add_executable(sample_parameter src/sample_parameter.cpp)
ament_target_dependencies(sample_parameter rclcpp rcl_interfaces) 

add_executable(sample_service_server src/sample_service_server.cpp)
ament_target_dependencies(sample_service_server rclcpp robotic_arm_msgs) 

add_executable(sample_service_client src/sample_service_client.cpp)
ament_target_dependencies(sample_service_client rclcpp robotic_arm_msgs) 

add_library(sample_action_server SHARED src/sample_action_server.cpp)
target_include_directories(sample_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(sample_action_server
  PRIVATE "SAMPLE_ACTION_SERVER_CPP_BUILDING_DLL")
ament_target_dependencies(sample_action_server
  "robotic_arm_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(sample_action_server 
  PLUGIN "sample_cpp::SampleActionServer" 
  EXECUTABLE sample_action_server_node
)

add_library(sample_action_client SHARED src/sample_action_client.cpp)
target_include_directories(sample_action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(sample_action_client
  PRIVATE "SAMPLE_ACTION_CLIENT_CPP_BUILDING_DLL")
ament_target_dependencies(sample_action_client
  "robotic_arm_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(sample_action_client
  PLUGIN "sample_cpp::SampleActionClient" 
  EXECUTABLE sample_action_client_node
)

install(TARGETS
  sample_publisher
  sample_subsciber
  sample_parameter
  sample_service_server
  sample_service_client
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  sample_action_server
  sample_action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
