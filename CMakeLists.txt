cmake_minimum_required(VERSION 3.8)
project(tugbot_autoware_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(ackermann_to_twist_node src/ackermann_to_twist.cpp)
ament_target_dependencies(ackermann_to_twist_node rclcpp geometry_msgs autoware_auto_control_msgs)

add_executable(steering_report_publisher_node src/steering_report_publisher.cpp)
ament_target_dependencies(steering_report_publisher_node rclcpp nav_msgs autoware_auto_vehicle_msgs tf2 tf2_geometry_msgs)


install(TARGETS
  ackermann_to_twist_node
  steering_report_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  param
  config
  rviz
  map
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()