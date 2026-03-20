cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)   # 🔥 추가
find_package(visualization_msgs REQUIRED)

# 🔹 MPC 노드
add_executable(control_node src/CONTROL.cpp)
ament_target_dependencies(control_node 
  rclcpp nav_msgs std_msgs ackermann_msgs geometry_msgs tf2_ros)

# 🔹 MPC 실험 노드 (있으면 유지)
add_executable(control_node2 src/CONTROL2.cpp)
ament_target_dependencies(control_node2 
  rclcpp nav_msgs std_msgs ackermann_msgs geometry_msgs visualization_msgs tf2_ros)

# 🔹 DriveMux 노드 추가
add_executable(drive_mux_node src/drive_mux_node.cpp)
ament_target_dependencies(drive_mux_node
  rclcpp ackermann_msgs std_msgs)

# 🔹 JoyDrive 노드 (있다면)
add_executable(joy_drive_node src/joy_drive_node.cpp)
ament_target_dependencies(joy_drive_node
  rclcpp sensor_msgs ackermann_msgs std_msgs)

# 설치
install(TARGETS 
  control_node 
  control_node2
  drive_mux_node
  joy_drive_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
