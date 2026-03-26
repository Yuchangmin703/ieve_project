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
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 🔹 1. 기존 제어 실험 노드 (원형 Pure Pursuit)
add_executable(control_node2 src/CONTROL2.cpp)
ament_target_dependencies(control_node2 
  rclcpp nav_msgs std_msgs ackermann_msgs geometry_msgs visualization_msgs tf2_ros)

# 🚀 2. JHcontrol 노드 (타원형 Pure Pursuit)
add_executable(JHcontrol src/JHcontrol.cpp)
ament_target_dependencies(JHcontrol 
  rclcpp nav_msgs std_msgs ackermann_msgs geometry_msgs visualization_msgs tf2_ros)

# 🔹 3. 나머지 노드들 (Mux, Joy 등)
add_executable(drive_mux_node src/drive_mux_node.cpp)
ament_target_dependencies(drive_mux_node rclcpp ackermann_msgs std_msgs)

add_executable(joy_drive_node src/joy_drive_node.cpp)
ament_target_dependencies(joy_drive_node rclcpp sensor_msgs ackermann_msgs std_msgs)

# 🔹 실행 파일 설치 (모두 등록)
install(TARGETS 
  control_node2
  JHcontrol
  drive_mux_node
  joy_drive_node
  DESTINATION lib/${PROJECT_NAME})

# ⭐ Launch 파일 폴더 설치
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
