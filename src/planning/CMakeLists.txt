cmake_minimum_required(VERSION 3.8)
project(planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------------------------------
# 1. 필수 패키지 로드
# ----------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(perception REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

# ----------------------------------------------------
# 2. 기존 실행 파일 생성 (planning_node)
# ----------------------------------------------------
add_executable(planning_node src/planning_node.cpp)
ament_target_dependencies(planning_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  nav_msgs 
  std_msgs 
  visualization_msgs 
  perception
  image_transport
  cv_bridge
)

# ----------------------------------------------------
# ⭐ 3. 새로 추가된 실행 파일 생성 (JHplanning_node)
# ----------------------------------------------------
add_executable(JHplanning_node src/JHplanning.cpp)
ament_target_dependencies(JHplanning_node
  rclcpp
  nav_msgs
  geometry_msgs
  perception
  visualization_msgs
)

# ----------------------------------------------------
# 4. 설치 (Install) 규칙 - 더미 노드 완벽 삭제됨!
# ----------------------------------------------------
install(TARGETS
  planning_node
  JHplanning_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
