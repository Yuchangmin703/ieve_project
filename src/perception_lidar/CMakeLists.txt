cmake_minimum_required(VERSION 3.8)
project(perception_lidar)

# C++ 컴파일러 설정 (아까 짝이 안 맞았던 그 부분입니다!)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# --- perception_node7 ---
add_executable(perception_node7 src/perception_node7.cpp)
ament_target_dependencies(perception_node7
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
)
install(TARGETS
  perception_node7
  DESTINATION lib/${PROJECT_NAME}
)

# --- perception_node8 ---
add_executable(perception_node8 src/perception_node8.cpp)
ament_target_dependencies(perception_node8
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
)
install(TARGETS
  perception_node8
  DESTINATION lib/${PROJECT_NAME}
)

# 테스트 빌드 설정
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
