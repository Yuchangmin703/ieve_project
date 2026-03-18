cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc ximgproc)
find_package(yaml-cpp REQUIRED)

# ⭐ 누락되었던 필수 ROS 2 메시지 패키지 추가
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 2. 커스텀 메시지 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Lane.msg"
  "msg/Lanes.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# 3. 실행 파일(노드) 정의
add_executable(undistort_node src/undistort_node.cpp)
ament_target_dependencies(undistort_node
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
)
target_link_libraries(undistort_node
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

add_executable(bev_warp_node src/bev_warp_node.cpp)
ament_target_dependencies(bev_warp_node 
  rclcpp 
  sensor_msgs
  image_transport 
  cv_bridge
)
target_link_libraries(bev_warp_node ${OpenCV_LIBRARIES})

add_executable(lane_candidate_mask_node src/lane_candidate_mask_node.cpp)
ament_target_dependencies(lane_candidate_mask_node 
  rclcpp 
  sensor_msgs
  geometry_msgs
  image_transport 
  cv_bridge
  visualization_msgs  # ⭐ 에러의 원인이었던 부분! 여기에 추가해야 적용됩니다.
)
target_link_libraries(lane_candidate_mask_node ${OpenCV_LIBRARIES})

add_executable(centerline_extractor_node src/centerline_extractor_node.cpp)
ament_target_dependencies(centerline_extractor_node 
  rclcpp 
  sensor_msgs
  geometry_msgs
  image_transport 
  cv_bridge
)
target_link_libraries(centerline_extractor_node
  ${OpenCV_LIBRARIES}
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# 4. 설치
install(TARGETS
  undistort_node
  bev_warp_node
  lane_candidate_mask_node
  centerline_extractor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/camera_auto_setup.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# 5. 패키지 마무리
ament_package()
