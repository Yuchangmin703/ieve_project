cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# ⭐ 수정 1: cv::VideoCapture 사용을 위해 OpenCV COMPONENTS에 'videoio' 추가!
find_package(OpenCV REQUIRED COMPONENTS core imgproc ximgproc videoio)
find_package(yaml-cpp REQUIRED)

# ⭐ 수정 2: ament_index_cpp 패키지 추가!
find_package(ament_index_cpp REQUIRED) 

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Lane.msg"
  "msg/Lanes.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(unified_camera_node src/unified_camera_node.cpp)
ament_target_dependencies(unified_camera_node rclcpp sensor_msgs image_transport cv_bridge ament_index_cpp)
target_link_libraries(unified_camera_node ${OpenCV_LIBRARIES} yaml-cpp)

add_executable(lane_candidate_mask_node src/lane_candidate_mask_node.cpp)
ament_target_dependencies(lane_candidate_mask_node rclcpp sensor_msgs image_transport cv_bridge)
target_link_libraries(lane_candidate_mask_node ${OpenCV_LIBRARIES})

add_executable(centerline_extractor_node src/centerline_extractor_node.cpp)
ament_target_dependencies(centerline_extractor_node rclcpp sensor_msgs geometry_msgs image_transport cv_bridge)
target_link_libraries(centerline_extractor_node ${OpenCV_LIBRARIES} ${PROJECT_NAME}__rosidl_typesupport_cpp)

add_executable(perception_viz_node src/perception_viz_node.cpp)
ament_target_dependencies(perception_viz_node rclcpp sensor_msgs)
target_link_libraries(perception_viz_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

install(TARGETS
  unified_camera_node
  lane_candidate_mask_node
  centerline_extractor_node
  perception_viz_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/camera_auto_setup.sh DESTINATION share/${PROJECT_NAME}/scripts)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
