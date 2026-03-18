#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

class UndistortNode : public rclcpp::Node {
public:
  UndistortNode() : Node("undistort_node") {
    // 런치 파일에서 결정된 카메라 타입과 디바이스 경로를 받음
    declare_parameter<std::string>("camera_type", "HW40");
    declare_parameter<std::string>("device_path", "/dev/video6");
    declare_parameter<int>("fps", 30);

    std::string cam_type = get_parameter("camera_type").as_string();
    std::string dev_path = get_parameter("device_path").as_string();
    int fps = get_parameter("fps").as_int();

    // 패키지 경로를 찾아 모델별 YAML 파일 로드
    std::string pkg_share = ament_index_cpp::get_package_share_directory("perception");
    std::string config_path = (cam_type == "HW40") ? 
        pkg_share + "/config/hw40_params.yaml" : pkg_share + "/config/usb2_params.yaml";

    loadIntrinsics(config_path);
    buildUndistortMap();

    // V4L2로 장치 오픈 (문자열 경로 사용)
    cap_.open(dev_path, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera: %s", dev_path.c_str());
        return;
    }

    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, camera_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height_);
    cap_.set(cv::CAP_PROP_FPS, fps);

    pub_undistorted_ = image_transport::create_publisher(this, "/perception/camera/undistorted");
    pub_raw_ = image_transport::create_publisher(this, "/perception/camera/raw");

    timer_ = create_wall_timer(std::chrono::milliseconds(1000/fps), std::bind(&UndistortNode::tick, this));
  }

private:
  void loadIntrinsics(const std::string& file) {
      YAML::Node cfg = YAML::LoadFile(file);
      frame_id_ = cfg["camera"]["frame_id"].as<std::string>();
      camera_width_ = cfg["camera"]["width"].as<int>();
      camera_height_ = cfg["camera"]["height"].as<int>();
      fx_ = cfg["intrinsics"]["fx"].as<double>();
      fy_ = cfg["intrinsics"]["fy"].as<double>();
      cx_ = cfg["intrinsics"]["cx"].as<double>();
      cy_ = cfg["intrinsics"]["cy"].as<double>();
      k1_ = cfg["intrinsics"]["k1"].as<double>();
      k2_ = cfg["intrinsics"]["k2"].as<double>();
      p1_ = cfg["intrinsics"]["p1"].as<double>();
      p2_ = cfg["intrinsics"]["p2"].as<double>();
      k3_ = cfg["intrinsics"]["k3"].as<double>();
  }

  void buildUndistortMap() {
      camera_matrix_ = (cv::Mat_<double>(3,3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
      dist_coeffs_ = (cv::Mat_<double>(1,5) << k1_, k2_, p1_, p2_, k3_);
      cv::initUndistortRectifyMap(camera_matrix_, dist_coeffs_, cv::Mat(), camera_matrix_,
          cv::Size(camera_width_, camera_height_), CV_32FC1, undistort_map_x_, undistort_map_y_);
  }

  void tick() {
      cv::Mat frame;
      if (!cap_.read(frame) || frame.empty()) return;

      std_msgs::msg::Header header;
      header.stamp = now();
      header.frame_id = frame_id_;

      cv::Mat raw_img;
      cv::resize(frame, raw_img, cv::Size(1280, 960));
      pub_raw_.publish(*cv_bridge::CvImage(header, "bgr8", raw_img).toImageMsg());

      cv::Mat undistorted;
      cv::remap(frame, undistorted, undistort_map_x_, undistort_map_y_, cv::INTER_LINEAR);
      pub_undistorted_.publish(*cv_bridge::CvImage(header, "bgr8", undistorted).toImageMsg());
  }

  cv::VideoCapture cap_;
  cv::Mat camera_matrix_, dist_coeffs_, undistort_map_x_, undistort_map_y_;
  image_transport::Publisher pub_undistorted_, pub_raw_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;
  int camera_width_, camera_height_;
  double fx_, fy_, cx_, cy_, k1_, k2_, p1_, p2_, k3_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UndistortNode>());
  rclcpp::shutdown();
  return 0;
}