#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vector>
#include <cmath>

class UnifiedCameraNode : public rclcpp::Node {
public:
  UnifiedCameraNode() : Node("unified_camera_node") {
    declare_parameter<std::string>("camera_type", "HW40");
    declare_parameter<std::string>("device_path", "/dev/video0");
    declare_parameter<int>("fps", 30);

    std::string cam_type = get_parameter("camera_type").as_string();
    std::string dev_path = get_parameter("device_path").as_string();
    int fps = get_parameter("fps").as_int();

    std::string pkg_share = ament_index_cpp::get_package_share_directory("perception");
    std::string config_path = (cam_type == "HW40") ? 
        pkg_share + "/config/hw40_params.yaml" : pkg_share + "/config/usb2_params.yaml";

    loadParamsAndBuildMap(config_path);

    cap_.open(dev_path, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
        return;
    }

    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1); // ⭐ [추가!] 낡은 사진 버리고 무조건 최신 1장만 가져와!
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, camera_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height_);
    cap_.set(cv::CAP_PROP_FPS, fps);

    pub_raw_ = image_transport::create_publisher(this, "/perception/camera/raw");
    pub_bev_ = image_transport::create_publisher(this, "/perception/bev/image");

    timer_ = create_wall_timer(std::chrono::milliseconds(1000/fps), std::bind(&UnifiedCameraNode::tick, this));
  }

private:
  void loadParamsAndBuildMap(const std::string& file) {
      YAML::Node cfg = YAML::LoadFile(file);
      
      frame_id_ = cfg["camera"]["frame_id"].as<std::string>();
      camera_width_ = cfg["camera"]["width"].as<int>();
      camera_height_ = cfg["camera"]["height"].as<int>();
      
      double fx = cfg["intrinsics"]["fx"].as<double>();
      double fy = cfg["intrinsics"]["fy"].as<double>();
      double cx = cfg["intrinsics"]["cx"].as<double>();
      double cy = cfg["intrinsics"]["cy"].as<double>();
      double k1 = cfg["intrinsics"]["k1"].as<double>();
      double k2 = cfg["intrinsics"]["k2"].as<double>();
      double p1 = cfg["intrinsics"]["p1"].as<double>();
      double p2 = cfg["intrinsics"]["p2"].as<double>();
      double k3 = cfg["intrinsics"]["k3"].as<double>();

      double pitch_deg = cfg["extrinsics"]["pitch_deg"].as<double>();
      double h_m = cfg["extrinsics"]["height_m"].as<double>();
      double pitch_rad = pitch_deg * CV_PI / 180.0;

      double x_min = cfg["bev"]["x_min"].as<double>();
      double x_max = cfg["bev"]["x_max"].as<double>();
      double y_min = cfg["bev"]["y_min"].as<double>();
      double y_max = cfg["bev"]["y_max"].as<double>();
      out_w_ = cfg["bev"]["output_width"].as<int>();
      out_h_ = cfg["bev"]["output_height"].as<int>();

      cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
      cv::Mat dist_coeffs = (cv::Mat_<double>(1,5) << k1, k2, p1, p2, k3);
      cv::Mat map_x, map_y;
      cv::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(), camera_matrix,
          cv::Size(camera_width_, camera_height_), CV_32FC1, map_x, map_y);

      // ⭐ [NEW] 허공 픽셀 번짐을 막기 위한 '완벽한 검은색 방어 마스크' 생성!
      cv::Mat valid_x, valid_y, valid_undistort_mask;
      cv::compare(map_x, 0.0f, valid_x, cv::CMP_GE);
      cv::Mat valid_x2; cv::compare(map_x, (float)camera_width_ - 1.0f, valid_x2, cv::CMP_LE);
      cv::bitwise_and(valid_x, valid_x2, valid_x);

      cv::compare(map_y, 0.0f, valid_y, cv::CMP_GE);
      cv::Mat valid_y2; cv::compare(map_y, (float)camera_height_ - 1.0f, valid_y2, cv::CMP_LE);
      cv::bitwise_and(valid_y, valid_y2, valid_y);
      cv::bitwise_and(valid_x, valid_y, valid_undistort_mask); // 렌즈 안쪽 유효 영역 추출 완료

      std::vector<cv::Point2f> src_pts, dst_pts;
      auto world_to_bev_pixel = [&](double wx, double wy) {
          float px = (float)((wy - y_min) / (y_max - y_min) * out_w_);
          float py = (float)((x_max - wx) / (x_max - x_min) * out_h_);
          return cv::Point2f(px, py);
      };

      std::vector<cv::Point2d> ground_corners = {{x_max, y_min}, {x_max, y_max}, {x_min, y_min}, {x_min, y_max}};
      for (const auto& pt : ground_corners) {
          double cam_x = pt.y;
          double cam_y = -pt.x * sin(pitch_rad) + h_m * cos(pitch_rad);
          double cam_z =  pt.x * cos(pitch_rad) + h_m * sin(pitch_rad);
          float u = (float)((fx * cam_x / cam_z) + cx);
          float v = (float)((fy * cam_y / cam_z) + cy);
          src_pts.push_back(cv::Point2f(u, v));
          dst_pts.push_back(world_to_bev_pixel(pt.x, pt.y));
      }

      cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);

      cv::warpPerspective(map_x, combined_map_x_, M, cv::Size(out_w_, out_h_), cv::INTER_LINEAR);
      cv::warpPerspective(map_y, combined_map_y_, M, cv::Size(out_w_, out_h_), cv::INTER_LINEAR);

      // ⭐ [NEW] 방어 마스크도 탑뷰로 똑같이 구부림! (섞이지 않도록 무조건 INTER_NEAREST 적용)
      cv::warpPerspective(valid_undistort_mask, bev_valid_mask_, M, cv::Size(out_w_, out_h_), cv::INTER_NEAREST);
  }

  void tick() {
      cv::Mat frame;
      if (!cap_.read(frame) || frame.empty()) return;

      std_msgs::msg::Header header;
      header.stamp = now();
      header.frame_id = frame_id_;

      pub_raw_.publish(*cv_bridge::CvImage(header, "bgr8", frame).toImageMsg());

      cv::Mat bev_image;
      cv::remap(frame, bev_image, combined_map_x_, combined_map_y_, cv::INTER_LINEAR);

      // ⭐ [NEW] 마스크를 이용해 허공(낭떠러지)은 무조건 순수한 검은색(0,0,0)으로 강제 밀봉!
      bev_image.setTo(cv::Scalar(0, 0, 0), bev_valid_mask_ == 0);

      pub_bev_.publish(*cv_bridge::CvImage(header, "bgr8", bev_image).toImageMsg());
  }

  cv::VideoCapture cap_;
  cv::Mat combined_map_x_, combined_map_y_, bev_valid_mask_;
  image_transport::Publisher pub_bev_, pub_raw_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;
  int camera_width_, camera_height_, out_w_, out_h_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UnifiedCameraNode>());
  rclcpp::shutdown();
  return 0;
}