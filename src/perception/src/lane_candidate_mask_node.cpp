#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <algorithm>
#include <cstdint>

class LaneCandidateMaskNode : public rclcpp::Node {
public:
  LaneCandidateMaskNode() : Node("lane_candidate_mask_node") {
    // ==========================================================
    // 0. 내부 연산용 다운샘플 크기
    // ==========================================================
    declare_parameter<int>("proc_width", 320);
    declare_parameter<int>("proc_height", 240);

    // ==========================================================
    // 1. 흰색 차선 (White) 파라미터
    // ==========================================================
    declare_parameter<int>("strict_white_v_min", 105);
    declare_parameter<int>("loose_white_v_min", 90);
    declare_parameter<int>("white_s_max", 25);

    // ==========================================================
    // 2. 검은 도로 (Black) 파라미터
    // ==========================================================
    declare_parameter<int>("black_v_min", 0);
    declare_parameter<int>("black_v_max", 125);

    // ==========================================================
    // 3. 노란색 차선 (Yellow) 파라미터
    // ==========================================================
    declare_parameter<int>("yellow_h_min", 20);
    declare_parameter<int>("yellow_h_max", 35);
    declare_parameter<int>("yellow_s_min", 70);
    declare_parameter<int>("yellow_s_max", 180);
    declare_parameter<int>("yellow_v_min", 145);
    declare_parameter<int>("yellow_v_max", 255);

    // ==========================================================
    // 4. 형태학적 필터 사이즈 (320 기준)
    // ==========================================================
    declare_parameter<int>("tophat_size", 10);
    declare_parameter<int>("blast_size", 12);
    declare_parameter<int>("noise_eraser_size", 3);

    // ==========================================================
    // 5. 팽창(Fat) 방어벽 & FloodFill 세팅 (320 기준)
    // ==========================================================
    declare_parameter<int>("yellow_fat_radius", 3);
    declare_parameter<int>("black_fat_radius", 5);
    declare_parameter<int>("seed_row_from_bottom", 25);

    // ==========================================================
    // 6. BEV 물리 거리 파라미터 (RViz border marker용)
    // ==========================================================
    declare_parameter<double>("bev_x_min", 0.12);
    declare_parameter<double>("bev_x_max", 2.0);
    declare_parameter<double>("bev_y_min", -0.85);
    declare_parameter<double>("bev_y_max", 0.85);

    // ==========================================================
    // 7. 디버그 이미지 publish 여부
    // ==========================================================
    declare_parameter<bool>("publish_debug", true);

    // ==========================================================
    // Subscriber / Publisher
    // ==========================================================
    sub_ = image_transport::create_subscription(
      this,
      "/perception/bev/image",
      std::bind(&LaneCandidateMaskNode::cb, this, std::placeholders::_1),
      "raw"
    );

    pub_lane_mask_ = image_transport::create_publisher(this, "/perception/lane/mask");

    pub_roi_ = image_transport::create_publisher(this, "/perception/debug/roi");
    pub_fat_black_ = image_transport::create_publisher(this, "/perception/debug/fat_black");
    pub_white_ = image_transport::create_publisher(this, "/perception/debug/white");
    pub_black_ = image_transport::create_publisher(this, "/perception/debug/black");
    pub_yellow_ = image_transport::create_publisher(this, "/perception/debug/yellow");
    pub_overlay_ = image_transport::create_publisher(this, "/perception/debug/overlay");

    pub_loose_white_ = image_transport::create_publisher(this, "/perception/debug/loose_white");
    pub_massive_white_ = image_transport::create_publisher(this, "/perception/debug/massive_white");
    pub_strict_white_ = image_transport::create_publisher(this, "/perception/debug/strict_white");

    pub_border_marker_ =
      this->create_publisher<visualization_msgs::msg::Marker>("/perception/debug/bev_border", 10);

    RCLCPP_INFO(this->get_logger(),
      "[lane_candidate_mask_node] initialized (input=640x480, internal=320x240)");
  }

private:
  static int ensure_odd(int x) {
    if (x < 1) x = 1;
    if (x % 2 == 0) x += 1;
    return x;
  }

  void upsample_mask_to_original(const cv::Mat& src, cv::Mat& dst, int orig_w, int orig_h) {
    if (src.cols != orig_w || src.rows != orig_h) {
      cv::resize(src, dst, cv::Size(orig_w, orig_h), 0, 0, cv::INTER_NEAREST);
    } else {
      dst = src.clone();
    }
  }

  void cb(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    const bool publish_debug = get_parameter("publish_debug").as_bool();

    cv::Mat bev = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (bev.empty()) return;

    const int orig_w = bev.cols;
    const int orig_h = bev.rows;

    const int proc_w = static_cast<int>(get_parameter("proc_width").as_int());
    const int proc_h = static_cast<int>(get_parameter("proc_height").as_int());

    // ==========================================================
    // 0. 내부 처리용 다운샘플
    // ==========================================================
    cv::Mat bev_small;
    if (orig_w != proc_w || orig_h != proc_h) {
      cv::resize(bev, bev_small, cv::Size(proc_w, proc_h), 0, 0, cv::INTER_LINEAR);
    } else {
      bev_small = bev.clone();
    }

    const int h = bev_small.rows;
    const int w = bev_small.cols;

    cv::Mat hsv;
    cv::cvtColor(bev_small, hsv, cv::COLOR_BGR2HSV);

    // ==========================================================
    // 1. 색상 마스크 추출
    // ==========================================================
    cv::Mat loose_white_mask, strict_white_mask, black_mask, yellow_mask;

    cv::inRange(
      hsv,
      cv::Scalar(0, 0, static_cast<int>(get_parameter("loose_white_v_min").as_int())),
      cv::Scalar(180, static_cast<int>(get_parameter("white_s_max").as_int()), 255),
      loose_white_mask
    );

    cv::inRange(
      hsv,
      cv::Scalar(0, 0, static_cast<int>(get_parameter("strict_white_v_min").as_int())),
      cv::Scalar(180, static_cast<int>(get_parameter("white_s_max").as_int()), 255),
      strict_white_mask
    );

    cv::inRange(
      hsv,
      cv::Scalar(0, 0, static_cast<int>(get_parameter("black_v_min").as_int())),
      cv::Scalar(180, 255, static_cast<int>(get_parameter("black_v_max").as_int())),
      black_mask
    );

    cv::inRange(
      hsv,
      cv::Scalar(
        static_cast<int>(get_parameter("yellow_h_min").as_int()),
        static_cast<int>(get_parameter("yellow_s_min").as_int()),
        static_cast<int>(get_parameter("yellow_v_min").as_int())
      ),
      cv::Scalar(
        static_cast<int>(get_parameter("yellow_h_max").as_int()),
        static_cast<int>(get_parameter("yellow_s_max").as_int()),
        static_cast<int>(get_parameter("yellow_v_max").as_int())
      ),
      yellow_mask
    );

    // ==========================================================
    // 2. 흰색 후보 정제
    // ==========================================================
    int t_size = ensure_odd(static_cast<int>(get_parameter("tophat_size").as_int()));
    cv::Mat tophat_kernel =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(t_size, t_size));

    cv::Mat massive_white;
    cv::morphologyEx(loose_white_mask, massive_white, cv::MORPH_OPEN, tophat_kernel);

    int blast_size = ensure_odd(static_cast<int>(get_parameter("blast_size").as_int()));
    cv::Mat blast_kernel =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(blast_size, blast_size));
    cv::dilate(massive_white, massive_white, blast_kernel);

    cv::Mat white_mask;
    cv::subtract(strict_white_mask, massive_white, white_mask);

    int eraser_size = ensure_odd(static_cast<int>(get_parameter("noise_eraser_size").as_int()));
    if (eraser_size > 1) {
      cv::Mat eraser_kernel =
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(eraser_size, eraser_size));
      cv::morphologyEx(white_mask, white_mask, cv::MORPH_OPEN, eraser_kernel);
    }

    // ==========================================================
    // 3. 팽창 방어벽
    // ==========================================================
    cv::Mat fat_yellow;
    int y_r = static_cast<int>(get_parameter("yellow_fat_radius").as_int());
    y_r = std::max(0, y_r);
    int y_size = std::max(1, y_r * 2 + 1);
    cv::Mat yellow_kernel =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(y_size, y_size));
    cv::dilate(yellow_mask, fat_yellow, yellow_kernel);

    cv::Mat fat_black;
    int b_r = static_cast<int>(get_parameter("black_fat_radius").as_int());
    b_r = std::max(0, b_r);
    int b_size_black = std::max(1, b_r * 2 + 1);
    cv::Mat black_kernel =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(b_size_black, b_size_black));
    cv::dilate(black_mask, fat_black, black_kernel);

    cv::Mat expandable = fat_black.clone();

    // yellow는 flood fill 못 가도록 막음
    expandable.setTo(0, fat_yellow);

    // 가장자리 새는 것 방지
    cv::rectangle(expandable, cv::Point(0, 0), cv::Point(w - 1, h - 1), cv::Scalar(0), 3);

    // ==========================================================
    // 4. 유효한 BEV 영역만 사용
    // ==========================================================
    cv::Mat bev_gray, bev_valid_mask;
    cv::cvtColor(bev_small, bev_gray, cv::COLOR_BGR2GRAY);
    cv::threshold(bev_gray, bev_valid_mask, 0, 255, cv::THRESH_BINARY);

    cv::bitwise_and(expandable, bev_valid_mask, expandable);

    // ==========================================================
    // 5. BEV 테두리 마커 생성
    // ==========================================================
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bev_valid_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
      auto largest_contour = *std::max_element(
        contours.begin(), contours.end(),
        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
          return cv::contourArea(a) < cv::contourArea(b);
        });

      visualization_msgs::msg::Marker border_marker;
      border_marker.header.frame_id = "base_link";
      border_marker.header.stamp = msg->header.stamp;
      border_marker.ns = "bev_border";
      border_marker.id = 0;
      border_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      border_marker.action = visualization_msgs::msg::Marker::ADD;
      border_marker.scale.x = 0.05;
      border_marker.color.r = 1.0f;
      border_marker.color.g = 0.5f;
      border_marker.color.b = 0.0f;
      border_marker.color.a = 1.0f;
      border_marker.lifetime = rclcpp::Duration::from_seconds(0.0);

      double bev_x_min = get_parameter("bev_x_min").as_double();
      double bev_x_max = get_parameter("bev_x_max").as_double();
      double bev_y_min = get_parameter("bev_y_min").as_double();
      double bev_y_max = get_parameter("bev_y_max").as_double();

      for (const auto& pt : largest_contour) {
        geometry_msgs::msg::Point p;
        p.x = bev_x_max - (static_cast<double>(pt.y) / h) * (bev_x_max - bev_x_min);
        p.y = bev_y_max - (static_cast<double>(pt.x) / w) * (bev_y_max - bev_y_min);
        p.z = 0.0;
        border_marker.points.push_back(p);
      }

      if (!largest_contour.empty()) {
        geometry_msgs::msg::Point p;
        p.x = bev_x_max - (static_cast<double>(largest_contour[0].y) / h) * (bev_x_max - bev_x_min);
        p.y = bev_y_max - (static_cast<double>(largest_contour[0].x) / w) * (bev_y_max - bev_y_min);
        p.z = 0.0;
        border_marker.points.push_back(p);
      }

      pub_border_marker_->publish(border_marker);
    }

    // ==========================================================
    // 6. FloodFill 기반 ROI 추출
    // ==========================================================
    cv::Mat roi = cv::Mat::zeros(h, w, CV_8UC1);

    int seed_x = w / 2;
    int seed_row_from_bottom = static_cast<int>(get_parameter("seed_row_from_bottom").as_int());
    seed_row_from_bottom = std::max(1, seed_row_from_bottom);
    int seed_y = std::max(0, h - seed_row_from_bottom);

    bool found = false;
    for (int y = h - 1; y > h / 2; --y) {
      if (expandable.at<uchar>(y, seed_x) > 0) {
        seed_y = y;
        found = true;
        break;
      }
    }

    if (found) {
      cv::Mat flood_mask = cv::Mat::zeros(h + 2, w + 2, CV_8UC1);

      // floodFill는 입력 영상을 바꾸므로 복사본 사용
      cv::Mat expandable_copy = expandable.clone();

      cv::floodFill(
        expandable_copy,
        flood_mask,
        cv::Point(seed_x, seed_y),
        cv::Scalar(255),
        nullptr,
        cv::Scalar(0),
        cv::Scalar(0),
        8 | (255 << 8) | cv::FLOODFILL_MASK_ONLY
      );

      roi = flood_mask(cv::Rect(1, 1, w, h)).clone();
    }

    // ==========================================================
    // 7. 최종 lane mask
    // ==========================================================
    cv::Mat lane_final;
    cv::bitwise_and(white_mask, roi, lane_final);

    // 끊긴 차선 살짝 연결
    int final_fat_size = 3;
    cv::Mat final_kernel =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(final_fat_size, final_fat_size));
    cv::dilate(lane_final, lane_final, final_kernel);

    // ==========================================================
    // 8. 다시 원래 해상도(640x480)로 복원
    // ==========================================================
    cv::Mat lane_final_up, roi_up, expandable_up, white_up, black_up, yellow_up;
    cv::Mat loose_white_up, massive_white_up, strict_white_up;

    upsample_mask_to_original(lane_final, lane_final_up, orig_w, orig_h);
    upsample_mask_to_original(roi, roi_up, orig_w, orig_h);
    upsample_mask_to_original(expandable, expandable_up, orig_w, orig_h);
    upsample_mask_to_original(white_mask, white_up, orig_w, orig_h);
    upsample_mask_to_original(black_mask, black_up, orig_w, orig_h);
    upsample_mask_to_original(fat_yellow, yellow_up, orig_w, orig_h);

    upsample_mask_to_original(loose_white_mask, loose_white_up, orig_w, orig_h);
    upsample_mask_to_original(massive_white, massive_white_up, orig_w, orig_h);
    upsample_mask_to_original(strict_white_mask, strict_white_up, orig_w, orig_h);

    // ==========================================================
    // 9. Publish
    // ==========================================================
    auto h_msg = msg->header;

    // centerline이 받는 최종 mask는 640x480
    pub_lane_mask_.publish(*cv_bridge::CvImage(h_msg, "mono8", lane_final_up).toImageMsg());

    if (publish_debug) {
      pub_roi_.publish(*cv_bridge::CvImage(h_msg, "mono8", roi_up).toImageMsg());
      pub_white_.publish(*cv_bridge::CvImage(h_msg, "mono8", white_up).toImageMsg());
      pub_black_.publish(*cv_bridge::CvImage(h_msg, "mono8", black_up).toImageMsg());
      pub_fat_black_.publish(*cv_bridge::CvImage(h_msg, "mono8", expandable_up).toImageMsg());
      pub_yellow_.publish(*cv_bridge::CvImage(h_msg, "mono8", yellow_up).toImageMsg());

      pub_loose_white_.publish(*cv_bridge::CvImage(h_msg, "mono8", loose_white_up).toImageMsg());
      pub_massive_white_.publish(*cv_bridge::CvImage(h_msg, "mono8", massive_white_up).toImageMsg());
      pub_strict_white_.publish(*cv_bridge::CvImage(h_msg, "mono8", strict_white_up).toImageMsg());

      cv::Mat overlay = bev.clone();
      overlay.setTo(cv::Scalar(255, 0, 0), roi_up);         // ROI = 파랑
      overlay.setTo(cv::Scalar(0, 0, 255), yellow_up);      // fat yellow = 빨강
      overlay.setTo(cv::Scalar(0, 255, 0), lane_final_up);  // final lane = 초록

      pub_overlay_.publish(*cv_bridge::CvImage(h_msg, "bgr8", overlay).toImageMsg());
    }
  }

private:
  image_transport::Subscriber sub_;

  image_transport::Publisher pub_lane_mask_;
  image_transport::Publisher pub_roi_;
  image_transport::Publisher pub_fat_black_;
  image_transport::Publisher pub_white_;
  image_transport::Publisher pub_black_;
  image_transport::Publisher pub_yellow_;
  image_transport::Publisher pub_overlay_;

  image_transport::Publisher pub_loose_white_;
  image_transport::Publisher pub_massive_white_;
  image_transport::Publisher pub_strict_white_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_border_marker_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneCandidateMaskNode>());
  rclcpp::shutdown();
  return 0;
}
