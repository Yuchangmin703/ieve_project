#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "perception/msg/lane.hpp"
#include "perception/msg/lanes.hpp"

#include <queue>
#include <vector>
#include <cmath>

class CenterlineExtractorNode : public rclcpp::Node
{
public:
  CenterlineExtractorNode() : Node("centerline_extractor_node")
  {
    declare_parameter<int>("width", 640);
    declare_parameter<int>("height", 480);
    declare_parameter<double>("meters_per_pixel_x", 0.0050);
    declare_parameter<double>("meters_per_pixel_y", 0.0030);
    declare_parameter<double>("origin_u", 320.0);
    declare_parameter<double>("origin_v", 383.0);
    declare_parameter<double>("bev_origin_to_camera_x_m", 0.0);
    declare_parameter<double>("bev_origin_to_camera_y_m", 0.0);
    declare_parameter<double>("camera_to_base_x_m", 0.0);
    declare_parameter<double>("camera_to_base_y_m", 0.0);
    declare_parameter<std::string>("frame_id", "base_link");
    declare_parameter<double>("y_thresh", 0.25);
    declare_parameter<double>("x_thresh", 0.60);
    declare_parameter<int>("min_lane_pts", 30);

    int cfg_w = get_parameter("width").as_int();
    int cfg_h = get_parameter("height").as_int();
    double cfg_mpp_x = get_parameter("meters_per_pixel_x").as_double();
    double cfg_mpp_y = get_parameter("meters_per_pixel_y").as_double();
    double cfg_origin_u = get_parameter("origin_u").as_double();
    double cfg_origin_v = get_parameter("origin_v").as_double();

    // lane_mask가 320×240으로 출력하므로 그 해상도에서 직접 thinning
    // metric 파라미터를 비례 스케일하여 동일한 미터 좌표 출력 보장
    //   pixel(u,v) at half-res ↔ pixel(2u,2v) at full-res
    //   (origin_v - 2v) * mpp_x = (origin_v/2 - v) * (mpp_x*2)  ← 수학적 동치
    width_  = 320;
    height_ = 240;
    double scale_x = static_cast<double>(cfg_w) / width_;
    double scale_y = static_cast<double>(cfg_h) / height_;
    mpp_x_ = cfg_mpp_x * scale_x;
    mpp_y_ = cfg_mpp_y * scale_y;
    origin_u_ = cfg_origin_u / scale_x;
    origin_v_ = cfg_origin_v / scale_y;
    frame_id_ = get_parameter("frame_id").as_string();
    y_thresh_ = get_parameter("y_thresh").as_double();
    x_thresh_ = get_parameter("x_thresh").as_double();
    min_lane_pts_ = get_parameter("min_lane_pts").as_int();

    offset_x_ = get_parameter("bev_origin_to_camera_x_m").as_double()
               + get_parameter("camera_to_base_x_m").as_double();
    offset_y_ = get_parameter("bev_origin_to_camera_y_m").as_double()
               + get_parameter("camera_to_base_y_m").as_double();

    morph_kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    // 버퍼 사전 할당 (320×240 해상도)
    resize_buf_  = cv::Mat(height_, width_, CV_8UC1);
    binary_buf_  = cv::Mat(height_, width_, CV_8UC1);
    morph_buf_   = cv::Mat(height_, width_, CV_8UC1);
    skeleton_buf_= cv::Mat(height_, width_, CV_8UC1);
    visited_buf_ = cv::Mat::zeros(height_, width_, CV_8UC1);

    sub_mask_ = image_transport::create_subscription(
      this, "/perception/lane/mask",
      std::bind(&CenterlineExtractorNode::mask_callback, this, std::placeholders::_1),
      "raw"
    );
    pub_lanes_ = create_publisher<perception::msg::Lanes>("/perception/lane/lanes", 10);

    RCLCPP_INFO(this->get_logger(),
      "[centerline_extractor] thinning at %dx%d (cfg %dx%d), mpp=(%.6f, %.6f), origin=(%.1f, %.1f)",
      width_, height_, cfg_w, cfg_h, mpp_x_, mpp_y_, origin_u_, origin_v_);
  }

private:
  struct PixelPoint { int u, v; };

  inline void pixel_to_metric(int u, int v, double &ox, double &oy) const {
    ox = (origin_v_ - v) * mpp_x_ + offset_x_;
    oy = (origin_u_ - u) * mpp_y_ + offset_y_;
  }

  void mask_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    cv::Mat mask = cv_bridge::toCvShare(msg, "mono8")->image;
    if (mask.empty()) return;

    // 320×240에서 직접 처리 (640→320 업스케일 제거로 thinning 5배 가속)
    if (mask.cols != width_ || mask.rows != height_) {
      cv::resize(mask, resize_buf_, cv::Size(width_, height_), 0, 0, cv::INTER_NEAREST);
      cv::threshold(resize_buf_, binary_buf_, 127, 255, cv::THRESH_BINARY);
    } else {
      cv::threshold(mask, binary_buf_, 127, 255, cv::THRESH_BINARY);
    }

    // 스켈레톤 전 모폴로지 전처리
    cv::morphologyEx(binary_buf_, morph_buf_, cv::MORPH_CLOSE, morph_kernel_);
    cv::morphologyEx(morph_buf_, morph_buf_, cv::MORPH_OPEN, morph_kernel_);

    // thinning: 전체 파이프라인의 병목 (~15-25ms @ 640×480)
    cv::ximgproc::thinning(morph_buf_, skeleton_buf_, cv::ximgproc::THINNING_ZHANGSUEN);

    // findNonZero로 스켈레톤 픽셀 수집 (SIMD 가속, 수동 루프 대비 ~2배 빠름)
    nz_points_.clear();
    cv::findNonZero(skeleton_buf_, nz_points_);

    perception::msg::Lanes lanes_msg;
    lanes_msg.header = msg->header;
    lanes_msg.header.frame_id = frame_id_;

    if (nz_points_.empty()) {
      pub_lanes_->publish(lanes_msg);
      return;
    }

    // visited 버퍼 재사용 (Mat::zeros 재할당 대신 memset)
    visited_buf_.setTo(0);

    static const int du8[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static const int dv8[8] = {-1,-1,-1,  0, 0,  1, 1, 1};

    for (const auto &pt : nz_points_) {
      int su = pt.x, sv = pt.y;
      if (visited_buf_.ptr<uchar>(sv)[su] > 0) continue;

      component_.clear();
      queue_.push({su, sv});
      visited_buf_.ptr<uchar>(sv)[su] = 1;

      while (!queue_.empty()) {
        PixelPoint cur = queue_.front();
        queue_.pop();
        component_.push_back(cur);

        // 행 포인터 캐싱으로 .at() 대비 ~30% 빠른 접근
        for (int k = 0; k < 8; ++k) {
          int nu = cur.u + du8[k];
          int nv = cur.v + dv8[k];
          if (static_cast<unsigned>(nu) >= static_cast<unsigned>(width_) ||
              static_cast<unsigned>(nv) >= static_cast<unsigned>(height_)) continue;
          uchar* vis_row = visited_buf_.ptr<uchar>(nv);
          if (vis_row[nu] > 0) continue;
          if (skeleton_buf_.ptr<uchar>(nv)[nu] == 0) continue;
          vis_row[nu] = 1;
          queue_.push({nu, nv});
        }
      }

      if (static_cast<int>(component_.size()) < min_lane_pts_) continue;

      std::sort(component_.begin(), component_.end(),
        [](const PixelPoint &a, const PixelPoint &b) {
          return (a.v != b.v) ? (a.v > b.v) : (a.u < b.u);
        });

      // 5점 이동평균 스무딩
      if (component_.size() >= 5) {
        smoothed_ = component_;
        for (size_t k = 2; k < component_.size() - 2; k++) {
          smoothed_[k].u = (component_[k-2].u + component_[k-1].u
                           + component_[k].u
                           + component_[k+1].u + component_[k+2].u) / 5;
        }
        component_.swap(smoothed_);
      }

      perception::msg::Lane lane_msg;
      double prev_mx = 0.0, prev_my = 0.0;
      bool has_prev = false;

      for (const auto &px : component_) {
        double mx, my;
        pixel_to_metric(px.u, px.v, mx, my);

        if (has_prev) {
          if (std::abs(mx - prev_mx) > x_thresh_ || std::abs(my - prev_my) > y_thresh_)
            continue;
        }

        geometry_msgs::msg::Point p;
        p.x = mx; p.y = my; p.z = 0.0;
        lane_msg.points.push_back(p);
        prev_mx = mx;
        prev_my = my;
        has_prev = true;
      }

      if (static_cast<int>(lane_msg.points.size()) >= min_lane_pts_)
        lanes_msg.lanes.push_back(std::move(lane_msg));
    }

    std::sort(lanes_msg.lanes.begin(), lanes_msg.lanes.end(),
      [](const perception::msg::Lane &a, const perception::msg::Lane &b) {
        if (a.points.empty() || b.points.empty()) return a.points.size() > b.points.size();
        return a.points.front().y < b.points.front().y;
      });

    pub_lanes_->publish(lanes_msg);
  }

  int width_, height_;
  double mpp_x_, mpp_y_;
  double origin_u_, origin_v_;
  double offset_x_, offset_y_;
  std::string frame_id_;
  double y_thresh_, x_thresh_;
  int min_lane_pts_;

  cv::Mat morph_kernel_;
  cv::Mat resize_buf_, binary_buf_, morph_buf_, skeleton_buf_, visited_buf_;
  std::vector<cv::Point> nz_points_;
  std::vector<PixelPoint> component_;
  std::vector<PixelPoint> smoothed_;
  std::queue<PixelPoint> queue_;

  image_transport::Subscriber sub_mask_;
  rclcpp::Publisher<perception::msg::Lanes>::SharedPtr pub_lanes_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CenterlineExtractorNode>());
  rclcpp::shutdown();
  return 0;
}
