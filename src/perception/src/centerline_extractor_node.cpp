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
#include <string>
#include <cmath>
#include <algorithm>

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
    declare_parameter<bool>("publish_debug", true);

    width_ = get_parameter("width").as_int();
    height_ = get_parameter("height").as_int();

    meters_per_pixel_x_ = get_parameter("meters_per_pixel_x").as_double();
    meters_per_pixel_y_ = get_parameter("meters_per_pixel_y").as_double();

    origin_u_ = get_parameter("origin_u").as_double();
    origin_v_ = get_parameter("origin_v").as_double();

    bev_origin_to_camera_x_m_ = get_parameter("bev_origin_to_camera_x_m").as_double();
    bev_origin_to_camera_y_m_ = get_parameter("bev_origin_to_camera_y_m").as_double();

    camera_to_base_x_m_ = get_parameter("camera_to_base_x_m").as_double();
    camera_to_base_y_m_ = get_parameter("camera_to_base_y_m").as_double();

    frame_id_ = get_parameter("frame_id").as_string();
    y_thresh_ = get_parameter("y_thresh").as_double();
    x_thresh_ = get_parameter("x_thresh").as_double();
    min_lane_pts_ = get_parameter("min_lane_pts").as_int();
    publish_debug_ = get_parameter("publish_debug").as_bool();

    sub_mask_ = image_transport::create_subscription(
      this,
      "/perception/lane/mask",
      std::bind(&CenterlineExtractorNode::mask_callback, this, std::placeholders::_1),
      "raw"
    );

    pub_lanes_ = create_publisher<perception::msg::Lanes>("/perception/lane/lanes", 10);

    if (publish_debug_) {
      pub_skeleton_ = image_transport::create_publisher(this, "/perception/lane/skeleton");
    }

    RCLCPP_INFO(this->get_logger(), "[centerline_extractor_node] initialized");
    RCLCPP_INFO(this->get_logger(),
      "metric: mpp_x=%.6f, mpp_y=%.6f, origin=(%.1f, %.1f), bev->cam=(%.3f, %.3f), cam->base=(%.3f, %.3f)",
      meters_per_pixel_x_, meters_per_pixel_y_,
      origin_u_, origin_v_,
      bev_origin_to_camera_x_m_, bev_origin_to_camera_y_m_,
      camera_to_base_x_m_, camera_to_base_y_m_);
  }

private:
  struct PixelPoint
  {
    int u;
    int v;
  };

  static bool inside(int u, int v, int width, int height)
  {
    return (u >= 0 && u < width && v >= 0 && v < height);
  }

  geometry_msgs::msg::Point pixel_to_metric(int u, int v) const
  {
    geometry_msgs::msg::Point p;

    // 1) BEV 픽셀 원점 기준 로컬 거리
    double x_local = (origin_v_ - static_cast<double>(v)) * meters_per_pixel_x_;
    double y_local = (origin_u_ - static_cast<double>(u)) * meters_per_pixel_y_;

    // 2) BEV 원점 -> 카메라
    // 3) 카메라 -> base_link
    p.x = x_local + bev_origin_to_camera_x_m_ + camera_to_base_x_m_;
    p.y = y_local + bev_origin_to_camera_y_m_ + camera_to_base_y_m_;
    p.z = 0.0;

    return p;
  }

  void mask_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    cv::Mat mask = cv_bridge::toCvShare(msg, "mono8")->image;
    if (mask.empty()) return;

    cv::Mat resized_mask;
    if (mask.cols != width_ || mask.rows != height_) {
      cv::resize(mask, resized_mask, cv::Size(width_, height_), 0, 0, cv::INTER_NEAREST);
    } else {
      resized_mask = mask.clone();
    }

    cv::Mat binary;
    cv::threshold(resized_mask, binary, 127, 255, cv::THRESH_BINARY);

    cv::Mat skeleton;
    cv::ximgproc::thinning(binary, skeleton, cv::ximgproc::THINNING_ZHANGSUEN);

    if (publish_debug_) {
      pub_skeleton_.publish(*cv_bridge::CvImage(msg->header, "mono8", skeleton).toImageMsg());
    }

    std::vector<PixelPoint> pixels;
    pixels.reserve(width_ * height_ / 20);

    for (int v = 0; v < skeleton.rows; ++v) {
      const uchar* row_ptr = skeleton.ptr<uchar>(v);
      for (int u = 0; u < skeleton.cols; ++u) {
        if (row_ptr[u] > 0) {
          pixels.push_back({u, v});
        }
      }
    }

    perception::msg::Lanes lanes_msg;
    lanes_msg.header = msg->header;
    lanes_msg.header.frame_id = frame_id_;

    if (pixels.empty()) {
      pub_lanes_->publish(lanes_msg);
      return;
    }

    cv::Mat visited = cv::Mat::zeros(skeleton.size(), CV_8UC1);

    const int du8[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dv8[8] = {-1,-1,-1,  0, 0,  1, 1, 1};

    for (const auto &start : pixels) {
      if (visited.at<uchar>(start.v, start.u) > 0) continue;
      if (skeleton.at<uchar>(start.v, start.u) == 0) continue;

      std::queue<PixelPoint> q;
      std::vector<PixelPoint> component;

      q.push(start);
      visited.at<uchar>(start.v, start.u) = 1;

      while (!q.empty()) {
        PixelPoint cur = q.front();
        q.pop();

        component.push_back(cur);

        for (int k = 0; k < 8; ++k) {
          int nu = cur.u + du8[k];
          int nv = cur.v + dv8[k];

          if (!inside(nu, nv, width_, height_)) continue;
          if (visited.at<uchar>(nv, nu) > 0) continue;
          if (skeleton.at<uchar>(nv, nu) == 0) continue;

          visited.at<uchar>(nv, nu) = 1;
          q.push({nu, nv});
        }
      }

      if (static_cast<int>(component.size()) < min_lane_pts_) {
        continue;
      }

      std::sort(component.begin(), component.end(),
        [](const PixelPoint &a, const PixelPoint &b) {
          if (a.v != b.v) return a.v > b.v;
          return a.u < b.u;
        });

      perception::msg::Lane lane_msg;

      int last_u = -999999;
      int last_v = -999999;
      bool has_last = false;

      for (const auto &px : component) {
        geometry_msgs::msg::Point p = pixel_to_metric(px.u, px.v);

        if (has_last) {
          geometry_msgs::msg::Point prev = pixel_to_metric(last_u, last_v);

          double dx = std::abs(p.x - prev.x);
          double dy = std::abs(p.y - prev.y);

          if (dx > x_thresh_ || dy > y_thresh_) {
            continue;
          }
        }

        lane_msg.points.push_back(p);
        last_u = px.u;
        last_v = px.v;
        has_last = true;
      }

      if (static_cast<int>(lane_msg.points.size()) >= min_lane_pts_) {
        lanes_msg.lanes.push_back(lane_msg);
      }
    }

    std::sort(lanes_msg.lanes.begin(), lanes_msg.lanes.end(),
      [](const perception::msg::Lane &a, const perception::msg::Lane &b) {
        if (a.points.empty() || b.points.empty()) return a.points.size() > b.points.size();
        return a.points.front().y < b.points.front().y;
      });

    pub_lanes_->publish(lanes_msg);
  }

private:
  int width_, height_;

  double meters_per_pixel_x_;
  double meters_per_pixel_y_;

  double origin_u_;
  double origin_v_;

  double bev_origin_to_camera_x_m_;
  double bev_origin_to_camera_y_m_;

  double camera_to_base_x_m_;
  double camera_to_base_y_m_;

  std::string frame_id_;
  double y_thresh_;
  double x_thresh_;
  int min_lane_pts_;
  bool publish_debug_;

  image_transport::Subscriber sub_mask_;
  image_transport::Publisher pub_skeleton_;
  rclcpp::Publisher<perception::msg::Lanes>::SharedPtr pub_lanes_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CenterlineExtractorNode>());
  rclcpp::shutdown();
  return 0;
}
