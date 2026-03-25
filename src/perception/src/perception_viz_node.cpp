#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "perception/msg/lanes.hpp"

class PerceptionVizNode : public rclcpp::Node
{
public:
  PerceptionVizNode() : Node("perception_viz_node")
  {
    declare_parameter<double>("viz_x_min", 0.0);
    declare_parameter<double>("viz_x_max", 1.8);
    declare_parameter<double>("viz_y_min", -0.7);
    declare_parameter<double>("viz_y_max", 0.7);

    sub_bev_ = image_transport::create_subscription(
      this,
      "/perception/bev/image",
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        latest_bev_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
      },
      "raw"
    );

    sub_mask_ = image_transport::create_subscription(
      this,
      "/perception/lane/mask",
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        latest_mask_ = cv_bridge::toCvShare(msg, "mono8")->image.clone();
      },
      "raw"
    );

    sub_lanes_ = create_subscription<perception::msg::Lanes>(
      "/perception/lane/lanes",
      10,
      std::bind(&PerceptionVizNode::lanes_cb, this, std::placeholders::_1)
    );

    pub_overlay_ = image_transport::create_publisher(this, "/perception/viz/overlay");
    pub_pc2_ = create_publisher<sensor_msgs::msg::PointCloud2>("/perception/viz/pointcloud", 10);
    pub_border_marker_ = create_publisher<visualization_msgs::msg::Marker>("/perception/viz/bev_border", 10);

    RCLCPP_INFO(this->get_logger(), "[perception_viz_node] initialized");
  }

private:
  void lanes_cb(const perception::msg::Lanes::SharedPtr msg)
  {
    // 1) lane points -> PointCloud2
    sensor_msgs::msg::PointCloud2 pc2_msg;
    pc2_msg.header = msg->header;

    sensor_msgs::PointCloud2Modifier modifier(pc2_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    size_t total_points = 0;
    for (const auto& lane : msg->lanes) {
      total_points += lane.points.size();
    }
    modifier.resize(total_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(pc2_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pc2_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pc2_msg, "z");

    for (const auto& lane : msg->lanes) {
      for (const auto& p : lane.points) {
        *iter_x = static_cast<float>(p.x);
        *iter_y = static_cast<float>(p.y);
        *iter_z = static_cast<float>(p.z);
        ++iter_x;
        ++iter_y;
        ++iter_z;
      }
    }

    pub_pc2_->publish(pc2_msg);

    // 2) actual-coordinate border marker
    double x_min = get_parameter("viz_x_min").as_double();
    double x_max = get_parameter("viz_x_max").as_double();
    double y_min = get_parameter("viz_y_min").as_double();
    double y_max = get_parameter("viz_y_max").as_double();

    visualization_msgs::msg::Marker border_marker;
    border_marker.header = msg->header;
    border_marker.ns = "bev_border";
    border_marker.id = 0;
    border_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    border_marker.action = visualization_msgs::msg::Marker::ADD;

    border_marker.scale.x = 0.03;
    border_marker.color.r = 1.0f;
    border_marker.color.g = 0.5f;
    border_marker.color.b = 0.0f;
    border_marker.color.a = 1.0f;
    border_marker.pose.orientation.w = 1.0;

    std::vector<std::pair<double, double>> corners = {
      {x_max, y_min},
      {x_max, y_max},
      {x_min, y_max},
      {x_min, y_min},
      {x_max, y_min}
    };

    for (const auto& c : corners) {
      geometry_msgs::msg::Point p;
      p.x = c.first;
      p.y = c.second;
      p.z = 0.0;
      border_marker.points.push_back(p);
    }

    pub_border_marker_->publish(border_marker);

    // 3) overlay image
    if (!latest_bev_.empty() && !latest_mask_.empty()) {
      cv::Mat bev_resized, mask_resized;

      if (latest_bev_.size() != latest_mask_.size()) {
        bev_resized = latest_bev_.clone();
        cv::resize(latest_mask_, mask_resized, latest_bev_.size(), 0, 0, cv::INTER_NEAREST);
      } else {
        bev_resized = latest_bev_.clone();
        mask_resized = latest_mask_.clone();
      }

      cv::Mat overlay = bev_resized.clone();
      cv::Mat green_mask = cv::Mat::zeros(bev_resized.size(), CV_8UC3);
      green_mask.setTo(cv::Scalar(0, 255, 0), mask_resized);

      cv::addWeighted(overlay, 1.0, green_mask, 0.5, 0.0, overlay);

      pub_overlay_.publish(
        *cv_bridge::CvImage(msg->header, "bgr8", overlay).toImageMsg()
      );
    }
  }

private:
  cv::Mat latest_bev_;
  cv::Mat latest_mask_;

  image_transport::Subscriber sub_bev_;
  image_transport::Subscriber sub_mask_;
  rclcpp::Subscription<perception::msg::Lanes>::SharedPtr sub_lanes_;

  image_transport::Publisher pub_overlay_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc2_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_border_marker_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionVizNode>());
  rclcpp::shutdown();
  return 0;
}
