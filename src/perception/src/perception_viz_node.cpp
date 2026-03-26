#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "perception/msg/lanes.hpp"

class PerceptionVizNode : public rclcpp::Node
{
public:
  PerceptionVizNode() : Node("perception_viz_node")
  {
    sub_lanes_ = create_subscription<perception::msg::Lanes>(
      "/perception/lane/lanes",
      10,
      std::bind(&PerceptionVizNode::lanes_cb, this, std::placeholders::_1)
    );

    pub_pc2_ = create_publisher<sensor_msgs::msg::PointCloud2>("/perception/viz/pointcloud", 10);

    RCLCPP_INFO(this->get_logger(), "[perception_viz_node] initialized (PointCloud2 only)");
  }

private:
  void lanes_cb(const perception::msg::Lanes::SharedPtr msg)
  {
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
  }

private:
  rclcpp::Subscription<perception::msg::Lanes>::SharedPtr sub_lanes_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc2_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionVizNode>());
  rclcpp::shutdown();
  return 0;
}
