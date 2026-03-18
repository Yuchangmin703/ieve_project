#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

class DriveMuxNode : public rclcpp::Node {
public:
    DriveMuxNode() : Node("drive_mux_node") {

        joy_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/joy_drive", 10,
            std::bind(&DriveMuxNode::joyCallback, this, std::placeholders::_1));

        auto_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/auto_drive", 10,
            std::bind(&DriveMuxNode::autoCallback, this, std::placeholders::_1));

        mode_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/drive_mode", 10,
            std::bind(&DriveMuxNode::modeCallback, this, std::placeholders::_1));

        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);
    }

private:
    bool mode_auto_ = false;

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr joy_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr auto_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    void modeCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        mode_auto_ = msg->data;
    }

    void joyCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
        if (!mode_auto_) drive_pub_->publish(*msg);
    }

    void autoCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
        if (mode_auto_) drive_pub_->publish(*msg);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveMuxNode>());
    rclcpp::shutdown();
    return 0;
}
