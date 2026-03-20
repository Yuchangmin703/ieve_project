#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

class JoyDriveNode : public rclcpp::Node {
public:
    JoyDriveNode() : Node("joy_drive_node") {
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyDriveNode::joyCallback, this, std::placeholders::_1));
        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/joy_drive", 10);
        mode_pub_ = create_publisher<std_msgs::msg::Bool>("/drive_mode", 10);
    }

private:
    bool mode_auto_ = false;
    int prev_lb_ = 0;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_pub_;

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // C2: 조이스틱 버튼/축 배열 크기 미달 시 크래시 방지
        if (msg->buttons.size() <= 4 || msg->axes.size() <= 3) return;
        int lb = msg->buttons[4];  // F710 기준
        
        if (lb == 1 && prev_lb_ == 0) {
            mode_auto_ = !mode_auto_;
            std_msgs::msg::Bool mode_msg;
            mode_msg.data = mode_auto_;
            mode_pub_->publish(mode_msg);
            RCLCPP_INFO(get_logger(), "Mode Changed → %s", mode_auto_ ? "AUTO" : "MANUAL");
        }
        prev_lb_ = lb;

        // ⭐ 수정: 최대 출력을 0.4m/s로 매핑하여 미세 조종 가능하도록 함
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.drive.speed = msg->axes[1] * 0.4;   
        drive_msg.drive.steering_angle = msg->axes[3] * 0.4;
        drive_pub_->publish(drive_msg);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyDriveNode>());
    rclcpp::shutdown();
    return 0;
}
