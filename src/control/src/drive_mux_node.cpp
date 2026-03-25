#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath> // M_PI 사용을 위해 추가

class DriveMuxNode : public rclcpp::Node {
public:
    DriveMuxNode() : Node("drive_mux_node") {
        // 모든 통신 큐 사이즈를 1로 설정 (과거 데이터 폐기)
        joy_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/joy_drive", 1, std::bind(&DriveMuxNode::joyCallback, this, std::placeholders::_1));

        auto_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/auto_drive", 1, std::bind(&DriveMuxNode::autoCallback, this, std::placeholders::_1));

        mode_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/drive_mode", 1, std::bind(&DriveMuxNode::modeCallback, this, std::placeholders::_1));

        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);
    }

private:
    bool mode_auto_ = false;

    // ⭐ [추가] 직진 영점 조절 (Trim) 파라미터 (단위: Degree)
    // 기존 아두이노에 있던 STEERING_TRIM_DEG 값을 여기에 그대로 설정하세요.
    const double steering_trim_deg_ = 2.8; 

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr joy_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr auto_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    void modeCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        mode_auto_ = msg->data;
    }

    // ⭐ [추가] Trim 값을 씌워서 퍼블리시하는 통합 헬퍼 함수
    void publishWithTrim(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
        auto trimmed_msg = *msg; // 원본 메시지 복사
        
        // Degree(도)를 Radian(라디안)으로 변환
        double trim_rad = steering_trim_deg_ * (M_PI / 180.0);
        
        // 기존 조향각에 Trim 라디안 값을 보정
        // (아두이노의 수식 계산과 동일한 물리적 움직임을 내기 위해 빼줍니다)
        trimmed_msg.drive.steering_angle -= trim_rad; 
        
        // 최종 보정된 명령을 하위(Serial 브릿지)로 발사
        drive_pub_->publish(trimmed_msg);
    }

    void joyCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
        if (!mode_auto_) {
            publishWithTrim(msg); // 조이스틱 모드일 때도 영점 보정 적용
        }
    }

    void autoCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
        if (mode_auto_) {
            publishWithTrim(msg); // 자율주행 모드일 때도 영점 보정 적용
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveMuxNode>());
    rclcpp::shutdown();
    return 0;
}
