#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <cmath>
#include <algorithm>
#include <chrono> // 타이머 사용을 위한 헤더

using namespace std;

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("control_node") {
        // [설정] 차량 물리 파라미터
        wheelbase_ = 0.257;      // TT-02D 정확한 축간 거리 (m)
        max_steer_ = 0.523598;   // ⭐ 최대 조향각 (30도를 라디안으로 정확히 변환)
        
        // ⭐ 가변 전방 주시 (Adaptive Lookahead) 파라미터
        lookahead_min_ = 0.3;    
        lookahead_max_ = 1.0;    
        lookahead_gain_ = 0.8;   
        
        min_speed_ = 0.15;       
        max_speed_ = 0.4;        
        current_speed_ = 0.0;    

        // 구독 및 발행
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planning/local_path", 10, bind(&ControlNode::path_callback, this, placeholders::_1));
            
        speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/ego_speed", 10, bind(&ControlNode::speed_callback, this, placeholders::_1));
            
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/auto_drive", 10);

        // ⭐ 핵심: 경로 수신과 상관없이 50Hz(20ms) 주기로 조향을 즉각 계산하는 고속 타이머!
        timer_ = this->create_wall_timer(
            chrono::milliseconds(20), 
            bind(&ControlNode::control_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "🚀 [Control] 50Hz 고속 타이머 Adaptive Pure Pursuit 실행 중!");
    }

private:
    void speed_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_speed_ = msg->data;
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        // ⭐ 경로가 들어오면 연산하지 않고 최신 데이터로 저장만 해둡니다. (병목 제거)
        latest_path_ = *msg;
    }

    // ⭐ 타이머에 의해 0.02초마다 무조건 실행되는 고속 반응 제어 루프
    void control_loop() {
        if (latest_path_.poses.empty()) { 
            publish_drive(0.0, 0.0); 
            return; 
        }

        double dynamic_lookahead = lookahead_min_ + (lookahead_gain_ * abs(current_speed_));
        dynamic_lookahead = max(lookahead_min_, min(lookahead_max_, dynamic_lookahead));

        int target_idx = 0;
        
        for (size_t i = 0; i < latest_path_.poses.size(); ++i) {
            double tx = latest_path_.poses[i].pose.position.x;
            double ty = latest_path_.poses[i].pose.position.y;
            double dist = hypot(tx, ty); 
            
            if (dist >= dynamic_lookahead) {
                target_idx = i;
                break;
            }
        }
        
        if (target_idx == 0 && latest_path_.poses.size() > 1) {
            target_idx = latest_path_.poses.size() - 1;
        }

        double tx = latest_path_.poses[target_idx].pose.position.x;
        double ty = latest_path_.poses[target_idx].pose.position.y;
        double target_v = latest_path_.poses[target_idx].pose.position.z;

        double alpha = atan2(ty, tx);
        double actual_Ld = hypot(tx, ty); 
        double steer = 0.0;
        
        if (actual_Ld > 0.01) {
            steer = atan2(2.0 * wheelbase_ * sin(alpha), actual_Ld);
        }
        steer = max(-max_steer_, min(max_steer_, steer)); 

        if (abs(target_v) > 0.01) {
            double cornering_factor = 1.0 - (abs(steer) / max_steer_) * 0.4;
            target_v = target_v * cornering_factor;
        }

        if (abs(target_v) > 0.01 && abs(target_v) < min_speed_) {
            target_v = (target_v > 0) ? min_speed_ : -min_speed_;
        } else if (abs(target_v) <= 0.01) {
            target_v = 0.0;
        }
        target_v = max(-max_speed_, min(max_speed_, target_v));

        publish_drive(target_v, steer);
    }

    void publish_drive(double v, double delta) {
        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.drive.speed = (float)v;
        msg.drive.steering_angle = (float)delta;
        drive_pub_->publish(msg);
    }

    double wheelbase_, max_steer_;
    double lookahead_min_, lookahead_max_, lookahead_gain_; 
    double min_speed_, max_speed_; 
    double current_speed_;
    
    nav_msgs::msg::Path latest_path_; // ⭐ 최신 경로 저장 변수 추가

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::TimerBase::SharedPtr timer_; // ⭐ 50Hz 루프용 타이머 추가
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
