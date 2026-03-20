#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <cmath>
#include <algorithm>
#include <chrono>

using namespace std;

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("control_node") {
        // [차량 물리 파라미터]
        wheelbase_ = 0.257;      
        max_steer_ = 0.523598;   // 최대 30도 (라디안)
       
        // ⭐ [튜닝 포인트 1] 가변 시야 (Adaptive Lookahead)
        // 빡센 코너(R=0.5m) 파고들기 방지 및 직선 비틀거림 방지 황금 밸런스
        lookahead_min_ = 0.28;   // 축거(0.257)와 비슷하게 설정하여 코너에서 타이트하게 추종
        lookahead_max_ = 1.5;    // 직선에서는 1.5m까지 멀리 봐서 비틀거림 방지
        lookahead_gain_ = 1.2;   // 0.15m/s일 때 시야 약 0.46m 형성
       
        min_speed_ = 0.15;      
        max_speed_ = 0.4;        
       
        current_speed_ = 0.0;    
        smoothed_steer_ = 0.0;

        // 구독 및 발행
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planning/local_path", 10, bind(&ControlNode::path_callback, this, placeholders::_1));
           
        speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/ego_speed", 10, bind(&ControlNode::speed_callback, this, placeholders::_1));
           
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/auto_drive", 10);
        RCLCPP_INFO(this->get_logger(), "🔥 [Control] 비틀거림 방지 & 코너 최적화 Pure Pursuit 가동!");

        // 50Hz 고속 루프 타이머
        timer_ = this->create_wall_timer(
            chrono::milliseconds(20),
            bind(&ControlNode::control_loop, this)
        );
    }

private:
    void speed_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_speed_ = msg->data;
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        latest_path_ = *msg;
    }

    void control_loop() {
        if (latest_path_.poses.empty()) {
            publish_drive(0.0, 0.0);
            return;
        }

        // 1. 다이나믹 시야 계산
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

        // 2. Pure Pursuit 기하학 조향각 계산
        double alpha = atan2(ty, tx);
        double actual_Ld = hypot(tx, ty);
        double steer = 0.0;
       
        if (actual_Ld > 0.01) {
            steer = atan2(2.0 * wheelbase_ * sin(alpha), actual_Ld);
        }
        steer = max(-max_steer_, min(max_steer_, steer));

        // ==========================================================
        // ⭐ [튜닝 포인트 2] 조향 둔감대 (Soft Deadband)
        // 하드웨어 유격 무시 & 경로에 비행기 착륙처럼 스르륵 합류!
        // ==========================================================
        if (abs(steer) < 0.15) { // 약 8.5도 이내로 선에 가까워지면
            steer = steer * 0.5; // 조향각을 절반으로 줄여 댐핑
        }

        // ==========================================================
        // ⭐ [튜닝 포인트 3] 조향 스무딩 필터
        // 서보모터가 요동치지 않고 묵직하게 움직이도록 비율 조정
        // ==========================================================
        smoothed_steer_ = (0.6 * steer) + (0.4 * smoothed_steer_);

        // 3. 코너링 감속 & 최종 속도 결정
        if (abs(target_v) > 0.01) {
            // 핸들이 많이 꺾일수록 기획팀의 목표 속도에서 한 번 더 깎아냄 (최대 60% 감속)
            double cornering_factor = 1.0 - (abs(smoothed_steer_) / max_steer_) * 0.6;
            target_v = target_v * cornering_factor;
        }

        // 속도 하한 및 상한(클리핑)
        if (abs(target_v) > 0.01 && abs(target_v) < min_speed_) {
            target_v = (target_v > 0) ? min_speed_ : -min_speed_;
        } else if (abs(target_v) <= 0.01) {
            target_v = 0.0;
        }
        target_v = max(-max_speed_, min(max_speed_, target_v));

        publish_drive(target_v, smoothed_steer_);
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
    double smoothed_steer_;

    nav_msgs::msg::Path latest_path_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}