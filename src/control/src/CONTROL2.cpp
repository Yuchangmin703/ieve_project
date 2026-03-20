#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp> // ⭐ 시각화 메시지 추가
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
        lookahead_min_ = 0.28;   // 축거(0.257)와 비슷하게 설정하여 코너에서 타이트하게 추종
        lookahead_max_ = 1.5;    // 직선에서는 1.5m까지 멀리 봐서 비틀거림 방지
        lookahead_gain_ = 1.2;   // 0.15m/s일 때 시야 약 0.46m 형성
       
        min_speed_ = 0.15;      
        max_speed_ = 0.4;        // ⭐ 제어기의 절대 한계 속도
       
        current_speed_ = 0.0;    
        smoothed_steer_ = 0.0;
        last_serial_pub_time_ = this->get_clock()->now(); // Serial 타이머 초기화

        // 구독 및 발행
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planning/local_path", 10, bind(&ControlNode::path_callback, this, placeholders::_1));
           
        speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/ego_speed", 10, bind(&ControlNode::speed_callback, this, placeholders::_1));
           
        // 하드웨어로 나가는 Serial 토픽 (속도 제한 필요)
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/auto_drive", 10);
       
        // ⭐ [시각화 추가] 목표점 토픽
        target_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/target_point", 10);

        RCLCPP_INFO(this->get_logger(), "🚀 [Control] 시각화 및 Serial 부하 방지 로직 적용 완료!");

        // 50Hz 고속 루프 타이머 (계산은 여전히 빠르게 함)
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
            publish_throtte_drive(0.0, 0.0); // 멈춤 명령도 Throttling 필요
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
       
        // ==========================================================
        // ⭐ [수정] 속도 채택 로직: Planning z값 vs 제어기 max_speed_ 중 Min 선택
        // ==========================================================
        double raw_target_v = latest_path_.poses[target_idx].pose.position.z;
        double final_target_v = std::clamp(raw_target_v, -max_speed_, max_speed_); // 명시적으로 절대 한계 속도로 Clamping

        // ⭐ [시각화 실행] RViz2에 목표점 발행
        publish_target_marker(tx, ty);

        // 2. Pure Pursuit 기하학 조향각 계산
        double alpha = atan2(ty, tx);
        double actual_Ld = hypot(tx, ty);
        double steer = 0.0;
       
        if (actual_Ld > 0.01) {
            steer = atan2(2.0 * wheelbase_ * sin(alpha), actual_Ld);
        }
        steer = max(-max_steer_, min(max_steer_, steer));

        // 조향 둔감대 (Soft Deadband) - 비틀거림 방지
        if (abs(steer) < 0.15) {
            steer = steer * 0.5;
        }

        // 조향 스무딩 필터
        smoothed_steer_ = (0.6 * steer) + (0.4 * smoothed_steer_);

        // 3. 코너링 감속 & 최종 속도 결정
        if (abs(final_target_v) > 0.01) {
            double cornering_factor = 1.0 - (abs(smoothed_steer_) / max_steer_) * 0.6;
            final_target_v = final_target_v * cornering_factor;
        }

        // 속도 하한 클리핑
        if (abs(final_target_v) > 0.01 && abs(final_target_v) < min_speed_) {
            final_target_v = (final_target_v > 0) ? min_speed_ : -min_speed_;
        } else if (abs(final_target_v) <= 0.01) {
            final_target_v = 0.0;
        }

        // ⭐ [수정] Serial 부하를 방지하기 위해 속도가 제한된 발행 함수 호출
        publish_throtte_drive(final_target_v, smoothed_steer_);
    }

    // ⭐ [추가] Serial Serial 출력 주행 속도 제한 함수 (Serial Throttling)
    void publish_throtte_drive(double v, double delta) {
        rclcpp::Time now = this->get_clock()->now();
        // 마지막 발행 시간으로부터 50ms(20Hz)가 지나지 않았으면 토픽 발행을 무시합니다.
        // 조향이 연속적으로 꺾여도 Serial 버퍼가 꽉 막히는 것을 방지합니다.
        if ((now - last_serial_pub_time_).seconds() < 0.05) {
            return;
        }

        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        msg.header.stamp = now;
        msg.drive.speed = (float)v;
        msg.drive.steering_angle = (float)delta;
        drive_pub_->publish(msg);
       
        last_serial_pub_time_ = now; // 발행 시간 업데이트
    }

    // ⭐ [추가] RViz2 시각화 함수 (Red Point)
    void publish_target_marker(double x, double y) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link"; // 차량 기준 좌표계
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "target_point";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE; // 구체 모양
        marker.action = visualization_msgs::msg::Marker::ADD;
       
        // 목표점 위치
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.0;
       
        // 크기 설정 (지름 10cm)
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
       
        // 색상 설정 (빨간색, 불투명)
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
       
        target_marker_pub_->publish(marker);
    }

    double wheelbase_, max_steer_;
    double lookahead_min_, lookahead_max_, lookahead_gain_;
    double min_speed_, max_speed_;
    double current_speed_;
    double smoothed_steer_;
    rclcpp::Time last_serial_pub_time_; // 마지막 Serial 발행 시간 저장

    nav_msgs::msg::Path latest_path_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_pub_; // ⭐ 시각화 퍼블리셔 추가
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}