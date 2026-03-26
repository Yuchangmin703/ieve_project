#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <algorithm>
#include <chrono>

using namespace std;

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("control_node") {
        // ==========================================
        // ⚙️ 차량 제어 핵심 튜닝 패널 
        // ==========================================
        wheelbase_ = 0.257;      
        max_steer_ = 0.523598;   // 약 30.0도 (라디안)
        
        // ⭐ S-Curve 비선형 조향 가중치 및 기준 각도 설정
        expo_weight_ = 0.9;      
        crossover_deg_ = 20.0;   
        
        // ⭐ [NEW] 타원형 Pure Pursuit 측면 시야 비율 설정
        // 1.0이면 기존의 완전한 원형. 값이 작아질수록 (예: 0.6) 코너에서 시야가 짧아져 더 예리하게 돕니다.
        lateral_ratio_ = 0.65;   
        
        lookahead_min_ = 0.5;    
        lookahead_max_ = 1.5;    
        lookahead_gain_ = 0.7;   
        min_speed_ = 0.15;      
        max_speed_ = 0.5;        
        // ==========================================
        
        current_speed_ = 0.0;
        smoothed_steer_ = 0.0;
        last_path_time_ = this->get_clock()->now();

        // 수신 큐 사이즈 1 (항상 최신 경로와 최신 속도만 확인)
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planning/local_path", 1, bind(&ControlNode::path_callback, this, placeholders::_1));
            
        speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/ego_speed", 1, bind(&ControlNode::speed_callback, this, placeholders::_1));
            
        // 발행 큐 사이즈 1
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/auto_drive", 1);
        target_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/target_point", 1);

        // 15Hz 제어 루프 (약 66ms 간격)
        timer_ = this->create_wall_timer(
            chrono::milliseconds(66),
            bind(&ControlNode::control_loop, this)
        );
    }

private:
    void speed_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_speed_ = msg->data;
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        latest_path_ = *msg;
        last_path_time_ = this->get_clock()->now();
    }

    void control_loop() {
        double path_age = (this->get_clock()->now() - last_path_time_).seconds();
        if (path_age > 0.5 || latest_path_.poses.empty()) {
            publish_drive(0.0, 0.0);
            return;
        }

        // 동적 Lookahead 거리 계산 (타원의 기준이 되는 긴 축)
        double dynamic_lookahead = lookahead_min_ + (lookahead_gain_ * abs(current_speed_));
        dynamic_lookahead = max(lookahead_min_, min(lookahead_max_, dynamic_lookahead));

        // ========================================================
        // ⭐ [NEW] 타원형(Elliptical) Target Point 탐색 로직 적용
        // ========================================================
        double a = dynamic_lookahead;                  // 전방 탐색 거리 (긴 축, x축)
        double b = dynamic_lookahead * lateral_ratio_; // 측면 탐색 거리 (짧은 축, y축)

        // b가 0이 되는 것을 방지하기 위한 안전장치
        if (b < 0.1) b = 0.1;

        int target_idx = 0;
        for (size_t i = 0; i < latest_path_.poses.size(); ++i) {
            double tx = latest_path_.poses[i].pose.position.x;
            double ty = latest_path_.poses[i].pose.position.y;
            
            // 타원 방정식: (x^2 / a^2) + (y^2 / b^2) >= 1
            double ellipse_val = (tx * tx) / (a * a) + (ty * ty) / (b * b);
            
            // 경로의 점이 타원 경계선에 닿거나 타원 밖으로 나가는 순간을 타겟으로 지정
            if (ellipse_val >= 1.0) {
                target_idx = i;
                break;
            }
        }
        
        if (target_idx == 0 && latest_path_.poses.size() > 1) {
            target_idx = latest_path_.poses.size() - 1;
        }

        double tx = latest_path_.poses[target_idx].pose.position.x;
        double ty = latest_path_.poses[target_idx].pose.position.y;
        
        double raw_target_v = latest_path_.poses[target_idx].pose.position.z;
        if (!std::isfinite(raw_target_v)) raw_target_v = 0.0;
        double final_target_v = std::clamp(raw_target_v, -max_speed_, max_speed_);

        publish_target_marker(tx, ty);

        // 1. 순수 기하학적 조향각 계산 (Pure Pursuit)
        // 주의: 조향각을 계산하는 수학 공식 자체는 실제 유클리드 거리가 필요하므로 hypot를 사용합니다.
        double alpha = atan2(ty, tx);
        double actual_Ld = hypot(tx, ty);
        double steer = 0.0;
        
        if (actual_Ld > 0.01) {
            steer = atan2(2.0 * wheelbase_ * sin(alpha), actual_Ld);
        }
        
        // 2. 하드웨어 물리적 한계로 클램핑 (±30도)
        steer = max(-max_steer_, min(max_steer_, steer));

        // 3. 사용자 맞춤형 커스텀 S-Curve 적용 (crossover_deg_ 기준)
        double norm_steer = steer / max_steer_; // -1.0 ~ 1.0 비율로 변환

        double max_steer_deg = max_steer_ * (180.0 / M_PI);
        double xc = crossover_deg_ / max_steer_deg;
        xc = std::max(0.1, std::min(0.9, xc)); 

        double A = (1.0 / xc) + 1.0;
        double B = 1.0 - A;

        double custom_s_curve = A * norm_steer * std::abs(norm_steer) + B * std::pow(norm_steer, 3.0);
        double blended_norm = (1.0 - expo_weight_) * norm_steer + expo_weight_ * custom_s_curve;
        
        steer = blended_norm * max_steer_; 

        // 4. 감속 로직 (코너링 시 속도 줄임)
        if (abs(final_target_v) > 0.01) {
            double cornering_factor = 1.0 - (abs(steer) / max_steer_) * 0.6;
            final_target_v = final_target_v * cornering_factor;
        }

        // 5. 반응성 향상을 위해 최신 조향값 90% 반영 (Low-pass Filter)
        smoothed_steer_ = (0.9 * steer) + (0.1 * smoothed_steer_);

        // 6. 최소 속도 보장
        if (abs(final_target_v) > 0.01 && abs(final_target_v) < min_speed_) {
            final_target_v = (final_target_v > 0) ? min_speed_ : -min_speed_;
        } else if (abs(final_target_v) <= 0.01) {
            final_target_v = 0.0;
        }

        publish_drive(final_target_v, smoothed_steer_);
    }

    void publish_drive(double v, double delta) {
        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.drive.speed = (float)v;
        msg.drive.steering_angle = (float)delta;
        drive_pub_->publish(msg);
    }

    void publish_target_marker(double x, double y) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link"; 
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "target_point";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE; 
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.0;
        
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        target_marker_pub_->publish(marker);
    }

    // 멤버 변수
    double wheelbase_, max_steer_;
    double expo_weight_;    
    double crossover_deg_;  
    double lateral_ratio_;  // ⭐ [NEW] 타원 측면 찌그러짐 비율
    double lookahead_min_, lookahead_max_, lookahead_gain_;
    double min_speed_, max_speed_;
    double current_speed_;
    double smoothed_steer_;
    rclcpp::Time last_path_time_; 

    nav_msgs::msg::Path latest_path_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_pub_; 
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
