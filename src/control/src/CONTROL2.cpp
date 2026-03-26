#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <algorithm>
#include <chrono>

using namespace std;

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("control_node") {
        wheelbase_ = 0.257;      
        max_steer_ = 0.523598;   
        
        expo_weight_ = 0.85;      
        crossover_deg_ = 15.0;   
        
        min_speed_ = 0.15;      
        max_speed_ = 1.0;        
        
        current_speed_ = 0.0;
        smoothed_steer_ = 0.0;
        last_path_time_ = this->get_clock()->now();

        // ROS 2 통신 설정
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planning/local_path", 1, bind(&ControlNode::path_callback, this, placeholders::_1));
            
        speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/ego_speed", 1, bind(&ControlNode::speed_callback, this, placeholders::_1));
            
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/auto_drive", 1);
        target_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/target_point", 1);
        parabola_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/parabola_boundary", 1);

        // ✅ [최적화] 포물선 마커 사전 연산 (루프 내부의 불필요한 연산 제거)
        init_parabola_marker();

        // ✅ 20Hz (50ms) 고정 주기 타이머
        timer_ = this->create_wall_timer(
            chrono::milliseconds(50),
            bind(&ControlNode::control_loop, this)
        );
    }

private:
    void init_parabola_marker() {
        parabola_marker_.header.frame_id = "base_link";
        parabola_marker_.ns = "parabola_boundary";
        parabola_marker_.id = 1;
        parabola_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        parabola_marker_.action = visualization_msgs::msg::Marker::ADD;
        parabola_marker_.scale.x = 0.02; // 선 두께
        
        // 형광 초록색
        parabola_marker_.color.r = 0.0;
        parabola_marker_.color.g = 1.0;
        parabola_marker_.color.b = 0.0;
        parabola_marker_.color.a = 1.0;

        double L_max = 1.3;
        double y_width = 0.3;
        double a = L_max / (y_width * y_width); // 약 14.4444

        for (double y = -y_width; y <= y_width; y += 0.02) {
            geometry_msgs::msg::Point p;
            p.x = L_max - a * (y * y);
            p.y = y;
            p.z = 0.0;
            parabola_marker_.points.push_back(p);
        }
    }

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

        // ✅ 1. 지연 없는 고속 타겟 탐색 (O(N))
        double L_max = 1.3;
        double a = 14.44444444; // 1.3 / 0.09
        int target_idx = -1;
        size_t path_size = latest_path_.poses.size();

        for (size_t i = 0; i < path_size; ++i) {
            double tx = latest_path_.poses[i].pose.position.x;
            double ty = latest_path_.poses[i].pose.position.y;
            
            if (tx < 0.0) continue; // 차량 뒤쪽 점은 연산 없이 즉시 스킵
            
            // 수학적 교차 판단: x좌표가 포물선 경계(L - ay^2)를 넘었는가?
            if (tx >= (L_max - a * ty * ty)) {
                target_idx = i;
                break;
            }
        }
        
        // 포물선을 벗어나는 점이 없다면 가장 마지막 점으로 대체
        if (target_idx == -1) {
            target_idx = path_size - 1;
        }

        double tx = latest_path_.poses[target_idx].pose.position.x;
        double ty = latest_path_.poses[target_idx].pose.position.y;
        
        double raw_target_v = latest_path_.poses[target_idx].pose.position.z;
        if (!std::isfinite(raw_target_v)) raw_target_v = 0.0;
        double final_target_v = std::clamp(raw_target_v, -max_speed_, max_speed_);

        // 시각화 퍼블리시
        publish_target_marker(tx, ty);
        
        // 사전 연산된 포물선 마커의 시간 스탬프만 갱신 후 즉시 발행
        parabola_marker_.header.stamp = this->get_clock()->now();
        parabola_marker_pub_->publish(parabola_marker_);

        // ✅ 2. 조향각 연산 및 10도 미만 반감 로직
        double alpha = atan2(ty, tx);
        double actual_Ld = hypot(tx, ty);
        double steer = 0.0;
        
        if (actual_Ld > 0.01) {
            steer = atan2(2.0 * wheelbase_ * sin(alpha), actual_Ld);
        }
        
        // 10도 미만 반감 로직 (10도 = 약 0.1745 라디안)
        if (abs(steer) < 0.1745329) {
            steer *= 0.5;
        }
        
        steer = max(-max_steer_, min(max_steer_, steer));

        // S-Curve 및 스무딩 (기존 로직 유지, 연산량 매우 적음)
        double norm_steer = steer / max_steer_; 
        double max_steer_deg = max_steer_ * (180.0 / M_PI);
        double xc = crossover_deg_ / max_steer_deg;
        xc = std::max(0.1, std::min(0.9, xc)); 

        double A = (1.0 / xc) + 1.0;
        double B = 1.0 - A;

        double custom_s_curve = A * norm_steer * std::abs(norm_steer) + B * std::pow(norm_steer, 3.0);
        double blended_norm = (1.0 - expo_weight_) * norm_steer + expo_weight_ * custom_s_curve;
        
        steer = blended_norm * max_steer_; 

        if (abs(final_target_v) > 0.01) {
            double cornering_factor = 1.0 - (abs(steer) / max_steer_) * 0.6;
            final_target_v = final_target_v * cornering_factor;
        }

        smoothed_steer_ = (0.8 * steer) + (0.2 * smoothed_steer_);

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
        
        // ✅ [정밀도 제어] 속도 소수점 2자리, 조향각 4자리 절삭
        double rounded_v = std::round(v * 100.0) / 100.0;
        double rounded_steer = std::round(delta * 10000.0) / 10000.0;

        msg.drive.speed = static_cast<float>(rounded_v);
        msg.drive.steering_angle = static_cast<float>(rounded_steer);
        
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

    double wheelbase_, max_steer_;
    double expo_weight_;    
    double crossover_deg_;  
    double min_speed_, max_speed_;
    double current_speed_;
    double smoothed_steer_;
    rclcpp::Time last_path_time_; 

    nav_msgs::msg::Path latest_path_;
    visualization_msgs::msg::Marker parabola_marker_; // 사전 연산용 마커 변수

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_pub_; 
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr parabola_marker_pub_; 
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
