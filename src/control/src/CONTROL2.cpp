#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace std;

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("control_node") {
        // [설정] 차량 물리 파라미터 및 제어 가중치
        wheelbase_ = 0.26; max_steer_ = 0.523;
        w_cte_ = 100.0; w_epsi_ = 50.0; w_d_steer_ = 1500.0; w_gforce_ = 300.0;
        latency_ = 0.05; max_lat_g_ = 3.0; lpf_alpha_ = 0.2;
        
        current_speed_ = 0.0; filtered_speed_ = 0.0; last_steer_ = 0.0;
        x_ = 0.0; y_ = 0.0; yaw_ = 0.0; N_ = 10;

        // ⭐ TF 브로드캐스터 초기화
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // [구독] 경로 및 현재 속도 (SerialBridge로부터 수신)
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planning/local_path", 10, bind(&ControlNode::path_callback, this, placeholders::_1));
        
        ego_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/ego_speed", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
                // Low Pass Filter로 엔코더 노이즈 제거
                this->filtered_speed_ = lpf_alpha_ * msg->data + (1.0 - lpf_alpha_) * this->filtered_speed_;
                this->current_speed_ = this->filtered_speed_;
                this->update_odometry(this->current_speed_);
            });

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/auto_drive", 10);
        RCLCPP_INFO(this->get_logger(), "🔥 [Control] MPC 실행 중 (실전 주행 모드 + TF 방송)");
    }

private:
    // 데드 레코닝: 실제 속도와 조향각으로 현재 위치(x, y, yaw) 추정
    void update_odometry(double v) {
        rclcpp::Time now = this->get_clock()->now();
        if (last_odom_time_.nanoseconds() != 0) {
            double dt = (now - last_odom_time_).seconds();
            
            // ⭐ [복구 완료] 실제 주행용 운동학(Kinematic) 모델
            x_ += v * cos(yaw_) * dt;
            y_ += v * sin(yaw_) * dt;
            yaw_ += (v / wheelbase_) * tan(last_steer_) * dt;
        }
        last_odom_time_ = now;
        
        // ⭐ 위치 업데이트가 끝날 때마다 RViz로 위치 전송
        publish_tf(); 
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) { publish_drive(0.0, 0.0); return; }

        double target_v = msg->poses[0].pose.position.z;
        if (target_v <= 0.01) target_v = 3.0;

        // 지연 보상: latency초 후의 위치 예측
        double cv = max(current_speed_, 0.5);
        double cx = x_ + cv * cos(yaw_) * latency_;
        double cy = y_ + cv * sin(yaw_) * latency_;
        double cyaw = yaw_ + (cv / wheelbase_) * tan(last_steer_) * latency_;

        double best_steer = find_optimal_steering(*msg, cx, cy, cyaw, cv);
        last_steer_ = best_steer;
        publish_drive(target_v, best_steer);
    }

    // Coarse-to-Fine 최적화 및 비용 함수 로직
    double find_optimal_steering(const nav_msgs::msg::Path& path, double cx, double cy, double cyaw, double cv) {
        double best_s = 0.0; double min_c = 1e18;
        double steps[] = { 0.1, 0.01 };
        double center = 0.0;
        for (double step : steps) {
            double low = max(-max_steer_, center - 0.15);
            double high = min(max_steer_, center + 0.15);
            for (double s = low; s <= high; s += step) {
                double c = evaluate_cost(path, s, cx, cy, cyaw, cv);
                if (c < min_c) { min_c = c; best_s = s; }
            }
            center = best_s;
        }
        return best_s;
    }

    double evaluate_cost(const nav_msgs::msg::Path& path, double steer, double cx, double cy, double cyaw, double cv) {
        double cost = 0.0;
        double px = cx, py = cy, pyaw = cyaw;
        int search_idx = 0;
        double adaptive_dt = 0.05 * (1.0 + 0.2 * cv); 

        for (int i = 1; i <= N_; ++i) {
            double step_v = path.poses[search_idx].pose.position.z;
            if (step_v <= 0.1) step_v = cv;
            px += step_v * cos(pyaw) * adaptive_dt;
            py += step_v * sin(pyaw) * adaptive_dt;
            pyaw += (step_v / wheelbase_) * tan(steer) * adaptive_dt;

            double min_d = 1e9; int best_j = search_idx;
            for (int j = search_idx; j < min((int)path.poses.size(), search_idx + 15); ++j) {
                double d = pow(path.poses[j].pose.position.x - px, 2) + pow(path.poses[j].pose.position.y - py, 2);
                if (d < min_d) { min_d = d; best_j = j; }
            }
            search_idx = best_j;

            // 횡가속도 제한 패널티
            double radius = wheelbase_ / (tan(abs(steer)) + 1e-6);
            double lat_g = (step_v * step_v) / radius;
            if (lat_g > max_lat_g_) cost += w_gforce_ * pow(lat_g - max_lat_g_, 2);

            cost += w_cte_ * min_d; // 경로 이탈 오차 합산
        }
        cost += w_d_steer_ * pow(steer - last_steer_, 2);
        return cost;
    }

    void publish_drive(double v, double delta) {
        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.drive.speed = (float)v;
        msg.drive.steering_angle = (float)max(-max_steer_, min(max_steer_, delta));
        drive_pub_->publish(msg);
    }

    // ⭐ RViz 시각화를 위한 TF 발행 함수
    void publish_tf() {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;
        // Yaw(라디안) 값을 쿼터니언으로 변환
        t.transform.rotation.z = sin(yaw_ / 2.0);
        t.transform.rotation.w = cos(yaw_ / 2.0);
        tf_broadcaster_->sendTransform(t);
    }

    double wheelbase_, max_steer_, w_cte_, w_epsi_, w_d_steer_, w_gforce_, current_speed_, filtered_speed_, last_steer_, latency_, max_lat_g_, lpf_alpha_, x_, y_, yaw_;
    int N_;
    rclcpp::Time last_odom_time_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ego_speed_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
