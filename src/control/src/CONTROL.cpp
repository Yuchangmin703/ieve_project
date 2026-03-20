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
        // [설정] 차량 물리 파라미터
        wheelbase_ = 0.26; max_steer_ = 0.523;
        w_cte_ = 100.0; w_d_steer_ = 1500.0; w_gforce_ = 300.0;
        latency_ = 0.05; max_lat_g_ = 3.0;

        // 초기값 설정
        current_speed_ = 0.0; last_steer_ = 0.0;
        x_ = 0.0; y_ = 0.0; yaw_ = 0.0; N_ = 10;
        is_initialized_ = false; 

        // [구독] 가상 경로 (Dummy로부터 수신)
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planning/local_path", 10, bind(&ControlNode::path_callback, this, placeholders::_1));

        // [발행] 제어 명령 및 TF 시각화
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // ✅ 수정: create_timer -> create_wall_timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&ControlNode::sim_loop, this));

        RCLCPP_INFO(this->get_logger(), "🚀 [Control] 시뮬레이션 모드 가동 중");
    }

private:
    void sim_loop() {
        // ✅ 수정: .now() -> ->now()
        rclcpp::Time now = this->get_clock()->now();
        if (last_sim_time_.nanoseconds() != 0) {
            double dt = (now - last_sim_time_).seconds();
            
            x_ += current_speed_ * cos(yaw_) * dt;
            y_ += current_speed_ * sin(yaw_) * dt;
            yaw_ += (current_speed_ / wheelbase_) * tan(last_steer_) * dt;
        }
        last_sim_time_ = now;
        publish_tf(); 
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.size() < 2) return;

        if (!is_initialized_) {
            x_ = msg->poses[0].pose.position.x;
            y_ = msg->poses[0].pose.position.y;
            yaw_ = atan2(msg->poses[1].pose.position.y - y_, msg->poses[1].pose.position.x - x_);
            is_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "📍 경로 시작점으로 위치 초기화 완료!");
        }

        double target_v = msg->poses[0].pose.position.z;
        if (target_v <= 0.01) target_v = 2.0;

        double cv = max(current_speed_, 0.5);
        double cx = x_ + cv * cos(yaw_) * latency_;
        double cy = y_ + cv * sin(yaw_) * latency_;
        double cyaw = yaw_ + (cv / wheelbase_) * tan(last_steer_) * latency_;

        double best_steer = find_optimal_steering(*msg, cx, cy, cyaw, cv);
        
        current_speed_ = target_v; 
        last_steer_ = best_steer;
        
        publish_drive(target_v, best_steer);
    }

    double find_optimal_steering(const nav_msgs::msg::Path& path, double cx, double cy, double cyaw, double cv) {
        double best_s = 0.0; double min_c = 1e18;
        for (double s = -max_steer_; s <= max_steer_; s += 0.05) {
            double c = evaluate_cost(path, s, cx, cy, cyaw, cv);
            if (c < min_c) { min_c = c; best_s = s; }
        }
        return best_s;
    }

    double evaluate_cost(const nav_msgs::msg::Path& path, double steer, double cx, double cy, double cyaw, double cv) {
        double cost = 0.0;
        double px = cx, py = cy, pyaw = cyaw;
        int search_idx = 0;
        for (int i = 1; i <= N_; ++i) {
            px += cv * cos(pyaw) * 0.1;
            py += cv * sin(pyaw) * 0.1;
            pyaw += (cv / wheelbase_) * tan(steer) * 0.1;

            double min_d = 1e9;
            for (int j = search_idx; j < min((int)path.poses.size(), search_idx + 20); ++j) {
                double d = pow(path.poses[j].pose.position.x - px, 2) + pow(path.poses[j].pose.position.y - py, 2);
                if (d < min_d) { min_d = d; search_idx = j; }
            }
            cost += w_cte_ * min_d;
        }
        cost += w_d_steer_ * pow(steer - last_steer_, 2);
        return cost;
    }

    void publish_drive(double v, double delta) {
        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        // ✅ 수정: .now() -> ->now()
        msg.header.stamp = this->get_clock()->now();
        msg.drive.speed = (float)v;
        msg.drive.steering_angle = (float)delta;
        drive_pub_->publish(msg);
    }

    void publish_tf() {
        geometry_msgs::msg::TransformStamped t;
        // ✅ 수정: .now() -> ->now()
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.rotation.z = sin(yaw_ / 2.0);
        t.transform.rotation.w = cos(yaw_ / 2.0);
        tf_broadcaster_->sendTransform(t);
    }

    // ✅ 변수 선언부: w_gforce_ 추가됨
    double wheelbase_, max_steer_, w_cte_, w_d_steer_, w_gforce_, current_speed_, last_steer_, x_, y_, yaw_, latency_, max_lat_g_;
    int N_; bool is_initialized_;
    rclcpp::Time last_sim_time_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
