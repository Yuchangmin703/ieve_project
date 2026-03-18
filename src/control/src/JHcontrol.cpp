#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>

using namespace std;

class PurePursuitControlNode : public rclcpp::Node {
public:
    PurePursuitControlNode() : Node("pure_pursuit_control_node") {
        // ==========================================================
        // ⭐ [파라미터 선언] 튜닝이 매우 직관적입니다.
        // ==========================================================
        declare_parameter<double>("wheelbase", 0.26);       // 차량 축거 (앞뒤 바퀴 거리)
        declare_parameter<double>("max_steer", 0.523);      // 최대 조향각 (약 30도)
        
        // Pure Pursuit 전용 파라미터 (동적 전방 주시 거리)
        declare_parameter<double>("lookahead_k", 0.5);      // 속도 비례 계수 (속도가 빠를수록 멀리 봄)
        declare_parameter<double>("lookahead_min", 0.4);    // 최소 주시 거리 (m)
        
        // 방어망 A (입력 필터): 이동 평균 필터 사이즈
        declare_parameter<int>("ma_window_size", 5);        // 5개 데이터를 모아서 평균을 냄
        
        // 방어망 B (출력 제한): 가속도 및 조향 변화율 제한 (Slew Rate)
        declare_parameter<double>("max_accel", 1.5);        // 초당 최대 가속/감속량 (m/s^2)
        declare_parameter<double>("max_steer_rate", 2.0);   // 초당 최대 조향 변화량 (rad/s)

        // 파라미터 로드
        wheelbase_ = get_parameter("wheelbase").as_double();
        max_steer_ = get_parameter("max_steer").as_double();
        lookahead_k_ = get_parameter("lookahead_k").as_double();
        lookahead_min_ = get_parameter("lookahead_min").as_double();
        ma_window_size_ = get_parameter("ma_window_size").as_int();
        max_accel_ = get_parameter("max_accel").as_double();
        max_steer_rate_ = get_parameter("max_steer_rate").as_double();

        current_speed_ = 0.0;
        last_pub_speed_ = 0.0;
        last_pub_steer_ = 0.0;
        x_ = 0.0; y_ = 0.0; yaw_ = 0.0;
        last_cmd_time_ = this->get_clock()->now();

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // ==========================================================
        // ⭐ [구독 1] 방어망 A 적용: 엔코더 속도 이동 평균 필터
        // ==========================================================
        ego_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/ego_speed", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
                // 이동 평균 큐에 데이터 삽입
                speed_buffer_.push_back(msg->data);
                if ((int)speed_buffer_.size() > ma_window_size_) {
                    speed_buffer_.pop_front();
                }
                
                // 평균 계산 (노이즈 평활화)
                double sum = 0.0;
                for (float s : speed_buffer_) sum += s;
                current_speed_ = sum / speed_buffer_.size();
                
                // TF 시각화를 위한 Odometry 업데이트
                update_odometry(current_speed_);
            });

        // ==========================================================
        // ⭐ [구독 2] 플래닝 경로 수신 및 Pure Pursuit 실행
        // ==========================================================
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planning/local_path", 10, bind(&PurePursuitControlNode::path_callback, this, placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/auto_drive", 10);
        RCLCPP_INFO(this->get_logger(), "🚀 [Control] Pure Pursuit + 이중 방어망 제어 실행 중!");
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            publish_drive_limited(0.0, 0.0);
            return;
        }

        // 플래닝이 z축에 숨겨서 보낸 '목표 속도(Target Speed)' 추출
        double target_v = msg->poses[0].pose.position.z;

        // ==========================================================
        // ⭐ 동적 Pure Pursuit 알고리즘
        // ==========================================================
        // 1. 현재 속도에 비례하여 전방 주시 거리(Lookahead Distance) 계산
        double Ld = lookahead_k_ * current_speed_ + lookahead_min_;
        
        double target_x = 0.0;
        double target_y = 0.0;
        bool point_found = false;

        // 2. 경로 상에서 Ld보다 멀리 있는 첫 번째 점 찾기
        for (const auto& pose_stamped : msg->poses) {
            double px = pose_stamped.pose.position.x;
            double py = pose_stamped.pose.position.y;
            double dist = hypot(px, py);

            if (dist >= Ld) {
                target_x = px;
                target_y = py;
                Ld = dist; // 실제 찾은 점의 거리로 Ld 보정
                point_found = true;
                break;
            }
        }

        // 만약 경로가 너무 짧아서 Ld만큼 먼 점이 없다면 가장 끝점을 타겟으로 삼음
        if (!point_found) {
            target_x = msg->poses.back().pose.position.x;
            target_y = msg->poses.back().pose.position.y;
            Ld = hypot(target_x, target_y);
        }

        // 3. 기하학적 조향각 계산 공식 (Pure Pursuit Formula)
        // 델타 = arctan(2 * L * y / Ld^2)
        double raw_steer = 0.0;
        if (Ld > 0.01) {
            raw_steer = std::atan2(2.0 * wheelbase_ * target_y, Ld * Ld);
        }

        // 4. 안전 방어망(B)을 거쳐 최종 출력
        publish_drive_limited(target_v, raw_steer);
    }

    // ==========================================================
    // ⭐ 방어망 B (출력 제한): 급가감속 및 조향 진동 완벽 차단
    // ==========================================================
    void publish_drive_limited(double target_v, double raw_steer) {
        rclcpp::Time now = this->get_clock()->now();
        double dt = (now - last_cmd_time_).seconds();
        if (dt <= 0.0 || dt > 0.5) dt = 0.05; // 비정상 시간차 예외 처리
        last_cmd_time_ = now;

        // 허용 가능한 최대 변화량 계산
        double max_dv = max_accel_ * dt;
        double max_dsteer = max_steer_rate_ * dt;

        double final_v = target_v;
        double final_steer = raw_steer;

        // 1. 속도 Slew Rate Limit (목표 속도가 널뛰어도 부드럽게 따라감)
        if (target_v > last_pub_speed_ + max_dv) {
            final_v = last_pub_speed_ + max_dv;
        } else if (target_v < last_pub_speed_ - max_dv) {
            final_v = last_pub_speed_ - max_dv;
        }

        // 2. 조향각 Slew Rate Limit (핸들 덜컹거림 원천 차단)
        if (raw_steer > last_pub_steer_ + max_dsteer) {
            final_steer = last_pub_steer_ + max_dsteer;
        } else if (raw_steer < last_pub_steer_ - max_dsteer) {
            final_steer = last_pub_steer_ - max_dsteer;
        }

        // 물리적 한계 각도 자르기 (Clamp)
        final_steer = std::max(-max_steer_, std::min(max_steer_, final_steer));

        // 최종 발행 및 상태 업데이트
        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        msg.header.stamp = now;
        msg.drive.speed = (float)final_v;
        msg.drive.steering_angle = (float)final_steer;
        drive_pub_->publish(msg);

        last_pub_speed_ = final_v;
        last_pub_steer_ = final_steer;
    }

    // 데드 레코닝 로직 (기존과 동일 - 시각화용)
    void update_odometry(double v) {
        rclcpp::Time now = this->get_clock()->now();
        if (last_odom_time_.nanoseconds() != 0) {
            double dt = (now - last_odom_time_).seconds();
            x_ += v * cos(yaw_) * dt;
            y_ += v * sin(yaw_) * dt;
            yaw_ += (v / wheelbase_) * tan(last_pub_steer_) * dt;
        }
        last_odom_time_ = now;
        publish_tf(); 
    }

    void publish_tf() {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;
        t.transform.rotation.z = sin(yaw_ / 2.0);
        t.transform.rotation.w = cos(yaw_ / 2.0);
        tf_broadcaster_->sendTransform(t);
    }

    // 설정 변수들
    double wheelbase_, max_steer_;
    double lookahead_k_, lookahead_min_;
    int ma_window_size_;
    double max_accel_, max_steer_rate_;

    // 상태 저장 변수들
    std::deque<float> speed_buffer_;
    double current_speed_;
    double last_pub_speed_, last_pub_steer_;
    rclcpp::Time last_cmd_time_;
    
    // 시각화용 Odometry 변수
    double x_, y_, yaw_;
    rclcpp::Time last_odom_time_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ego_speed_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<PurePursuitControlNode>());
    rclcpp::shutdown();
    return 0;
}