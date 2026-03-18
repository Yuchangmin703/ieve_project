#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

struct TrackedObject {
    int id;
    float x, y;
    float vx, vy;
    int age;
    int miss_count;
};

class PerceptionNode7 : public rclcpp::Node {
public:
    PerceptionNode7() : Node("perception_node7"), next_id_(0) {
        // QoS 설정: F1TENTH 표준 및 호쿠요 라이다 드라이버와 호환
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&PerceptionNode7::lidar_callback, this, std::placeholders::_1));
        
        pub_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/perception/tracked_objects", 10);
        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/perception/object_markers", 10);

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Perception Node 7 (C++): Duplicate Marker 오류 수정 및 필터 적용 버전");
    }

private:
    const float ALPHA_POS = 0.85f; 
    const float ALPHA_VEL = 0.15f; 

    float get_adaptive_threshold(float dist) {
        if (dist < 1.5f) return 0.18f;
        return 0.28f;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        rclcpp::Time curr_time = this->now();
        double dt = (curr_time - last_time_).seconds();
        if (dt <= 0 || dt > 0.5) { last_time_ = curr_time; return; }

        std::vector<std::pair<float, float>> points;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];
            if (r > 0.1f && r < 4.5f) {
                float angle = msg->angle_min + i * msg->angle_increment;
                points.push_back({r * std::cos(angle), r * std::sin(angle)});
            }
        }

        std::vector<std::pair<float, float>> current_clusters;
        std::vector<bool> visited(points.size(), false);
        for (size_t i = 0; i < points.size(); ++i) {
            if (visited[i]) continue;
            std::vector<size_t> q = {i};
            visited[i] = true;
            float sum_x = 0, sum_y = 0;
            int count = 0;
            size_t head = 0;
            while(head < q.size()){
                size_t curr = q[head++];
                sum_x += points[curr].first; sum_y += points[curr].second; count++;
                float d_sensor = std::sqrt(std::pow(points[curr].first, 2) + std::pow(points[curr].second, 2));
                float threshold = get_adaptive_threshold(d_sensor);
                for (size_t j = 0; j < points.size(); ++j) {
                    if (!visited[j]) {
                        float d = std::sqrt(std::pow(points[curr].first - points[j].first, 2) + std::pow(points[curr].second - points[j].second, 2));
                        if (d < threshold) { visited[j] = true; q.push_back(j); }
                    }
                }
            }
            if (count >= 4) current_clusters.push_back({sum_x / count, sum_y / count});
        }

        update_tracking(current_clusters, dt);
        publish_data(curr_time);
        last_time_ = curr_time;
    }

    void update_tracking(const std::vector<std::pair<float, float>>& clusters, double dt) {
        std::vector<bool> matched(clusters.size(), false);
        for (auto& obj : tracked_objects_) {
            float min_d = 0.5f; 
            int best_idx = -1;
            for (size_t i = 0; i < clusters.size(); ++i) {
                if (matched[i]) continue;
                float d = std::sqrt(std::pow(obj.x - clusters[i].first, 2) + std::pow(obj.y - clusters[i].second, 2));
                if (d < min_d) { min_d = d; best_idx = i; }
            }
            if (best_idx != -1) {
                float raw_vx = (clusters[best_idx].first - obj.x) / dt;
                float raw_vy = (clusters[best_idx].second - obj.y) / dt;
                obj.vx = (1.0f - ALPHA_VEL) * obj.vx + ALPHA_VEL * raw_vx;
                obj.vy = (1.0f - ALPHA_VEL) * obj.vy + ALPHA_VEL * raw_vy;

                obj.x = (1.0f - ALPHA_POS) * obj.x + ALPHA_POS * clusters[best_idx].first;
                obj.y = (1.0f - ALPHA_POS) * obj.y + ALPHA_POS * clusters[best_idx].second;

                obj.age++; obj.miss_count = 0; matched[best_idx] = true;
            } else { obj.miss_count++; }
        }
        for (size_t i = 0; i < clusters.size(); ++i) {
            if (!matched[i]) tracked_objects_.push_back({next_id_++, clusters[i].first, clusters[i].second, 0, 0, 1, 0});
        }
        tracked_objects_.erase(std::remove_if(tracked_objects_.begin(), tracked_objects_.end(),
            [](const TrackedObject& o){ return o.miss_count > 3; }), tracked_objects_.end());
    }

    void publish_data(const rclcpp::Time& t) {
        geometry_msgs::msg::PoseArray pose_array;
        visualization_msgs::msg::MarkerArray marker_array;
        pose_array.header.frame_id = "laser";
        pose_array.header.stamp = t;

        // 1. 모든 마커를 싹 지우는 명령 (가장 먼저 추가)
        visualization_msgs::msg::Marker delete_all;
        delete_all.header.frame_id = "laser";
        delete_all.header.stamp = t;
        delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        // ns를 지정하지 않아야 전체가 지워집니다.
        marker_array.markers.push_back(delete_all);

        for (const auto& obj : tracked_objects_) {
            // 미스 카운트가 있거나 너무 어린 객체는 건너뜀
            if (obj.age < 3 || obj.miss_count > 0) continue;

            float speed = std::sqrt(obj.vx * obj.vx + obj.vy * obj.vy);
            float yaw = std::atan2(obj.vy, obj.vx);

            // PoseArray 데이터 채우기
            geometry_msgs::msg::Pose data_pose;
            data_pose.position.x = obj.x;
            data_pose.position.y = obj.y;
            data_pose.position.z = speed; 
            data_pose.orientation.x = static_cast<double>(obj.id); 
            data_pose.orientation.z = std::sin(yaw / 2.0);
            data_pose.orientation.w = std::cos(yaw / 2.0);
            pose_array.poses.push_back(data_pose);

            // 2. 마커 생성 공통 함수
            auto create_marker = [&](std::string ns, int id_offset, int type) {
                visualization_msgs::msg::Marker m;
                m.header.frame_id = "laser"; 
                m.header.stamp = t;
                m.ns = ns; 
                m.id = obj.id + id_offset; 
                m.type = type; 
                m.action = visualization_msgs::msg::Marker::ADD;
                m.pose.position.x = obj.x; 
                m.pose.position.y = obj.y;
                m.pose.position.z = 0.0;
                m.pose.orientation.w = 1.0;
                // --- 핵심: 마커의 수명을 0.2초로 제한 (데이터가 안 오면 자동 삭제) ---
                m.lifetime = rclcpp::Duration::from_seconds(0.2); 
                return m;
            };

            // --- 화살표 (ns: velocity) ---
            auto arrow = create_marker("velocity", 2000, visualization_msgs::msg::Marker::ARROW);
            arrow.pose.orientation.z = std::sin(yaw / 2.0); 
            arrow.pose.orientation.w = std::cos(yaw / 2.0);
            arrow.scale.x = std::max(0.15f, std::min(speed * 0.3f, 0.7f));
            arrow.scale.y = 0.05; arrow.scale.z = 0.05;
            arrow.color.r = 1.0; arrow.color.a = 1.0;
            marker_array.markers.push_back(arrow);

            // --- 상자 (ns: boxes) ---
            auto cube = create_marker("boxes", 0, visualization_msgs::msg::Marker::CUBE);
            cube.pose.position.z = 0.1; 
            cube.scale.x = 0.2; cube.scale.y = 0.2; cube.scale.z = 0.2;
            cube.color.g = 1.0; cube.color.a = 0.6;
            marker_array.markers.push_back(cube);

            // --- 텍스트 (ns: ids) ---
            auto text = create_marker("ids", 1000, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
            text.pose.position.z = 0.4; 
            text.scale.z = 0.25;
            text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0;
            text.text = "ID: " + std::to_string(obj.id);
            marker_array.markers.push_back(text);
        }
        pub_poses_->publish(pose_array);
        pub_markers_->publish(marker_array);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_poses_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    std::vector<TrackedObject> tracked_objects_;
    rclcpp::Time last_time_;
    int next_id_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode7>());
    rclcpp::shutdown();
    return 0;
}