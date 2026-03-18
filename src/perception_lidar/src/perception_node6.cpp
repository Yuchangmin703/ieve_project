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

class PerceptionNode6 : public rclcpp::Node {
public:
    PerceptionNode6() : Node("perception_node6"), next_id_(0) {
        // QoS 설정: F1TENTH 표준 및 호쿠요 라이다 드라이버와 호환
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&PerceptionNode6::lidar_callback, this, std::placeholders::_1));
        
        // Planning 노드 전송용 PoseArray
        pub_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/perception/tracked_objects", 10);
        // RViz 시각화용 MarkerArray
        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/perception/object_markers", 10);

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Perception Node 6 (C++): Planning 데이터 호환 및 시각화 최적화 모드");
    }

private:
    // 거리에 따른 클러스터링 거리 임계값 조절 (Adaptive Clustering)
    float get_adaptive_threshold(float dist) {
        if (dist < 1.5f) return 0.18f;
        return 0.28f;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        rclcpp::Time curr_time = this->now();
        double dt = (curr_time - last_time_).seconds();
        if (dt <= 0 || dt > 0.5) { last_time_ = curr_time; return; }

        // 1. Scan 데이터를 XY 평면 좌표로 변환
        std::vector<std::pair<float, float>> points;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];
            if (r > 0.1f && r < 4.5f) { // 인지 범위 설정
                float angle = msg->angle_min + i * msg->angle_increment;
                points.push_back({r * std::cos(angle), r * std::sin(angle)});
            }
        }

        // 2. Clustering: 점 4개 이상 모여야 물체로 인지 (노이즈 필터링)
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
                
                // Low Pass Filter 적용 (속도 요동 방지)
                float alpha = 0.15f; 
                obj.vx = (1 - alpha) * obj.vx + alpha * raw_vx;
                obj.vy = (1 - alpha) * obj.vy + alpha * raw_vy;

                obj.x = clusters[best_idx].first; obj.y = clusters[best_idx].second;
                obj.age++; obj.miss_count = 0; matched[best_idx] = true;
            } else { obj.miss_count++; }
        }
        for (size_t i = 0; i < clusters.size(); ++i) {
            if (!matched[i]) tracked_objects_.push_back({next_id_++, clusters[i].first, clusters[i].second, 0, 0, 1, 0});
        }
        // 물체가 사라지면 즉시(miss_count > 3) 리스트에서 제거하여 잔상 방지
        tracked_objects_.erase(std::remove_if(tracked_objects_.begin(), tracked_objects_.end(),
            [](const TrackedObject& o){ return o.miss_count > 3; }), tracked_objects_.end());
    }

    void publish_data(const rclcpp::Time& t) {
        geometry_msgs::msg::PoseArray pose_array;
        visualization_msgs::msg::MarkerArray marker_array;
        pose_array.header.frame_id = "laser";
        pose_array.header.stamp = t;

        // RViz 이전 마커들 즉시 삭제 명령
        visualization_msgs::msg::Marker delete_all;
        delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_all);

        for (const auto& obj : tracked_objects_) {
            // 최소 3프레임 이상 생존한 신뢰할 수 있는 물체만 발행
            if (obj.age >= 3 && obj.miss_count == 0) { 
                float speed = std::sqrt(obj.vx * obj.vx + obj.vy * obj.vy);
                float yaw = std::atan2(obj.vy, obj.vx);

                // --- 1. Planning용 PoseArray (데이터 전송용) ---
                geometry_msgs::msg::Pose data_pose;
                data_pose.position.x = obj.x;
                data_pose.position.y = obj.y;
                data_pose.position.z = speed; // Planning 노드에서 속력으로 사용
                data_pose.orientation.x = static_cast<double>(obj.id); // Planning 노드에서 ID로 사용
                
                // 방향 데이터는 평면상의 Yaw 값으로 채움
                data_pose.orientation.z = std::sin(yaw / 2.0);
                data_pose.orientation.w = std::cos(yaw / 2.0);
                pose_array.poses.push_back(data_pose);

                // --- 2. 시각화 전용 화살표 (수평 고정) ---
                visualization_msgs::msg::Marker arrow;
                arrow.header.frame_id = "laser"; arrow.header.stamp = t;
                arrow.ns = "velocity_visual"; arrow.id = obj.id + 2000;
                arrow.type = visualization_msgs::msg::Marker::ARROW;
                arrow.action = visualization_msgs::msg::Marker::ADD;
                arrow.pose.position.x = obj.x; arrow.pose.position.y = obj.y; arrow.pose.position.z = 0.0;
                arrow.pose.orientation.x = 0.0; arrow.pose.orientation.y = 0.0;
                arrow.pose.orientation.z = std::sin(yaw / 2.0); arrow.pose.orientation.w = std::cos(yaw / 2.0);
                
                // 화살표 길이: 속도에 비례하되 보기 좋게 스케일링 (0.3m/s 당 약 10cm)
                float visual_length = std::min(speed * 0.3f, 0.7f); 
                arrow.scale.x = std::max(0.15f, visual_length); // 최소/최대 길이 제한
                arrow.scale.y = 0.05; arrow.scale.z = 0.05;
                arrow.color.r = 1.0; arrow.color.a = 1.0; // 빨간색
                marker_array.markers.push_back(arrow);

                // --- 3. 초록색 상자 마커 ---
                visualization_msgs::msg::Marker cube;
                cube.header.frame_id = "laser"; cube.ns = "boxes"; cube.id = obj.id;
                cube.type = visualization_msgs::msg::Marker::CUBE;
                cube.pose.position.x = obj.x; cube.pose.position.y = obj.y; cube.pose.position.z = 0.1;
                cube.scale.x = 0.2; cube.scale.y = 0.2; cube.scale.z = 0.2;
                cube.color.g = 1.0; cube.color.a = 0.6; // 반투명 초록
                marker_array.markers.push_back(cube);

                // --- 4. ID 텍스트 마커 ---
                visualization_msgs::msg::Marker text;
                text.header.frame_id = "laser"; text.ns = "ids"; text.id = obj.id + 1000;
                text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                text.pose.position.x = obj.x; text.pose.position.y = obj.y; text.pose.position.z = 0.4;
                text.scale.z = 0.25;
                text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0;
                text.text = "ID: " + std::to_string(obj.id);
                marker_array.markers.push_back(text);
            }
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
    rclcpp::spin(std::make_shared<PerceptionNode6>());
    rclcpp::shutdown();
    return 0;
}