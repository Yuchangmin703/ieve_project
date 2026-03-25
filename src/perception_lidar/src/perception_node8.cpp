#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>

struct TrackedObject {
    int id;
    float x, y;
    float vx, vy;
    int age;
    int miss_count;
    std::deque<float> hist_vx; 
    std::deque<float> hist_vy; 
};

// ⭐ [신규 최적화] 클러스터의 첫 점과 끝 점을 저장할 초경량 구조체
struct ClusterPt {
    float sum_x, sum_y;
    int count;
    float first_x, first_y; // 클러스터의 시작점
    float last_x, last_y;   // 클러스터의 끝점
};

class PerceptionNode8 : public rclcpp::Node {
public:
    PerceptionNode8() : Node("perception_node8"), next_id_(0) {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&PerceptionNode8::lidar_callback, this, std::placeholders::_1));
        
        pub_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/perception/tracked_objects", 10);
        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/perception/object_markers", 10);

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Perception Node 8: 시작점-끝점 거리(최대 50cm) 기반 초경량 필터 적용 완료!");
    }

private:
    // ==========================================================
    // ⭐ [파라미터 설정 구역]
    // ==========================================================
    
    // 1. 관심 영역 (ROI) 설정
    const float ROI_X_MIN = 0.0f;
    const float ROI_X_MAX = 3.0f;
    const float ROI_Y_MIN = -1.0f;
    const float ROI_Y_MAX = 1.0f;

    // 2. ⭐ [핵심 최적화 파라미터]
    const int MIN_CLUSTER_PTS = 4;     // 최소 점 개수 (4개 이하는 노이즈로 무시)
    const float MAX_OBJ_WIDTH = 0.5f;  // 시작점과 끝점의 최대 허용 거리 (50cm 이상은 벽으로 무시)

    // 3. 이동 평균 및 데드존
    const size_t MA_WINDOW = 15;
    const float DEADZONE_DIST = 0.0f;
    const float ZERO_CLAMPING = 0.0f;

    // 4. 추적 및 병합 설정
    const float ALPHA_POS = 0.85f;
    const float MERGE_THRESHOLD = 0.30f;
    const float LIDAR_OFFSET_X = -0.05f;
    // ==========================================================

    float get_adaptive_threshold(float dist) {
        if (dist < 1.5f) return 0.18f;
        return 0.28f;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        rclcpp::Time curr_time = this->now();
        double dt = (curr_time - last_time_).seconds();
        if (dt <= 0 || dt > 0.5) { last_time_ = curr_time; return; }

        std::vector<ClusterPt> raw_clusters; 
        std::vector<std::pair<float, float>> current_group;

        // 1차 클러스터링
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];
            if (std::isnan(r) || std::isinf(r) || r <= 0.1f) continue; 

            float angle = msg->angle_min + i * msg->angle_increment;
            if (angle < -M_PI / 2.0f || angle > M_PI / 2.0f) continue; 

            float lx = r * std::cos(angle);
            float ly = r * std::sin(angle);

            float bx = lx + LIDAR_OFFSET_X; 
            float by = ly;                  

            if (bx < ROI_X_MIN || bx > ROI_X_MAX || by < ROI_Y_MIN || by > ROI_Y_MAX) continue;

            std::pair<float, float> current_pt = {bx, by};

            if (current_group.empty()) {
                current_group.push_back(current_pt);
            } else {
                auto& last_pt = current_group.back();
                float dist = std::hypot(current_pt.first - last_pt.first, current_pt.second - last_pt.second);
                float threshold = get_adaptive_threshold(r); 

                if (dist < threshold) {
                    current_group.push_back(current_pt);
                } else {
                    int pts_count = current_group.size();
                    if (pts_count >= MIN_CLUSTER_PTS) {
                        // ⭐ [핵심 로직 1] 그룹이 끝났을 때 첫 점과 끝 점의 거리를 측정
                        float width = std::hypot(current_group.front().first - current_group.back().first,
                                                 current_group.front().second - current_group.back().second);
                        
                        if (width <= MAX_OBJ_WIDTH) { // 50cm 이하일 때만 등록
                            float sum_x = 0, sum_y = 0;
                            for (const auto& p : current_group) { sum_x += p.first; sum_y += p.second; }
                            raw_clusters.push_back({sum_x, sum_y, pts_count, 
                                                    current_group.front().first, current_group.front().second, 
                                                    current_group.back().first, current_group.back().second});
                        }
                    }
                    current_group.clear();
                    current_group.push_back(current_pt);
                }
            }
        }

        // 마지막으로 남은 그룹 처리
        int pts_count = current_group.size();
        if (pts_count >= MIN_CLUSTER_PTS) {
            float width = std::hypot(current_group.front().first - current_group.back().first,
                                     current_group.front().second - current_group.back().second);
            if (width <= MAX_OBJ_WIDTH) {
                float sum_x = 0, sum_y = 0;
                for (const auto& p : current_group) { sum_x += p.first; sum_y += p.second; }
                raw_clusters.push_back({sum_x, sum_y, pts_count, 
                                        current_group.front().first, current_group.front().second, 
                                        current_group.back().first, current_group.back().second});
            }
        }

        std::vector<std::pair<float, float>> current_clusters;
        std::vector<bool> merged(raw_clusters.size(), false);

        // 2차 병합 (Merge)
        for (size_t i = 0; i < raw_clusters.size(); ++i) {
            if (merged[i]) continue;
            
            ClusterPt combined = raw_clusters[i]; 

            for (size_t j = i + 1; j < raw_clusters.size(); ++j) {
                if (merged[j]) continue;
                
                float cx1 = combined.sum_x / combined.count;
                float cy1 = combined.sum_y / combined.count;
                float cx2 = raw_clusters[j].sum_x / raw_clusters[j].count;
                float cy2 = raw_clusters[j].sum_y / raw_clusters[j].count;
                
                float d = std::hypot(cx1 - cx2, cy1 - cy2);
                
                if (d < MERGE_THRESHOLD) {
                    combined.sum_x += raw_clusters[j].sum_x;
                    combined.sum_y += raw_clusters[j].sum_y;
                    combined.count += raw_clusters[j].count;
                    // ⭐ [핵심 로직 2] 병합될 때마다 '끝점'을 합쳐진 녀석의 끝점으로 갱신
                    combined.last_x = raw_clusters[j].last_x;
                    combined.last_y = raw_clusters[j].last_y;
                    merged[j] = true;
                }
            }
            
            // ⭐ [최종 확인] 여러 개가 뭉쳐서 50cm가 넘는 거대한 벽이 되었다면 버림!
            float final_width = std::hypot(combined.first_x - combined.last_x, combined.first_y - combined.last_y);
            if (final_width <= MAX_OBJ_WIDTH) {
                current_clusters.push_back({combined.sum_x / combined.count, combined.sum_y / combined.count});
            }
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
                float d = std::hypot(obj.x - clusters[i].first, obj.y - clusters[i].second);
                if (d < min_d) { min_d = d; best_idx = i; }
            }
            
            if (best_idx != -1) {
                float dx = clusters[best_idx].first - obj.x;
                float dy = clusters[best_idx].second - obj.y;
                float dist_diff = std::hypot(dx, dy);

                float raw_vx = 0.0f, raw_vy = 0.0f;
                if (dist_diff > DEADZONE_DIST) { raw_vx = dx / dt; raw_vy = dy / dt; }

                obj.hist_vx.push_back(raw_vx); obj.hist_vy.push_back(raw_vy);
                if (obj.hist_vx.size() > MA_WINDOW) { obj.hist_vx.pop_front(); obj.hist_vy.pop_front(); }

                float sum_vx = 0.0f, sum_vy = 0.0f;
                for (float v : obj.hist_vx) sum_vx += v;
                for (float v : obj.hist_vy) sum_vy += v;

                obj.vx = sum_vx / obj.hist_vx.size(); obj.vy = sum_vy / obj.hist_vy.size();

                if (std::hypot(obj.vx, obj.vy) < ZERO_CLAMPING) { obj.vx = 0.0f; obj.vy = 0.0f; }

                obj.x = (1.0f - ALPHA_POS) * obj.x + ALPHA_POS * clusters[best_idx].first;
                obj.y = (1.0f - ALPHA_POS) * obj.y + ALPHA_POS * clusters[best_idx].second;

                obj.age++; obj.miss_count = 0; matched[best_idx] = true;
            } else { 
                obj.hist_vx.push_back(0.0f); obj.hist_vy.push_back(0.0f);
                if (obj.hist_vx.size() > MA_WINDOW) { obj.hist_vx.pop_front(); obj.hist_vy.pop_front(); }
                obj.miss_count++; 
            }
        }
        
        for (size_t i = 0; i < clusters.size(); ++i) {
            if (!matched[i]) {
                std::deque<float> init_zeros(MA_WINDOW, 0.0f);
                tracked_objects_.push_back({
                    next_id_++, clusters[i].first, clusters[i].second, 0.0f, 0.0f, 1, 0, init_zeros, init_zeros
                });
            }
        }
        tracked_objects_.erase(std::remove_if(tracked_objects_.begin(), tracked_objects_.end(),
            [](const TrackedObject& o){ return o.miss_count > 3; }), tracked_objects_.end());
    }

    void publish_data(const rclcpp::Time& t) {
        geometry_msgs::msg::PoseArray pose_array;
        visualization_msgs::msg::MarkerArray marker_array;
        
        pose_array.header.frame_id = "base_link"; 
        pose_array.header.stamp = t;

        visualization_msgs::msg::Marker delete_all;
        delete_all.header.frame_id = "base_link";
        delete_all.header.stamp = t;
        delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_all);

        for (const auto& obj : tracked_objects_) {
            if (obj.age < 3 || obj.miss_count > 0) continue;

            float speed = std::hypot(obj.vx, obj.vy);
            float yaw = std::atan2(obj.vy, obj.vx);

            geometry_msgs::msg::Pose data_pose;
            data_pose.position.x = obj.x; data_pose.position.y = obj.y; data_pose.position.z = speed; 
            data_pose.orientation.x = static_cast<double>(obj.id); 
            data_pose.orientation.z = std::sin(yaw / 2.0); data_pose.orientation.w = std::cos(yaw / 2.0);
            pose_array.poses.push_back(data_pose);

            auto create_marker = [&](std::string ns, int id_offset, int type) {
                visualization_msgs::msg::Marker m;
                m.header.frame_id = "base_link"; m.header.stamp = t; m.ns = ns; m.id = obj.id + id_offset; 
                m.type = type; m.action = visualization_msgs::msg::Marker::ADD;
                m.pose.position.x = obj.x; m.pose.position.y = obj.y; m.pose.position.z = 0.0;
                m.pose.orientation.w = 1.0; m.lifetime = rclcpp::Duration::from_seconds(0.2); 
                return m;
            };

            auto arrow = create_marker("velocity", 2000, visualization_msgs::msg::Marker::ARROW);
            arrow.pose.orientation.z = std::sin(yaw / 2.0); arrow.pose.orientation.w = std::cos(yaw / 2.0);
            arrow.scale.x = std::max(0.01f, std::min(speed * 0.3f, 0.7f)); 
            arrow.scale.y = 0.05; arrow.scale.z = 0.05; arrow.color.r = 1.0; arrow.color.a = 1.0;
            marker_array.markers.push_back(arrow);

            auto cube = create_marker("boxes", 0, visualization_msgs::msg::Marker::CUBE);
            cube.pose.position.z = 0.1; cube.scale.x = 0.2; cube.scale.y = 0.2; cube.scale.z = 0.2;
            cube.color.g = 1.0; cube.color.a = 0.6; marker_array.markers.push_back(cube);

            auto text = create_marker("ids", 1000, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
            text.pose.position.z = 0.4; text.scale.z = 0.25;
            text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0;
            text.text = std::to_string(obj.id); marker_array.markers.push_back(text);
        }
        pub_poses_->publish(pose_array); pub_markers_->publish(marker_array);
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
    rclcpp::spin(std::make_shared<PerceptionNode8>());
    rclcpp::shutdown();
    return 0;
}
