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

class PerceptionNode8 : public rclcpp::Node {
public:
    PerceptionNode8() : Node("perception_node8"), next_id_(0) {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&PerceptionNode8::lidar_callback, this, std::placeholders::_1));
        
        pub_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/perception/tracked_objects", 10);
        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/perception/object_markers", 10);

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Perception Node 8: ROI 파라미터화 & 15프레임 웜업(Warm-up) 필터 적용 완료");
    }

private:
    // ==========================================================
    // ⭐ [파라미터 설정 구역] 언제든지 여기서 값을 쉽게 변경하세요!
    // ==========================================================
    
    // 1. 관심 영역 (ROI) 설정 (단위: m, 기준: 앞범퍼 base_link)
    const float ROI_X_MIN = 0.0f;   // 앞범퍼부터
    const float ROI_X_MAX = 3.0f;   // 전방 3m까지
    const float ROI_Y_MIN = -1.0f;  // 우측 10cm부터
    const float ROI_Y_MAX = 1.0f;   // 좌측 10cm까지

    // 2. 필터 및 데드존 설정 (현재 테스트를 위해 0.0)
    const size_t MA_WINDOW = 15;       // 이동 평균 필터 프레임 수 (15프레임 = 약 1.5초)
    const float DEADZONE_DIST = 0.0f;  // 위치 이동 데드존
    const float ZERO_CLAMPING = 0.0f;  // 최종 속도 고정 데드존

    // 3. 기타 하드웨어/병합 설정
    const float ALPHA_POS = 0.85f;       // 위치 추적(부드러운 움직임) 비율
    const float MERGE_THRESHOLD = 0.30f; // 이 거리 안의 물체는 하나로 합침
    const float LIDAR_OFFSET_X = -0.05f; // 라이다가 앞범퍼보다 5cm 뒤에 있음
    // ==========================================================

    float get_adaptive_threshold(float dist) {
        if (dist < 1.5f) return 0.18f;
        return 0.28f;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        rclcpp::Time curr_time = this->now();
        double dt = (curr_time - last_time_).seconds();
        if (dt <= 0 || dt > 0.5) { last_time_ = curr_time; return; }

        std::vector<std::pair<float, float>> raw_clusters;
        std::vector<std::pair<float, float>> current_group;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];
            
            if (std::isnan(r) || std::isinf(r) || r <= 0.1f) {
                continue; 
            }

            float angle = msg->angle_min + i * msg->angle_increment;
            
            if (angle < -M_PI / 2.0f || angle > M_PI / 2.0f) {
                continue; 
            }

            float lx = r * std::cos(angle);
            float ly = r * std::sin(angle);

            float bx = lx + LIDAR_OFFSET_X; 
            float by = ly;                  

            // ⭐ [수정] 변수화된 ROI 적용
            if (bx < ROI_X_MIN || bx > ROI_X_MAX || by < ROI_Y_MIN || by > ROI_Y_MAX) {
                continue;
            }

            std::pair<float, float> current_pt = {bx, by};

            if (current_group.empty()) {
                current_group.push_back(current_pt);
            } else {
                auto& last_pt = current_group.back();
                float dist = std::sqrt(std::pow(current_pt.first - last_pt.first, 2) + 
                                       std::pow(current_pt.second - last_pt.second, 2));
                
                float threshold = get_adaptive_threshold(r); 

                if (dist < threshold) {
                    current_group.push_back(current_pt);
                } else {
                    if (current_group.size() >= 4) {
                        float sum_x = 0, sum_y = 0;
                        for (const auto& p : current_group) {
                            sum_x += p.first; sum_y += p.second;
                        }
                        raw_clusters.push_back({sum_x / current_group.size(), sum_y / current_group.size()});
                    }
                    current_group.clear();
                    current_group.push_back(current_pt);
                }
            }
        }

        if (current_group.size() >= 4) {
            float sum_x = 0, sum_y = 0;
            for (const auto& p : current_group) {
                sum_x += p.first; sum_y += p.second;
            }
            raw_clusters.push_back({sum_x / current_group.size(), sum_y / current_group.size()});
        }

        std::vector<std::pair<float, float>> current_clusters;
        std::vector<bool> merged(raw_clusters.size(), false);

        for (size_t i = 0; i < raw_clusters.size(); ++i) {
            if (merged[i]) continue;
            float sum_x = raw_clusters[i].first;
            float sum_y = raw_clusters[i].second;
            int count = 1;

            for (size_t j = i + 1; j < raw_clusters.size(); ++j) {
                if (merged[j]) continue;
                
                float d = std::sqrt(std::pow(raw_clusters[i].first - raw_clusters[j].first, 2) + 
                                   std::pow(raw_clusters[i].second - raw_clusters[j].second, 2));
                
                if (d < MERGE_THRESHOLD) {
                    sum_x += raw_clusters[j].first;
                    sum_y += raw_clusters[j].second;
                    count++;
                    merged[j] = true;
                }
            }
            current_clusters.push_back({sum_x / count, sum_y / count});
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
                float dx = clusters[best_idx].first - obj.x;
                float dy = clusters[best_idx].second - obj.y;
                float dist_diff = std::sqrt(dx * dx + dy * dy);

                float raw_vx = 0.0f;
                float raw_vy = 0.0f;

                if (dist_diff > DEADZONE_DIST) {
                    raw_vx = dx / dt;
                    raw_vy = dy / dt;
                }

                obj.hist_vx.push_back(raw_vx); 
                obj.hist_vy.push_back(raw_vy);

                // ⭐ [수정] 변수화된 MA_WINDOW 사용
                if (obj.hist_vx.size() > MA_WINDOW) { 
                    obj.hist_vx.pop_front();
                    obj.hist_vy.pop_front();
                }

                float sum_vx = 0.0f, sum_vy = 0.0f;
                for (float v : obj.hist_vx) sum_vx += v;
                for (float v : obj.hist_vy) sum_vy += v;

                obj.vx = sum_vx / obj.hist_vx.size();
                obj.vy = sum_vy / obj.hist_vy.size();

                if (std::sqrt(obj.vx * obj.vx + obj.vy * obj.vy) < ZERO_CLAMPING) {
                    obj.vx = 0.0f;
                    obj.vy = 0.0f;
                }

                obj.x = (1.0f - ALPHA_POS) * obj.x + ALPHA_POS * clusters[best_idx].first;
                obj.y = (1.0f - ALPHA_POS) * obj.y + ALPHA_POS * clusters[best_idx].second;

                obj.age++; obj.miss_count = 0; matched[best_idx] = true;
            } else { 
                obj.hist_vx.push_back(0.0f);
                obj.hist_vy.push_back(0.0f);
                if (obj.hist_vx.size() > MA_WINDOW) { obj.hist_vx.pop_front(); obj.hist_vy.pop_front(); }
                
                obj.miss_count++; 
            }
        }
        
        for (size_t i = 0; i < clusters.size(); ++i) {
            if (!matched[i]) {
                // ⭐ [핵심 수정] 물체가 처음 태어날 때 버퍼를 0.0으로 15개 꽉 채워둠 (Zero-padding)
                std::deque<float> init_zeros(MA_WINDOW, 0.0f);
                
                tracked_objects_.push_back({
                    next_id_++, 
                    clusters[i].first, clusters[i].second, 
                    0.0f, 0.0f, 
                    1, 0,
                    init_zeros, init_zeros // 꽉 찬 0.0 버퍼 투입!
                });
            }
        }
        tracked_objects_.erase(std::remove_if(tracked_objects_.begin(), tracked_objects_.end(),
            [](const TrackedObject& o){ return o.miss_count > 3; }), tracked_objects_.end());
    }

    // ... (publish_data 함수는 이전과 완벽히 동일하므로 유지) ...
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

            float speed = std::sqrt(obj.vx * obj.vx + obj.vy * obj.vy);
            float yaw = std::atan2(obj.vy, obj.vx);

            geometry_msgs::msg::Pose data_pose;
            data_pose.position.x = obj.x;
            data_pose.position.y = obj.y;
            data_pose.position.z = speed; 
            data_pose.orientation.x = static_cast<double>(obj.id); 
            data_pose.orientation.z = std::sin(yaw / 2.0);
            data_pose.orientation.w = std::cos(yaw / 2.0);
            pose_array.poses.push_back(data_pose);

            auto create_marker = [&](std::string ns, int id_offset, int type) {
                visualization_msgs::msg::Marker m;
                m.header.frame_id = "base_link"; 
                m.header.stamp = t;
                m.ns = ns; 
                m.id = obj.id + id_offset; 
                m.type = type; 
                m.action = visualization_msgs::msg::Marker::ADD;
                m.pose.position.x = obj.x; 
                m.pose.position.y = obj.y;
                m.pose.position.z = 0.0;
                m.pose.orientation.w = 1.0;
                m.lifetime = rclcpp::Duration::from_seconds(0.2); 
                return m;
            };

            auto arrow = create_marker("velocity", 2000, visualization_msgs::msg::Marker::ARROW);
            arrow.pose.orientation.z = std::sin(yaw / 2.0); 
            arrow.pose.orientation.w = std::cos(yaw / 2.0);
            arrow.scale.x = std::max(0.01f, std::min(speed * 0.3f, 0.7f)); 
            arrow.scale.y = 0.05; arrow.scale.z = 0.05;
            arrow.color.r = 1.0; arrow.color.a = 1.0;
            marker_array.markers.push_back(arrow);

            auto cube = create_marker("boxes", 0, visualization_msgs::msg::Marker::CUBE);
            cube.pose.position.z = 0.1; 
            cube.scale.x = 0.2; cube.scale.y = 0.2; cube.scale.z = 0.2;
            cube.color.g = 1.0; cube.color.a = 0.6;
            marker_array.markers.push_back(cube);

            auto text = create_marker("ids", 1000, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
            text.pose.position.z = 0.4; 
            text.scale.z = 0.25;
            text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0;
            text.text = std::to_string(obj.id);
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
    rclcpp::spin(std::make_shared<PerceptionNode8>());
    rclcpp::shutdown();
    return 0;
}
