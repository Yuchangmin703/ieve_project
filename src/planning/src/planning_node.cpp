#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp> 
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <cmath>
#include <limits>
#include <iomanip> 
#include <algorithm> 

#include "perception/msg/lanes.hpp" 

using namespace std;

struct Point2D {
    float x;
    float y;
    bool visited;
    float curvature; // 각 점의 목표 속도를 저장 (곡률 기반)
};

struct TrackedObstacle {
    int id;       
    float x;      
    float y;      
    float speed;  
};

enum DrivingState {
    NORMAL,         
    AVOIDING,       
    FOLLOWING,      
    EMERGENCY_STOP  
};

class RacePlannerNode : public rclcpp::Node {
public:
    RacePlannerNode() : Node("race_planner_node") {
        
        // ==========================================
        // 🛠️ [고속 레이싱 파라미터 튜닝]
        // ==========================================
        v_max_ = 1.0f; 
        v_min_ = 0.5f; 

        max_search_radius_ = 0.4f; 
        
        avoidance_trigger_dist_ = 2.0f; // 2m 앞에서 고속 회피 시작
        avoidance_time_sec_ = 1.0; 
        tentacle_length_ = 1.0f; // S자 궤적 길이 1.0m로 확장

        curvature_speed_gain_ = 3.0f; // 곡선 감속 민감도
        
        lidar_to_base_offset_x_ = 0.0;  
        // ==========================================

        current_state_ = NORMAL;
        avoidance_direction_ = 1; 

        lanes_sub_ = this->create_subscription<perception::msg::Lanes>(
            "/perception/lane/lanes", 10, 
            std::bind(&RacePlannerNode::lanes_callback, this, std::placeholders::_1));
        
        obs_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/perception/tracked_objects", 10,
            std::bind(&RacePlannerNode::obs_callback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/local_path", 10);
        car_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/ego_car", 10);
        path_vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/path_vis", 10); 
        avoid_vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/avoidance_path", 10);
        speed_text_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/speed_text", 10); 

        RCLCPP_INFO(this->get_logger(), "🏁 [Planner] 버블 완전 삭제 & 고속 곡률 제어 플래너 가동!");
    }

private:
    void obs_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        obstacles_.clear();
        for (const auto& pose : msg->poses) {
            TrackedObstacle obs;
            obs.x = pose.position.x + lidar_to_base_offset_x_;
            obs.y = pose.position.y;
            obs.speed = 0.0f; 
            obstacles_.push_back(obs);
        }
    }

    float get_y_from_line(const vector<Point2D>& line, float target_x) {
        if (line.empty()) return 0.0f;
        if (target_x <= line.front().x) return line.front().y;
        if (target_x >= line.back().x) return line.back().y;
        
        for (size_t i = 0; i < line.size() - 1; ++i) {
            if (line[i].x <= target_x && line[i+1].x >= target_x) {
                float dx = line[i+1].x - line[i].x;
                if (std::abs(dx) < 1e-5) return line[i].y;
                return line[i].y + (target_x - line[i].x) / dx * (line[i+1].y - line[i].y);
            }
        }
        return line.back().y;
    }

    // ⭐ 동적 속도 프로파일링 (곡률 기반 미리 감속)
    void assign_adaptive_speed(vector<Point2D>& path_points) {
        if (path_points.size() < 3) {
            for(auto& p : path_points) p.curvature = v_min_;
            return;
        }

        path_points[0].curvature = v_max_; 

        for (size_t i = 1; i < path_points.size() - 1; ++i) {
            float dx1 = path_points[i].x - path_points[i-1].x;
            float dy1 = path_points[i].y - path_points[i-1].y;
            float dx2 = path_points[i+1].x - path_points[i].x;
            float dy2 = path_points[i+1].y - path_points[i].y;

            if (std::abs(dx1) < 1e-5 || std::abs(dx2) < 1e-5) {
                path_points[i].curvature = v_min_; continue;
            }

            float slope1 = std::atan2(dy1, dx1);
            float slope2 = std::atan2(dy2, dx2);
            float angle_diff = std::abs(slope2 - slope1);

            float reduction_ratio = std::exp(-curvature_speed_gain_ * angle_diff); 
            float target_v = v_min_ + (v_max_ - v_min_) * reduction_ratio;
            
            path_points[i].curvature = target_v; 
        }
        path_points.back().curvature = path_points[path_points.size()-2].curvature; 
    }

    // 🦋 S-Curve 나비 생성 및 충돌 검사
    bool generate_tentacle(const vector<Point2D>& ego_line, const vector<Point2D>& target_line, vector<Point2D>& out_path) {
        out_path.clear();
        out_path.push_back({0.0f, 0.0f, true, v_max_}); 

        bool collision = false;
        float ego_v = std::max(v_max_, 0.1f);

        for (float x = 0.05f; x <= 3.0f; x += 0.05f) {
            float sy = get_y_from_line(ego_line, x);
            float ty = get_y_from_line(target_line, x);
            
            float progress = std::clamp(x / tentacle_length_, 0.0f, 1.0f);
            float dodge_blend = 0.5f * (1.0f - std::cos(progress * M_PI));
            float path_y = sy + (ty - sy) * dodge_blend;

            float time_to_reach = x / ego_v;
            for (const auto& obs : obstacles_) {
                float future_obs_x = obs.x + (obs.speed * time_to_reach);
                if (future_obs_x > -0.2f && future_obs_x < 3.0f) {
                    if (std::hypot(future_obs_x - x, obs.y - path_y) < 0.35f) { 
                        collision = true; break;
                    }
                }
            }
            if (collision) break; 
            out_path.push_back({x, path_y, true, 0.0f}); 
        }

        if (collision) { out_path.clear(); return false; }
        
        assign_adaptive_speed(out_path); 
        return true; 
    }

    void lanes_callback(const perception::msg::Lanes::SharedPtr msg) {
        publish_car_marker();

        // ---------------------------------------------------------------------
        // 🟢 1. 동적 차선 분류 (버블 로직 완전 삭제 🗑️)
        // ---------------------------------------------------------------------
        vector<Point2D> my_lane_points, left_lane_points, right_lane_points;
        int closest_lane_idx = -1;
        float min_dist_to_car = numeric_limits<float>::max();

        for (size_t i = 0; i < msg->lanes.size(); ++i) {
            if (msg->lanes[i].points.empty()) continue;
            float dist = hypot(msg->lanes[i].points.front().x, msg->lanes[i].points.front().y);
            if (dist < min_dist_to_car) { min_dist_to_car = dist; closest_lane_idx = i; }
        }

        if (closest_lane_idx != -1) {
            float my_lane_y_ref = msg->lanes[closest_lane_idx].points.front().y;
            for (size_t i = 0; i < msg->lanes.size(); ++i) {
                const auto& lane = msg->lanes[i]; if (lane.points.empty()) continue;
                float current_lane_y_ref = lane.points.front().y;
                for (const auto& pt : lane.points) {
                    if (pt.x > 0.0f) {
                        Point2D p = {(float)pt.x, (float)pt.y, false, 0.0f};
                        if (i == closest_lane_idx) my_lane_points.push_back(p);      
                        else if (current_lane_y_ref > my_lane_y_ref) left_lane_points.push_back(p);    
                        else right_lane_points.push_back(p);   
                    }
                }
            }
        }

        if (my_lane_points.empty()) return;

        // ---------------------------------------------------------------------
        // 🔴 2. 상태 유지 (AVOIDING)
        // ---------------------------------------------------------------------
        if (current_state_ == AVOIDING) {
            double elapsed_time = (this->now() - avoidance_start_time_).seconds();
            if (elapsed_time > avoidance_time_sec_) {
                current_state_ = NORMAL; 
            } else {
                vector<Point2D> target_lane = (avoidance_direction_ == 1) ? left_lane_points : right_lane_points;
                if (target_lane.empty()) {
                    for(const auto& p : my_lane_points) target_lane.push_back({p.x, p.y + (avoidance_direction_ * 0.45f), false, 0.0f});
                }
                
                vector<Point2D> out_path;
                if (generate_tentacle(my_lane_points, target_lane, out_path)) {
                    publish_avoidance_path_visualization(out_path);
                    publish_path_msg(out_path, msg->header.stamp);
                    publish_speed_text(out_path[0].curvature); 
                    return; 
                } else {
                    current_state_ = NORMAL; 
                }
            }
        }

        // ---------------------------------------------------------------------
        // ⭐ 3. 라이다 주도 전방 감시
        // ---------------------------------------------------------------------
        float min_obs_x = numeric_limits<float>::max();
        bool obstacle_in_my_lane = false;
        for (const auto& obs : obstacles_) {
            if (obs.x > 0.0f && std::abs(obs.y) < 0.25f) { 
                if (obs.x < min_obs_x) { min_obs_x = obs.x; obstacle_in_my_lane = true; }
            }
        }

        // 🚨 4. 비상 상황 분기
        if (obstacle_in_my_lane && min_obs_x <= avoidance_trigger_dist_) {
            if (min_obs_x <= 1.0f) {
                current_state_ = EMERGENCY_STOP;
                publish_straight_path(msg->header.stamp, 0.0f); 
                publish_speed_text(0.0f);
                return;
            }

            vector<Point2D> left_path, right_path;
            bool left_safe = generate_tentacle(my_lane_points, left_lane_points, left_path);
            bool right_safe = generate_tentacle(my_lane_points, right_lane_points, right_path);

            if (left_safe) {
                current_state_ = AVOIDING; avoidance_direction_ = 1; avoidance_start_time_ = this->now();
                RCLCPP_WARN(this->get_logger(), "🦋 좌측 나비 채택! 1.0m/s S-Curve 회피 시작!");
                publish_avoidance_path_visualization(left_path);
                publish_path_msg(left_path, msg->header.stamp);
                return;
            } else if (right_safe) {
                current_state_ = AVOIDING; avoidance_direction_ = -1; avoidance_start_time_ = this->now();
                RCLCPP_WARN(this->get_logger(), "🦋 우측 나비 채택! 1.0m/s S-Curve 회피 시작!");
                publish_avoidance_path_visualization(right_path);
                publish_path_msg(right_path, msg->header.stamp);
                return;
            } else {
                current_state_ = FOLLOWING; 
                publish_straight_path(msg->header.stamp, v_min_); publish_speed_text(v_min_);
                return;
            }
        }

        // ---------------------------------------------------------------------
        // 🟢 5. 평상시 주행 궤적 생성 (원점 연결)
        // ---------------------------------------------------------------------
        std::sort(my_lane_points.begin(), my_lane_points.end(), [](const Point2D& a, const Point2D& b) {
            return a.x < b.x;
        });

        vector<Point2D> final_path;
        final_path.push_back({0.0f, 0.0f, true, 0.0f}); // 무조건 원점 시작

        my_lane_points[0].visited = true; final_path.push_back(my_lane_points[0]);
        Point2D current_point = my_lane_points[0]; int current_search_start_idx = 0;

        while (true) {
            int next_idx = -1; float min_dist = numeric_limits<float>::max();
            for (size_t i = current_search_start_idx; i < my_lane_points.size(); ++i) {
                if (my_lane_points[i].visited) continue;
                if (my_lane_points[i].x - current_point.x > max_search_radius_) break; 
                float dist = hypot(my_lane_points[i].x - current_point.x, my_lane_points[i].y - current_point.y);
                if (dist < max_search_radius_ && dist < min_dist) { min_dist = dist; next_idx = i; }
            }
            if (next_idx == -1) break;
            my_lane_points[next_idx].visited = true; final_path.push_back(my_lane_points[next_idx]);
            current_point = my_lane_points[next_idx]; current_search_start_idx = next_idx + 1;
        }

        // ⚡ 곡률 기반 목표 속도 프로파일링 적용
        assign_adaptive_speed(final_path);

        publish_path_visualization(final_path, v_min_, v_max_);
        publish_path_msg(final_path, msg->header.stamp);
        publish_speed_text(final_path[0].curvature); 
    }

    void publish_straight_path(const builtin_interfaces::msg::Time& stamp, float speed) {
        vector<Point2D> straight_path;
        for (float dist = 0.0f; dist <= 3.0f; dist += 0.2f) {
            straight_path.push_back({dist, 0.0f, true, speed}); 
        }
        publish_path_visualization(straight_path, v_min_, v_max_);
        publish_path_msg(straight_path, stamp);
    }

    // ⚡ Path 메시지에 속도(z)와 부드러운 헤딩(yaw) 덮어씌우기
    void publish_path_msg(const vector<Point2D>& path_points, const builtin_interfaces::msg::Time& stamp) {
        if (path_points.empty()) return;
        
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = stamp;
        path_msg.header.frame_id = "base_link";

        for (size_t i = 0; i < path_points.size(); ++i) {
            const auto& pt = path_points[i];
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = pt.x;
            pose.pose.position.y = pt.y;
            // pt.curvature에 계산된 목표 속도(1.0~0.5)가 들어있음
            pose.pose.position.z = std::clamp(pt.curvature, 0.0f, v_max_); 
            
            if (i < path_points.size() - 1) {
                float dx = path_points[i+1].x - pt.x;
                float dy = path_points[i+1].y - pt.y;
                float yaw = std::atan2(dy, dx);
                pose.pose.orientation.z = std::sin(yaw / 2.0f);
                pose.pose.orientation.w = std::cos(yaw / 2.0f);
            } else if (i > 0) {
                pose.pose.orientation = path_msg.poses.back().pose.orientation;
            } else { pose.pose.orientation.w = 1.0; }
            
            path_msg.poses.push_back(pose);
        }
        path_pub_->publish(path_msg);
    }

    // 🦋 나비 S-Curve 궤적 시각화 (주황색)
    void publish_avoidance_path_visualization(const vector<Point2D>& path) {
        if (path.empty()) return;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link"; marker.header.stamp = this->now();
        marker.ns = "avoidance_path"; marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP; marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.08; 
        marker.pose.orientation.w = 1.0;
        marker.color.r = 1.0f; marker.color.g = 0.6f; marker.color.b = 0.0f; marker.color.a = 1.0f; 

        for (const auto& pt : path) {
            geometry_msgs::msg::Point p; p.x = pt.x; p.y = pt.y; p.z = 0.0;
            marker.points.push_back(p);
        }
        marker.lifetime = rclcpp::Duration::from_seconds(0.2); 
        avoid_vis_pub_->publish(marker);
    }

    void publish_speed_text(float speed) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link"; marker.header.stamp = this->now();
        marker.ns = "speed_text"; marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING; marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.z = 0.15; 
        marker.color.r = 1.0f; marker.color.g = 1.0f; marker.color.b = 1.0f; marker.color.a = 1.0f; 
        marker.pose.position.x = -0.6; marker.pose.position.y = 0.0; marker.pose.position.z = 0.2; 
        marker.pose.orientation.w = 1.0;
        
        string state_str = "NORMAL_ADAPTIVE"; 
        if (current_state_ == AVOIDING) state_str = "S-CURVE_DODGE";
        else if (current_state_ == FOLLOWING) state_str = "FOLLOWING";
        else if (current_state_ == EMERGENCY_STOP) state_str = "EMERG_STOP";

        stringstream ss; ss << "Mode: " << state_str << "\nSpd: " << fixed << setprecision(2) << speed << "m/s";
        marker.text = ss.str(); marker.lifetime = rclcpp::Duration::from_seconds(0.2); 
        speed_text_pub_->publish(marker);
    }

    void publish_path_visualization(const vector<Point2D>& path, float v_min, float v_max) {
        if (path.empty()) return;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link"; marker.header.stamp = this->now();
        marker.ns = "path_vis"; marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP; marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05; marker.pose.orientation.w = 1.0;

        for (const auto& pt : path) {
            geometry_msgs::msg::Point p; p.x = pt.x; p.y = pt.y; p.z = 0.0;
            marker.points.push_back(p);
            if (current_state_ == EMERGENCY_STOP) { marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 1.0f; marker.color.a = 0.8f; }
            else if (current_state_ == FOLLOWING) { marker.color.r = 1.0f; marker.color.g = 0.5f; marker.color.b = 0.0f; marker.color.a = 0.8f; }
            else { 
                // 속도(curvature 필드)에 따라 초록색(빠름) -> 노란색(감속)으로 그라데이션 시각화
                float speed_ratio = (pt.curvature - v_min) / (v_max - v_min);
                marker.color.r = 1.0f - speed_ratio; 
                marker.color.g = speed_ratio; 
                marker.color.b = 0.0f; marker.color.a = 0.7f; 
            }
        }
        marker.lifetime = rclcpp::Duration::from_seconds(0.2); path_vis_pub_->publish(marker);
    }

    void publish_car_marker() {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link"; marker.header.stamp = this->now();
        marker.ns = "ego_car"; marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE; marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.4; marker.scale.y = 0.2; marker.scale.z = 0.25;
        marker.color.r = 0.1f; marker.color.g = 0.5f; marker.color.b = 1.0f; marker.color.a = 0.7f; 
        marker.pose.position.x = -0.2; marker.pose.position.y = 0.0; marker.pose.position.z = 0.125; 
        marker.pose.orientation.w = 1.0;
        car_marker_pub_->publish(marker);
    }

    rclcpp::Subscription<perception::msg::Lanes>::SharedPtr lanes_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obs_sub_; 
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr car_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_vis_pub_; 
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr avoid_vis_pub_; 
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr speed_text_pub_; 
    
    std::vector<TrackedObstacle> obstacles_; 
    
    DrivingState current_state_;
    rclcpp::Time avoidance_start_time_;
    int avoidance_direction_;

    float v_max_, v_min_;
    float max_search_radius_;
    float avoidance_trigger_dist_;
    double avoidance_time_sec_;
    float tentacle_length_;
    float curvature_speed_gain_;
    float lidar_to_base_offset_x_; 
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<RacePlannerNode>());
    rclcpp::shutdown();
    return 0;
}
