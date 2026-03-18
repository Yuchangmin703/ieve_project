#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp> 
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include <cmath>
#include <limits>
#include <iomanip> 
#include <algorithm> // ⭐ std::sort 사용을 위해 추가!

#include "perception/msg/lanes.hpp" 

using namespace std;

// 차선 점을 위한 구조체
struct Point2D {
    float x;
    float y;
    bool visited;
};

// 라이다 인지 노드(perception_node8)의 포맷을 완벽하게 담을 장애물 구조체
struct TrackedObstacle {
    int id;       
    float x;      
    float y;      
    float speed;  
    float yaw;    
};

class RacePlannerNode : public rclcpp::Node {
public:
    RacePlannerNode() : Node("race_planner_node") {
        lanes_sub_ = this->create_subscription<perception::msg::Lanes>(
            "/perception/lane/lanes", 10, 
            std::bind(&RacePlannerNode::lanes_callback, this, std::placeholders::_1));
        
        obs_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/perception/tracked_objects", 10,
            std::bind(&RacePlannerNode::obs_callback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/local_path", 10);
        
        car_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/ego_car", 10);
        path_vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/path_vis", 10); 
        speed_text_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/speed_text", 10); 

        target_speed_ = 1.0;
        
        max_search_radius_ = 0.1;      
        jump_search_radius_ = 0.6;     
        bubble_a_radius_ = 0.1;    
        bubble_b_radius_ = 0.3;    
        
        lidar_to_base_offset_x_ = 0.0;  

        RCLCPP_INFO(this->get_logger(), "🏁 [Planner] O(N) 최적화 & 고정 스킵(70) & 동적 속도 제어 실행 중!");
    }

private:
    void obs_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        obstacles_.clear();
        for (const auto& pose : msg->poses) {
            TrackedObstacle obs;
            obs.id = static_cast<int>(pose.orientation.x);
            obs.x = pose.position.x + lidar_to_base_offset_x_;
            obs.y = pose.position.y;
            obs.speed = pose.position.z;
            obs.yaw = 2.0 * std::atan2(pose.orientation.z, pose.orientation.w);
            obstacles_.push_back(obs);
        }
    }

    void lanes_callback(const perception::msg::Lanes::SharedPtr msg) {
        publish_car_marker();

        vector<Point2D> points;
        
        // 버블 A 로직 (장애물 주변 노이즈 제거)
        for (const auto& lane : msg->lanes) {
            for (const auto& pt : lane.points) {
                if (pt.x > 0.0) {
                    bool in_bubble_a = false;
                    for (const auto& obs : obstacles_) { 
                        if (hypot(pt.x - obs.x, pt.y - obs.y) < bubble_a_radius_) {
                            in_bubble_a = true;
                            break;
                        }
                    }
                    if (!in_bubble_a) { 
                        points.push_back({(float)pt.x, (float)pt.y, false});
                    }
                }
            }
        }

        if (points.empty()) return;

        // ========================================================
        // ⭐ 최적화 1단계: 점들을 X축 기준으로 정렬 (O(N log N))
        // ========================================================
        std::sort(points.begin(), points.end(), [](const Point2D& a, const Point2D& b) {
            return a.x < b.x;
        });

        vector<Point2D> final_path;
        int start_idx = -1;
        float min_dist_to_origin = numeric_limits<float>::max();

        // 시작점 찾기
        for (size_t i = 0; i < points.size(); ++i) {
            float dist = hypot(points[i].x, points[i].y);
            if (dist < min_dist_to_origin) {
                min_dist_to_origin = dist;
                start_idx = i;
            }
        }

        if (start_idx == -1) return;

        final_path.push_back(points[start_idx]);
        points[start_idx].visited = true;
        
        int current_idx = start_idx; // ⭐ 현재 인덱스 기억
        Point2D current_point = points[current_idx];

        float min_dist_to_P = 999.0f; 

        while (true) {
            int next_idx = -1;
            float min_dist = numeric_limits<float>::max();
            float current_search_radius = max_search_radius_; 

            // 버블 B 로직
            for (const auto& obs : obstacles_) {
                if (hypot(current_point.x - obs.x, current_point.y - obs.y) < bubble_b_radius_) {
                    
                    float dist_P = hypot(current_point.x, current_point.y);
                    if (dist_P < min_dist_to_P) {
                        min_dist_to_P = dist_P;
                    }

                    current_search_radius = jump_search_radius_; 
                    
                    // 버블 B 내부 점 지우기도 O(N) 최적화 반영 (앞으로만 탐색 & 조기 종료)
                    for (size_t p_idx = current_idx + 1; p_idx < points.size(); ++p_idx) {
                        if (points[p_idx].x - current_point.x > bubble_b_radius_ + 0.3f) break; // 멀어지면 즉시 중단!
                        
                        if (!points[p_idx].visited) {
                            if (hypot(points[p_idx].x - obs.x, points[p_idx].y - obs.y) < bubble_b_radius_ || 
                                std::abs(points[p_idx].y - current_point.y) < 0.25) { 
                                points[p_idx].visited = true;
                            }
                        }
                    }
                    break; 
                }
            }

            // ========================================================
            // ⭐ 최적화 2단계: 인덱스 전진 탐색 & 조기 종료 (O(N) 달성)
            // ========================================================
            for (size_t i = current_idx + 1; i < points.size(); ++i) { // 0부터가 아니라 current_idx 다음부터!
                if (points[i].visited) continue;

                // X축으로 이미 탐색 반경(radius)을 넘어섰다면?
                // 리스트가 X축 정렬되어 있으므로 이후 점들은 볼 필요도 없음 -> 즉시 break!
                if (points[i].x - current_point.x > current_search_radius) {
                    break; 
                }

                float dist = hypot(points[i].x - current_point.x, points[i].y - current_point.y);
                if (dist < min_dist && dist < current_search_radius) { 
                    min_dist = dist;
                    next_idx = i;
                }
            }

            if (next_idx == -1) break;

            final_path.push_back(points[next_idx]);
            points[next_idx].visited = true;
            current_idx = next_idx; // ⭐ 다음 탐색은 여기서부터 시작
            current_point = points[next_idx];
        }

        // 유저 아이디어: 70개 스킵 + 차선 변경(점프) 시 앞뒤 30개 추가 스킵! (고정값 유지)
        vector<Point2D> smoothed_path;
        smoothed_path.push_back({0.0f, 0.0f, true}); // 내 차 앞범퍼 

        int start_skip = 70;        
        int jump_smooth_range = 30; 

        for (int i = 0; i < (int)final_path.size(); ++i) {
            if (i < start_skip) continue;

            bool is_near_jump = false;
            int check_start = std::max(0, i - jump_smooth_range);
            int check_end = std::min((int)final_path.size() - 2, i + jump_smooth_range);
            
            for (int k = check_start; k <= check_end; ++k) {
                float dist = hypot(final_path[k].x - final_path[k+1].x, final_path[k].y - final_path[k+1].y);
                if (dist > 0.2f) { 
                    is_near_jump = true;
                    break;
                }
            }

            if (!is_near_jump) {
                smoothed_path.push_back(final_path[i]);
            }
        }

        if (smoothed_path.size() == 1 && !final_path.empty()) {
            smoothed_path.push_back(final_path.back());
        }

        final_path = smoothed_path;

        // ========================================================
        // 동적 최대 속도 생성 로직 (V_p & V_q 융합)
        // ========================================================
        float v_max = 1.5f; 
        float v_min = 0.5f; 

        float V_p = v_max;
        float d_max = 1.5f; 
        float d_min = 0.5f; 
        
        if (min_dist_to_P <= d_min) {
            V_p = v_min;
        } else if (min_dist_to_P < d_max) {
            V_p = v_min + (v_max - v_min) * ((min_dist_to_P - d_min) / (d_max - d_min));
        }

        float V_q = v_max;
        if (final_path.size() > 1) {
            Point2D Q = final_path[1]; 
            float angle_Q = std::abs(std::atan2(Q.y, Q.x)); 
            
            float a_min = 5.0f * M_PI / 180.0f;  
            float a_max = 25.0f * M_PI / 180.0f; 
            
            if (angle_Q >= a_max) {
                V_q = v_min;
            } else if (angle_Q > a_min) {
                V_q = v_max - (v_max - v_min) * ((angle_Q - a_min) / (a_max - a_min));
            }
        }

        target_speed_ = std::min(V_p, V_q);

        // 시각화 퍼블리시
        publish_path_visualization(final_path, v_min, v_max);
        publish_speed_text(target_speed_);

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = msg->header.stamp;
        path_msg.header.frame_id = "base_link";

        for (const auto& pt : final_path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = pt.x;
            pose.pose.position.y = pt.y;
            pose.pose.position.z = target_speed_; 
            pose.pose.orientation.w = 1.0; 
            path_msg.poses.push_back(pose);
        }

        if (!path_msg.poses.empty()) path_pub_->publish(path_msg);
    }

    void publish_speed_text(float speed) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link"; 
        marker.header.stamp = this->now();
        marker.ns = "speed_text";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.z = 0.12; 

        marker.color.r = 1.0f; marker.color.g = 1.0f; marker.color.b = 1.0f; marker.color.a = 1.0f; 

        marker.pose.position.x = -0.6; 
        marker.pose.position.y = 0.0; 
        marker.pose.position.z = 0.1; 
        marker.pose.orientation.w = 1.0;

        stringstream ss;
        ss << "MaxSpeed:" << fixed << setprecision(2) << speed << "m/s";
        marker.text = ss.str();

        marker.lifetime = rclcpp::Duration::from_seconds(0.2); 

        speed_text_pub_->publish(marker);
    }

    void publish_path_visualization(const vector<Point2D>& path, float v_min, float v_max) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link"; 
        marker.header.stamp = this->now();
        marker.ns = "path_vis";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP; 
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.05; 
        marker.color.g = 1.0f; marker.color.a = 0.6f; 
        marker.pose.orientation.w = 1.0;

        for (const auto& pt : path) {
            geometry_msgs::msg::Point p;
            p.x = pt.x; p.y = pt.y; p.z = 0.0;
            marker.points.push_back(p);

            float v_range = v_max - v_min;
            if (v_range == 0) v_range = 1.0f;
            float ratio = std::max(0.0f, std::min(1.0f, (target_speed_ - v_min) / v_range));
            
            marker.color.r = 1.0f - ratio;
            marker.color.g = ratio;
        }

        marker.lifetime = rclcpp::Duration::from_seconds(0.2); 
        path_vis_pub_->publish(marker);
    }

    void publish_car_marker() {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link"; 
        marker.header.stamp = this->now();
        marker.ns = "ego_car";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE; 
        marker.action = visualization_msgs::msg::Marker::ADD;

        float car_length = 0.4; 
        float car_width = 0.2;  
        float car_height = 0.25; 

        marker.scale.x = car_length;
        marker.scale.y = car_width;
        marker.scale.z = car_height;

        marker.color.r = 0.1f;
        marker.color.g = 0.5f;
        marker.color.b = 1.0f;
        marker.color.a = 0.7f; 

        marker.pose.position.x = -car_length / 2.0; 
        marker.pose.position.y = 0.0;
        marker.pose.position.z = car_height / 2.0; 

        marker.pose.orientation.w = 1.0;
        car_marker_pub_->publish(marker);
    }

    rclcpp::Subscription<perception::msg::Lanes>::SharedPtr lanes_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obs_sub_; 
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr car_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_vis_pub_; 
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr speed_text_pub_; 
    
    std::vector<TrackedObstacle> obstacles_; 
    
    float target_speed_;
    float max_search_radius_;
    float jump_search_radius_;
    float bubble_a_radius_;
    float bubble_b_radius_;
    float lidar_to_base_offset_x_; 
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<RacePlannerNode>());
    rclcpp::shutdown();
    return 0;
}