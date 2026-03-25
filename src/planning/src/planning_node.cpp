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
};

struct TrackedObstacle {
    int id;       
    float x;      
    float y;      
    float speed;  
    float yaw;    
};

// 🚗 차량의 현재 주행 상태
enum DrivingState {
    NORMAL,         // 평상시 카메라 차선 추종
    AVOIDING,       // 인식된 차선 방향으로 강제 회피 기동
    FOLLOWING,      // 모든 차선이 막혔을 때 앞차 추종 (대기)
    EMERGENCY_STOP  // 1m 이내 장애물 감지 시 급정지
};

class RacePlannerNode : public rclcpp::Node {
public:
    RacePlannerNode() : Node("race_planner_node") {
        
        // ==========================================
        // 🛠️ [튜닝 파라미터 - 차선 간격 0.45m 스케일 맞춤]
        // ==========================================
        v_max_ = 0.4f; 
        v_min_ = 0.2f; 

        // 🟢 상태 1: 일반 경로 탐색 & 차선 분류 파라미터
        max_search_radius_ = 0.4f; // 점과 점을 이을 최대 반경 (차선 간격보다 살짝 작게 설정)
        
        // ⭐ 차선 분류 기준: 내 차선(0m)과 옆 차선(0.45m)을 가르는 임계값
        my_lane_y_threshold_ = 0.25f; 
        
        lateral_search_limit_ = 0.2f; // 내 차선 시작점을 찾을 때 좌우 0.2m 내에서만 찾음
        bubble_a_radius_ = 0.2f; // 버블 크기 (옆 차선까지 안 지워지도록 0.2m로 축소)
        blockage_check_dist_ = 1.0f; 

        // 🔴 상태 2: 회피 기동 파라미터
        avoidance_time_sec_ = 1.0; 
        avoidance_angle_deg_ = 25.0f; // 0.45m 이동에 맞게 회피 각도 축소 (45 -> 25도)
        
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
        speed_text_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/speed_text", 10); 

        RCLCPP_INFO(this->get_logger(), "🏁 [Planner] 0.45m 차선 폭 스케일 파라미터 적용 완료!");
    }

private:
    void obs_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        obstacles_.clear();
        for (const auto& pose : msg->poses) {
            TrackedObstacle obs;
            obs.x = pose.position.x + lidar_to_base_offset_x_;
            obs.y = pose.position.y;
            obstacles_.push_back(obs);
        }
    }

    void lanes_callback(const perception::msg::Lanes::SharedPtr msg) {
        publish_car_marker();

        // ---------------------------------------------------------------------
        // 🔴 STATE: AVOIDING (강제 회피 모드 중일 때)
        // ---------------------------------------------------------------------
        if (current_state_ == AVOIDING) {
            double elapsed_time = (this->now() - avoidance_start_time_).seconds();
            if (elapsed_time > avoidance_time_sec_) {
                current_state_ = NORMAL;
                RCLCPP_INFO(this->get_logger(), "🟢 회피 종료! 카메라 차선 추종으로 복귀합니다.");
            } else {
                publish_avoidance_path(msg->header.stamp);
                publish_speed_text(v_min_);
                return; 
            }
        } else {
            current_state_ = NORMAL;
        }

        // ---------------------------------------------------------------------
        // 🟢 STATE: NORMAL (명시적 차선 분류 및 필터링)
        // ---------------------------------------------------------------------
        vector<Point2D> my_lane_points;
        vector<Point2D> left_lane_points;
        vector<Point2D> right_lane_points;

        // 1. 차선 덩어리(Array) 단위로 이름표 붙이기 및 버블 필터링
        for (const auto& lane : msg->lanes) {
            if (lane.points.empty()) continue;

            // 뿌리 구역(X: 0~1.5m)의 평균 Y 좌표 계산
            float sum_y = 0.0f;
            int root_pt_count = 0;
            for (const auto& pt : lane.points) {
                if (pt.x > 0.0f && pt.x < 1.5f) {
                    sum_y += pt.y;
                    root_pt_count++;
                }
            }

            float avg_y = (root_pt_count > 0) ? (sum_y / root_pt_count) : lane.points.front().y;

            // 점들을 버블로 필터링한 뒤, 이름표에 맞는 바구니에 담기
            for (const auto& pt : lane.points) {
                if (pt.x > 0.0f) {
                    bool in_bubble = false;
                    for (const auto& obs : obstacles_) { 
                        if (hypot(pt.x - obs.x, pt.y - obs.y) < bubble_a_radius_) {
                            in_bubble = true;
                            break;
                        }
                    }
                    if (!in_bubble) {
                        Point2D p = {(float)pt.x, (float)pt.y, false};
                        
                        // ⭐ 차선 폭 0.45m에 맞춘 칼같은 분류 (0.25m 기준)
                        if (std::abs(avg_y) <= my_lane_y_threshold_) {
                            my_lane_points.push_back(p);      // 내 차선
                        } else if (avg_y > my_lane_y_threshold_) {
                            left_lane_points.push_back(p);    // 좌측 차선
                        } else {
                            right_lane_points.push_back(p);   // 우측 차선
                        }
                    }
                }
            }
        }

        // 2. 경로 단절 감지 (내 차선만 검사)
        bool path_blocked = true;
        for (const auto& pt : my_lane_points) {
            if (pt.x > 0.0f && pt.x < blockage_check_dist_) {
                path_blocked = false;
                break;
            }
        }

        // 3. 라이다 공간 로직 (내 차선 폭 안에 장애물이 있는지 검사)
        float min_obs_x = numeric_limits<float>::max();
        bool obstacle_in_my_lane = false;
        // ⭐ 내 차선 폭 인식: 좌우 0.25m (총 0.5m 폭) 이내면 내 앞을 막았다고 판단
        float my_lane_obs_width = 0.25f; 

        for (const auto& obs : obstacles_) {
            if (obs.x > 0.0f && std::abs(obs.y) < my_lane_obs_width) {
                obstacle_in_my_lane = true;
                if (obs.x < min_obs_x) {
                    min_obs_x = obs.x; 
                }
            }
        }

        // ---------------------------------------------------------------------
        // 🔴 상태 분기 (막힘 판단 시)
        // ---------------------------------------------------------------------
        if (path_blocked && obstacle_in_my_lane) {
            
            // [긴급 정지]
            if (min_obs_x <= 1.0f) {
                current_state_ = EMERGENCY_STOP;
                RCLCPP_WARN(this->get_logger(), "🚨 1m 이내 장애물 도달! 급정지합니다.");
                publish_straight_path(msg->header.stamp, 0.0f); 
                publish_speed_text(0.0f);
                return;
            }

            // [차선 변경 / 회피]
            bool can_go_left = false;
            for (const auto& pt : left_lane_points) if (pt.x < 2.0f) { can_go_left = true; break; }
            
            bool can_go_right = false;
            for (const auto& pt : right_lane_points) if (pt.x < 2.0f) { can_go_right = true; break; }

            if (can_go_left) {
                current_state_ = AVOIDING;
                avoidance_direction_ = 1; // 좌측(+25도)
                avoidance_start_time_ = this->now();
                RCLCPP_WARN(this->get_logger(), "🔴 좌측 차선 확인! 좌측으로 회피합니다.");
                publish_avoidance_path(msg->header.stamp);
                return;
            } else if (can_go_right) {
                current_state_ = AVOIDING;
                avoidance_direction_ = -1; // 우측(-25도)
                avoidance_start_time_ = this->now();
                RCLCPP_WARN(this->get_logger(), "🔴 우측 차선 확인! 우측으로 회피합니다.");
                publish_avoidance_path(msg->header.stamp);
                return;
            } else {
                // [앞차 추종 대기]
                current_state_ = FOLLOWING;
                float target_speed = (min_obs_x > 2.0f) ? v_max_ : v_min_;
                RCLCPP_INFO(this->get_logger(), "🚙 옆 차선 없음. 앞차 추종 중 (거리: %.2fm)", min_obs_x);
                publish_straight_path(msg->header.stamp, target_speed);
                publish_speed_text(target_speed);
                return;
            }
        }

        // ---------------------------------------------------------------------
        // 🟢 정상 경로 생성 (오직 '내 차선' 점들만 사용!)
        // ---------------------------------------------------------------------
        if (my_lane_points.empty()) return;

        // X축 오름차순 정렬
        std::sort(my_lane_points.begin(), my_lane_points.end(), [](const Point2D& a, const Point2D& b) {
            return a.x < b.x;
        });

        // 시작점 찾기 
        int start_idx = -1;
        float min_dist_to_origin = numeric_limits<float>::max();

        for (size_t i = 0; i < my_lane_points.size(); ++i) {
            float dist = hypot(my_lane_points[i].x, my_lane_points[i].y);
            if (dist < min_dist_to_origin) {
                min_dist_to_origin = dist;
                start_idx = i;
            }
        }

        if (start_idx == -1) return;

        // Greedy 경로 연결
        vector<Point2D> final_path;
        my_lane_points[start_idx].visited = true;
        final_path.push_back(my_lane_points[start_idx]);
        
        Point2D current_point = my_lane_points[start_idx];
        int current_search_start_idx = start_idx;

        while (true) {
            int next_idx = -1;
            float min_dist = numeric_limits<float>::max();

            for (size_t i = current_search_start_idx; i < my_lane_points.size(); ++i) {
                if (my_lane_points[i].visited) continue;

                if (my_lane_points[i].x - current_point.x > max_search_radius_) break; 

                float dist = hypot(my_lane_points[i].x - current_point.x, my_lane_points[i].y - current_point.y);
                if (dist < max_search_radius_ && dist < min_dist) {
                    min_dist = dist;
                    next_idx = i;
                }
            }

            if (next_idx == -1) break;

            my_lane_points[next_idx].visited = true;
            final_path.push_back(my_lane_points[next_idx]);
            
            current_point = my_lane_points[next_idx];
            current_search_start_idx = next_idx + 1;
        }

        // 발행
        publish_path_visualization(final_path, v_min_, v_max_);
        publish_speed_text(v_max_);
        publish_path_msg(final_path, msg->header.stamp, v_max_);
    }

    void publish_avoidance_path(const builtin_interfaces::msg::Time& stamp) {
        vector<Point2D> avoid_path;
        float angle_rad = avoidance_angle_deg_ * M_PI / 180.0f * avoidance_direction_;
        for (float dist = 0.0f; dist <= 2.0f; dist += 0.1f) {
            avoid_path.push_back({dist * std::cos(angle_rad), dist * std::sin(angle_rad), true});
        }
        publish_path_visualization(avoid_path, v_min_, v_max_);
        publish_path_msg(avoid_path, stamp, v_min_); 
    }

    void publish_straight_path(const builtin_interfaces::msg::Time& stamp, float speed) {
        vector<Point2D> straight_path;
        for (float dist = 0.0f; dist <= 2.0f; dist += 0.2f) {
            straight_path.push_back({dist, 0.0f, true}); 
        }
        publish_path_visualization(straight_path, v_min_, v_max_);
        publish_path_msg(straight_path, stamp, speed);
    }

    void publish_path_msg(const vector<Point2D>& path_points, const builtin_interfaces::msg::Time& stamp, float speed) {
        if (path_points.empty()) return;
        
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = stamp;
        path_msg.header.frame_id = "base_link";

        for (const auto& pt : path_points) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = pt.x;
            pose.pose.position.y = pt.y;
            pose.pose.position.z = speed; 
            pose.pose.orientation.w = 1.0; 
            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
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
        marker.pose.position.x = -0.6; marker.pose.position.y = 0.0; marker.pose.position.z = 0.1; 
        marker.pose.orientation.w = 1.0;
        
        string state_str = "NORMAL";
        if (current_state_ == AVOIDING) state_str = "AVOIDING";
        else if (current_state_ == FOLLOWING) state_str = "FOLLOWING";
        else if (current_state_ == EMERGENCY_STOP) state_str = "EMERG_STOP";

        stringstream ss; ss << "Mode: " << state_str << "\nSpd: " << fixed << setprecision(2) << speed << "m/s";
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
        marker.pose.orientation.w = 1.0;

        for (const auto& pt : path) {
            geometry_msgs::msg::Point p;
            p.x = pt.x; p.y = pt.y; p.z = 0.0;
            marker.points.push_back(p);
            
            if (current_state_ == AVOIDING) {
                marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 0.0f; marker.color.a = 0.8f; 
            } else if (current_state_ == EMERGENCY_STOP) {
                marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 1.0f; marker.color.a = 0.8f; 
            } else if (current_state_ == FOLLOWING) {
                marker.color.r = 1.0f; marker.color.g = 0.5f; marker.color.b = 0.0f; marker.color.a = 0.8f; 
            } else {
                marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; marker.color.a = 0.6f; 
            }
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
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr speed_text_pub_; 
    
    std::vector<TrackedObstacle> obstacles_; 
    
    DrivingState current_state_;
    rclcpp::Time avoidance_start_time_;
    int avoidance_direction_;

    float v_max_, v_min_;
    float max_search_radius_;
    float my_lane_y_threshold_;
    float bubble_a_radius_, lateral_search_limit_, blockage_check_dist_;
    double avoidance_time_sec_;
    float avoidance_angle_deg_;
    float lidar_to_base_offset_x_; 
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<RacePlannerNode>());
    rclcpp::shutdown();
    return 0;
}
