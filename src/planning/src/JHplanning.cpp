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
#include <algorithm> 

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
        
        // =====================================================================
        // 🛠️ [튜닝 파라미터 컨트롤 패널] 여기서 주행 성향을 모두 조절하세요!
        // =====================================================================
        
        // 1. 절대 속도 제한 (m/s)
        v_max_ = 0.4f; // 직선이나 뻥 뚫린 길에서 달릴 '최고 속도'
        v_min_ = 0.2f; // 급코너나 장애물 앞에서 기어갈 '최저 속도'

        // 2. 장애물 감속 거리 (m)
        d_max_ = 1.5f; // 이 거리 안으로 장애물이 들어오면 슬슬 브레이크 밟기 시작
        d_min_ = 0.5f; // 이 거리보다 가까우면 무조건 최저 속도(v_min)로 감속

        // 3. 코너링 감속 각도 (도, Degree) -> 직관적으로 숫자만 넣으세요!
        a_min_deg_ = 5.0f;  // 전방 경로가 이 각도(5도) 이하면 직선으로 간주 -> v_max
        a_max_deg_ = 25.0f; // 전방 경로가 이 각도(25도) 이상 꺾이면 급코너로 간주 -> v_min

        // 4. 경로 부드럽게 만들기 (스킵 파라미터)
        start_skip_ = 20;        // 내 차 바로 앞쪽 경로 버리기 (개수) — 70→20: 반응속도 개선
        jump_smooth_range_ = 30; // 차선 변경(점프) 시 앞뒤로 스킵할 범위 (개수)
        
        // 5. 경로 탐색 및 점프 반경 (m)
        max_search_radius_ = 0.1;     // 평상시 다음 점을 이을 최대 간격 (m)
        jump_search_radius_ = 1.5;    // 장애물 발견 시 허용할 '차선 점프' 최대 거리
        
        // 6. 장애물 회피 버블 크기 (m)
        bubble_a_radius_ = 0.1;    // 차선 점들을 지워버릴 1차 범위 (m)
        bubble_b_radius_ = 0.3;    // 회피를 결심할 2차 위험 범위 (m)

        // ⭐ 7. [NEW] 경로 직선 연장(Extrapolation) 안정화 기준점 설정
        extrapolate_base_idx_ = 5;  // 방향을 결정할 앞쪽 끝점 (뒤에서 5번째)
        extrapolate_ref_idx_ = 15;  // 방향을 결정할 뒤쪽 기준점 (뒤에서 15번째)
        
        // =====================================================================

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
        
        lidar_to_base_offset_x_ = 0.0;  

        RCLCPP_INFO(this->get_logger(), "🏁 [Planner] 튜닝 패널 적용 완료! 안정화된 꼬리 자르기 경로 연장이 적용됩니다.");
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

        // 최적화 1단계: 점들을 X축 기준으로 정렬
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
        
        int current_idx = start_idx; 
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
                    
                    for (size_t p_idx = current_idx + 1; p_idx < points.size(); ++p_idx) {
                        if (points[p_idx].x - current_point.x > bubble_b_radius_ + 0.3f) break; 
                        
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

            for (size_t i = current_idx + 1; i < points.size(); ++i) { 
                if (points[i].visited) continue;

                if (points[i].x - current_point.x > current_search_radius) break; 

                float dist = hypot(points[i].x - current_point.x, points[i].y - current_point.y);
                if (dist < min_dist && dist < current_search_radius) { 
                    min_dist = dist;
                    next_idx = i;
                }
            }

            if (next_idx == -1) break;

            final_path.push_back(points[next_idx]);
            points[next_idx].visited = true;
            current_idx = next_idx; 
            current_point = points[next_idx];
        }

        // 경로 스킵 로직
        vector<Point2D> smoothed_path;
        smoothed_path.push_back({0.0f, 0.0f, true}); 

        for (int i = 0; i < (int)final_path.size(); ++i) {
            if (i < start_skip_) continue;

            bool is_near_jump = false;
            int check_start = std::max(0, i - jump_smooth_range_);
            int check_end = std::min((int)final_path.size() - 2, i + jump_smooth_range_);
            
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
        // ⭐ [NEW] 안정화 업데이트: 꼬리 자르기 & 직선 연장 로직 (장애물 관통 방지 탑재!)
        // ========================================================
        int path_size = final_path.size();
        if (path_size >= 2) {
            int base_idx = std::max(0, path_size - extrapolate_base_idx_);
            int ref_idx = std::max(0, path_size - extrapolate_ref_idx_);
            
            Point2D p_base = final_path[base_idx];
            Point2D p_ref = final_path[ref_idx];

            float dx = p_base.x - p_ref.x;
            float dy = p_base.y - p_ref.y;
            
            float dist_check = std::hypot(dx, dy);
            if (dist_check > 0.01f) { 
                float heading = std::atan2(dy, dx);

                // ✂️ [핵심 수술!] 노이즈가 낀 썩은 꼬리(base_idx 이후의 점들)를 과감하게 잘라버립니다!
                final_path.erase(final_path.begin() + base_idx + 1, final_path.end());
                
                // 이제 연장선이 출발할 진짜 마지막 점은 안정적인 'p_base'가 됩니다.
                Point2D p_last = final_path.back();

                // 1.0m 연장 (0.1m 간격으로 10개의 가짜 점 생성)
                float extend_dist = 1.0f;
                float step_size = 0.1f;
                int steps = static_cast<int>(extend_dist / step_size);

                for (int i = 1; i <= steps; ++i) {
                    Point2D ext_pt;
                    ext_pt.x = p_last.x + (std::cos(heading) * step_size * i);
                    ext_pt.y = p_last.y + (std::sin(heading) * step_size * i);
                    ext_pt.visited = true;

                    // 🛑 [핵심 방어 로직] 이 가짜 점이 장애물을 파고드는가?
                    bool hit_obstacle = false;
                    for (const auto& obs : obstacles_) {
                        // 장애물과의 거리가 bubble_b_radius_ (회피 위험 반경) 이내라면!
                        if (std::hypot(ext_pt.x - obs.x, ext_pt.y - obs.y) < bubble_b_radius_) {
                            hit_obstacle = true;
                            break;
                        }
                    }

                    // 장애물에 닿기 직전이라면, 더 이상의 경로 연장을 즉시 중단합니다!
                    if (hit_obstacle) {
                        break; 
                    }

                    final_path.push_back(ext_pt);
                }
            }
        }
        // ========================================================
        
        // 동적 최대 속도 생성 로직
        float V_p = v_max_;
        if (min_dist_to_P <= d_min_) {
            V_p = v_min_;
        } else if (min_dist_to_P < d_max_) {
            float d_range = d_max_ - d_min_;
            if (d_range > 1e-4f) {
                V_p = v_min_ + (v_max_ - v_min_) * ((min_dist_to_P - d_min_) / d_range);
            } else {
                V_p = v_min_;
            }
        }

        float V_q = v_max_;
        if (final_path.size() > 1) {
            Point2D Q = final_path[1];
            float angle_Q = std::abs(std::atan2(Q.y, Q.x));

            float a_min_rad = a_min_deg_ * M_PI / 180.0f;
            float a_max_rad = a_max_deg_ * M_PI / 180.0f;

            if (angle_Q >= a_max_rad) {
                V_q = v_min_;
            } else if (angle_Q > a_min_rad) {
                float a_range = a_max_rad - a_min_rad;
                if (a_range > 1e-6f) {
                    V_q = v_max_ - (v_max_ - v_min_) * ((angle_Q - a_min_rad) / a_range);
                } else {
                    V_q = v_min_;
                }
            }
        }

        target_speed_ = std::min(V_p, V_q);
        if (!std::isfinite(target_speed_)) target_speed_ = v_min_;

        publish_path_visualization(final_path, v_min_, v_max_);
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

    // ROS 관련 변수들
    rclcpp::Subscription<perception::msg::Lanes>::SharedPtr lanes_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obs_sub_; 
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr car_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_vis_pub_; 
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr speed_text_pub_; 
    
    std::vector<TrackedObstacle> obstacles_; 
    
    // 💡 튜닝 패널용 멤버 변수 선언
    float v_max_, v_min_;
    float d_max_, d_min_;
    float a_min_deg_, a_max_deg_;
    int start_skip_, jump_smooth_range_;

    // ⭐ 새로 추가된 Extrapolation 멤버 변수
    int extrapolate_base_idx_;
    int extrapolate_ref_idx_;

    // 기존 멤버 변수
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