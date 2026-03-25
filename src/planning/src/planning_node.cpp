#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <limits>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "perception/msg/lanes.hpp"

enum class BehaviorState { KEEP_LANE_CRUISE, LANE_CHANGE, EMERGENCY_BRAKE };
enum class LaneDir { LEFT = -1, CENTER = 0, RIGHT = 1 };

struct Obstacle {
    double x; double y; double speed;
};

struct VehicleInfo {
    std::vector<geometry_msgs::msg::Point> left_line;
    std::vector<geometry_msgs::msg::Point> ego_line;  
    std::vector<geometry_msgs::msg::Point> right_line;
   
    std::vector<geometry_msgs::msg::Point> straight_ego_line;
    std::vector<geometry_msgs::msg::Point> straight_left_line;  
    std::vector<geometry_msgs::msg::Point> straight_right_line;

    double lane_obs_dist[3];
    double lane_obs_speed[3];
    double lane_rear_obs_dist[3];
    double lane_rear_obs_speed[3];

    double ego_speed;    
    bool   is_centered;  
    std::vector<Obstacle> obstacles;
};

struct DecisionResult {
    BehaviorState state;        
    LaneDir target_lane_dir;
    double target_speed;
};

struct Tentacle {
    double L_total;
    bool collision;
    double cost;    
    std::vector<geometry_msgs::msg::PoseStamped> poses;
};

class DecisionMaker {
private:
    double current_target_v_ = 0.0;
    int lc_lock_timer_ = 0;
    int cooldown_timer_ = 0;
    LaneDir committed_dir_ = LaneDir::CENTER;

public:
    void complete_lane_change() {
        lc_lock_timer_ = 0;
        cooldown_timer_ = 40;  
        committed_dir_ = LaneDir::CENTER;
    }

    void cancel_lane_change() {
        lc_lock_timer_ = 0;
        cooldown_timer_ = 40;  
        committed_dir_ = LaneDir::CENTER;
    }

   DecisionResult decide(const VehicleInfo& v) {
        DecisionResult result;
       
        if (cooldown_timer_ > 0) cooldown_timer_--;

        if (lc_lock_timer_ > 0) {
            lc_lock_timer_--;
        } else {
            committed_dir_ = LaneDir::CENTER;
        }

        result.target_lane_dir = committed_dir_;
        result.state = (lc_lock_timer_ > 0) ? BehaviorState::LANE_CHANGE : BehaviorState::KEEP_LANE_CRUISE;

        int active_idx = (committed_dir_ == LaneDir::LEFT) ? 0 : ((committed_dir_ == LaneDir::RIGHT) ? 2 : 1);
        double my_front_dist = v.lane_obs_dist[active_idx];
        double my_front_speed = v.lane_obs_speed[active_idx];
       
        double MAX_CRUISE_SPEED = 0.4;
        double raw_target_v = MAX_CRUISE_SPEED;
       
        if (my_front_dist < 2.5) {
            double follow_margin = 0.6;
            if (my_front_dist > follow_margin) {
                raw_target_v = std::clamp(my_front_speed + (my_front_dist - follow_margin) * 0.6, 0.0, MAX_CRUISE_SPEED);
            } else {
                raw_target_v = my_front_speed * 0.8;
            }
        }

        double rel_speed_front = v.ego_speed - my_front_speed;
        double dynamic_trigger_dist = 1.2 + std::max(0.0, rel_speed_front) * 2.0;

        if (v.is_centered && lc_lock_timer_ == 0 && cooldown_timer_ == 0 &&
            my_front_dist < dynamic_trigger_dist && my_front_speed < MAX_CRUISE_SPEED * 0.9) {
           
            LaneDir best_dir = LaneDir::CENTER;
            double my_score = my_front_dist + (my_front_speed * 3.0);
            double best_score = my_score + 0.3;
           
            int check_indices[] = {0, 2};
            LaneDir check_dirs[] = {LaneDir::LEFT, LaneDir::RIGHT};
           
            for (int i = 0; i < 2; ++i) {
                int idx = check_indices[i];
                if (idx == 0 && v.left_line.empty()) continue;
                if (idx == 2 && v.right_line.empty()) continue;
               
                double target_front_x = v.lane_obs_dist[idx];
                double target_rear_x = std::abs(v.lane_rear_obs_dist[idx]);
               
                double rel_speed_target_rear = v.lane_rear_obs_speed[idx] - v.ego_speed;
                double dynamic_rear_margin = 0.3 + std::max(0.0, rel_speed_target_rear) * 1.5;

                if (target_front_x < 0.8 || target_rear_x < dynamic_rear_margin) continue;

                double score = target_front_x + (v.lane_obs_speed[idx] * 4.0);
                if (score > best_score) {
                    best_score = score;
                    best_dir = check_dirs[i];
                }
            }

            if (best_dir != LaneDir::CENTER) {
                result.state = BehaviorState::LANE_CHANGE;
                result.target_lane_dir = best_dir;
                committed_dir_ = best_dir;
                lc_lock_timer_ = 20; 
            }
        }

        double rel_speed = v.ego_speed - my_front_speed;
        double emergency_dist = 0.25;
        if (my_front_dist < emergency_dist || ((rel_speed > 0.3) && ((my_front_dist / rel_speed) < 0.6))) {
            result.state = BehaviorState::EMERGENCY_BRAKE;
            raw_target_v = 0.0;
        }

        if (result.state == BehaviorState::EMERGENCY_BRAKE) {
            current_target_v_ = 0.0;
        } else {
            double alpha = (raw_target_v < current_target_v_) ? 0.5 : 0.15;
            current_target_v_ = (alpha * raw_target_v) + ((1.0 - alpha) * current_target_v_);
        }

        result.target_speed = current_target_v_;
        return result;
    }
};

class PlanningNode : public rclcpp::Node {
private:
    VehicleInfo myCar;
    DecisionMaker brain;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_objects_;
    rclcpp::Subscription<perception::msg::Lanes>::SharedPtr sub_lanes_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_ego_speed_;
   
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr viz_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidates_pub_;
   
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;

    bool is_changing_lane_ = false;
    double locked_total_L_ = 0.0;    
    double locked_offset_ = 0.0;      
    double driven_dist_ = 0.0;        
    LaneDir active_change_dir_ = LaneDir::CENTER;

    int left_miss_count_ = 0;
    int right_miss_count_ = 0;
    bool left_lane_exists_ = false;
    bool right_lane_exists_ = false;

    double smoothed_left_offset_ = 0.45;    
    double smoothed_right_offset_ = -0.45;
    double smoothed_yaw_offset_ = 0.0;

    void reset_sensor_data() {
        for(int i=0; i<3; i++) {
            myCar.lane_obs_dist[i] = 30.0; myCar.lane_obs_speed[i] = 0.0;
            myCar.lane_rear_obs_dist[i] = -30.0; myCar.lane_rear_obs_speed[i] = 0.0;
        }
        myCar.obstacles.clear();
    }

    void rotate_pt(double &x, double &y, double theta) {
        double nx = x * std::cos(theta) - y * std::sin(theta);
        double ny = x * std::sin(theta) + y * std::cos(theta);
        x = nx; y = ny;
    }

    // =========================================================================
    // [핵심: 직선/곡선 판별 필터 (Curvature Deadzone) 및 C1 연속 곡선 피팅]
    // =========================================================================
    double get_y_from_line(const std::vector<geometry_msgs::msg::Point>& line, double target_x) {
        if (line.empty()) return 0.0;
        
        // 1. 역방향 연장 (차량 범퍼 앞)
        if (target_x < line.front().x) {
            if (line.size() >= 5) {
                double dx = line[4].x - line[0].x;
                double dy = line[4].y - line[0].y;
                if (std::abs(dx) > 0.05) {
                    double slope = std::clamp(dy / dx, -0.3, 0.3);
                    return line[0].y + slope * (target_x - line[0].x);
                }
            } else if (line.size() >= 2) {
                double dx = line.back().x - line.front().x;
                double dy = line.back().y - line.front().y;
                if (std::abs(dx) > 1e-4) {
                    double slope = std::clamp(dy / dx, -0.3, 0.3);
                    return line.front().y + slope * (target_x - line.front().x);
                }
            }
            return line.front().y;
        }
        
        // 2. 점과 점 사이 빈 공간 보간
        for (size_t i = 0; i < line.size() - 1; ++i) {
            if ((line[i].x <= target_x && line[i+1].x >= target_x) ||
                (line[i].x >= target_x && line[i+1].x <= target_x)) {
                double dx = line[i+1].x - line[i].x;
                if (std::abs(dx) < 1e-6) return line[i].y;
                return line[i].y + (target_x - line[i].x) / dx * (line[i+1].y - line[i].y);
            }
        }
        
        // 3. 차선 끝(허공) 예측 연장
        int n = line.size();
        if (n >= 3 && target_x > line.back().x) {
            int idx1 = 0;
            int idx2 = n / 2;
            int idx3 = n - 1;
            
            double x1 = line[idx1].x, y1 = line[idx1].y;
            double x2 = line[idx2].x, y2 = line[idx2].y;
            double x3 = line[idx3].x, y3 = line[idx3].y;
            
            if ((x3 - x1) > 0.1) {
                double dx1 = x1 - x3; double dy1 = y1 - y3;
                double dx2 = x2 - x3; double dy2 = y2 - y3;
                
                double det = dx1 * dx2 * (dx1 - dx2);
                if (std::abs(det) > 1e-6) {
                    double a = (dy1 * dx2 - dy2 * dx1) / det;
                    double b = (dx1 * dx1 * dy2 - dx2 * dx2 * dy1) / det;
                    double c = y3;
                    
                    // [직선 필터링] 곡률(a)이 0.03 이하라면 직선으로 강제 변환
                    if (std::abs(a) < 0.03) {
                        a = 0.0; 
                    } else {
                        a = std::clamp(a, -0.6, 0.6); 
                    }
                    
                    b = std::clamp(b, -0.5, 0.5);
                    
                    double target_dx = target_x - x3;
                    return a * (target_dx * target_dx) + b * target_dx + c; 
                }
            }
        }
        
        // 4. 점이 2개뿐이거나 곡선 계산 불가 시 (매끄러운 접선 연장)
        if (n >= 2 && target_x > line.back().x) {
            double x0 = line.back().x;
            double y0 = line.back().y;

            double x_m = line.front().x;
            double y_m = line.front().y;
            for (int i = n - 2; i >= 0; --i) {
                if (x0 - line[i].x >= 0.15) {
                    x_m = line[i].x; y_m = line[i].y; break;
                }
            }
            
            double slope = 0.0;
            double dx_m = x0 - x_m;
            if (dx_m > 1e-4) {
                slope = (y0 - y_m) / dx_m;
                slope = std::clamp(slope, -0.4, 0.4);
            }

            double x_prev = line.front().x;
            double y_prev = line.front().y;
            bool found_prev = false;
            for (int i = n - 2; i >= 0; --i) {
                if (x0 - line[i].x >= 0.5) {
                    x_prev = line[i].x; y_prev = line[i].y; found_prev = true; break;
                }
            }

            double A = 0.0;
            if (found_prev) {
                double dx_prev = x_prev - x0; 
                double dy_prev = y_prev - y0;
                A = (dy_prev - slope * dx_prev) / (dx_prev * dx_prev);
                
                // [직선 필터링] 여기서도 곡률이 미미하면 완벽한 직선(A=0)으로 뻗음!
                if (std::abs(A) < 0.03) {
                    A = 0.0;
                } else {
                    A = std::clamp(A, -0.3, 0.3);
                }
            }

            double dx = target_x - x0;
            return y0 + slope * dx + A * dx * dx;
        }
        
        return line.empty() ? 0.0 : line.back().y;
    }

public:
    PlanningNode() : Node("planning_node") {
        myCar.ego_speed = 0.0;
        myCar.is_centered = true;
        last_time_ = this->get_clock()->now();
        reset_sensor_data();

        sub_objects_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/perception/tracked_objects", 10, std::bind(&PlanningNode::objects_callback, this, std::placeholders::_1));
        sub_lanes_ = this->create_subscription<perception::msg::Lanes>(
            "/perception/lane/lanes", 10, std::bind(&PlanningNode::lanes_callback, this, std::placeholders::_1));
        sub_ego_speed_ = this->create_subscription<std_msgs::msg::Float32>(
            "/ego_speed", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { this->myCar.ego_speed = msg->data; });

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/local_path", 10);
        viz_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/viz_path", 10);
        candidates_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planning/candidate_paths", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&PlanningNode::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), " Perfected Curvature Filtering & Throw-Catch Logic Active!");
    }

private:
    void lanes_callback(const perception::msg::Lanes::SharedPtr msg) {  
        if (msg->lanes.empty()) {
            myCar.is_centered = false;
            left_lane_exists_ = false;
            right_lane_exists_ = false;
            return;
        }

        int best_idx = -1;
        double min_y_abs = 999.0;
        
        // 1. 화면에서 내 차(base_link)와 가장 가까운 기준선 하나를 무조건 선택
        for (size_t i = 0; i < msg->lanes.size(); ++i) {
            if (msg->lanes[i].points.size() < 2) continue; 
            double y_int = std::abs(get_y_from_line(msg->lanes[i].points, 0.5));
            if (y_int < min_y_abs) {
                min_y_abs = y_int;
                best_idx = i;
            }
        }

        if (best_idx != -1) {
            myCar.is_centered = true;
            
            auto& base_line = msg->lanes[best_idx].points;
            
            // Yaw 보정
            double raw_yaw = 0.0;
            if (base_line.size() >= 3) {
                int lookahead_idx = std::min(10, (int)base_line.size() - 1);
                double dx = base_line[lookahead_idx].x - base_line[0].x;
                double dy = base_line[lookahead_idx].y - base_line[0].y;
                raw_yaw = std::atan2(dy, dx);
            }
            smoothed_yaw_offset_ = 0.05 * raw_yaw + 0.95 * smoothed_yaw_offset_;
            
            std::vector<geometry_msgs::msg::Point> straight_base;
            for (auto pt : base_line) {
                rotate_pt(pt.x, pt.y, -smoothed_yaw_offset_);
                straight_base.push_back(pt);
            }

            double base_y_05 = get_y_from_line(straight_base, 0.5);
            double base_y_12 = get_y_from_line(straight_base, 1.2);

            std::vector<geometry_msgs::msg::Point> stitched_base;
            std::vector<geometry_msgs::msg::Point> stitched_left;
            std::vector<geometry_msgs::msg::Point> stitched_right;
            
            // 2. 조각 모음 및 나비 경로 생성을 위한 좌/우 차선 인식
            for (size_t i = 0; i < msg->lanes.size(); ++i) {
                if (msg->lanes[i].points.size() < 2) continue;
                
                std::vector<geometry_msgs::msg::Point> straight_cand;
                for (auto pt : msg->lanes[i].points) {
                    double nx = pt.x, ny = pt.y;
                    rotate_pt(nx, ny, -smoothed_yaw_offset_);
                    pt.x = nx; pt.y = ny;
                    straight_cand.push_back(pt);
                }
                
                double cand_y_05 = get_y_from_line(straight_cand, 0.5);
                double cand_y_12 = get_y_from_line(straight_cand, 1.2);
                
                if (std::abs((cand_y_05 - base_y_05) - (cand_y_12 - base_y_12)) > 0.3) continue;
                
                double lat_dist = cand_y_05 - base_y_05;

                if (std::abs(lat_dist) <= 0.25) {
                    stitched_base.insert(stitched_base.end(), straight_cand.begin(), straight_cand.end());
                } else if (lat_dist > 0.25 && lat_dist < 1.5) {
                    stitched_left.insert(stitched_left.end(), straight_cand.begin(), straight_cand.end());
                } else if (lat_dist < -0.25 && lat_dist > -1.5) {
                    stitched_right.insert(stitched_right.end(), straight_cand.begin(), straight_cand.end());
                }
            }

            auto sortByX = [](const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b){ return a.x < b.x; };
            std::sort(stitched_base.begin(), stitched_base.end(), sortByX);
            std::sort(stitched_left.begin(), stitched_left.end(), sortByX);
            std::sort(stitched_right.begin(), stitched_right.end(), sortByX);

            double final_y_05 = get_y_from_line(stitched_base, 0.5);
            double shift_to_center = 0.0;
            
            // 3. 0.3m 엄격 필터링
            const double MAX_MY_LANE_DIST = 0.30; 
            const double LANE_HALF_WIDTH = 0.45; 

            if (final_y_05 > 0.15 && final_y_05 <= MAX_MY_LANE_DIST) {
                shift_to_center = -LANE_HALF_WIDTH; 
            } else if (final_y_05 < -0.15 && final_y_05 >= -MAX_MY_LANE_DIST) {
                shift_to_center = +LANE_HALF_WIDTH; 
            } else {
                // 내 차선이 아니면 추종하지 않고, 동적 평행 이동은 삭제
                shift_to_center = -final_y_05; 
            }

            // 최종 가상의 중앙선 (ego_line) 연성
            myCar.straight_ego_line.clear();
            myCar.ego_line.clear();
            for (double x = 0.0; x <= 2.5; x += 0.1) {
                double base_y = get_y_from_line(stitched_base, x);
                double center_y = base_y + shift_to_center; 
                
                geometry_msgs::msg::Point pt;
                pt.x = x; pt.y = center_y; pt.z = 0.0;
                myCar.straight_ego_line.push_back(pt);
                
                double raw_x = x, raw_y = center_y;
                rotate_pt(raw_x, raw_y, smoothed_yaw_offset_);
                geometry_msgs::msg::Point rpt;
                rpt.x = raw_x; rpt.y = raw_y; rpt.z = 0.0;
                myCar.ego_line.push_back(rpt);
            }

            // 4. 나비(Tentacle) 생성을 위해 좌/우 차선 인식 상태 업데이트
            if (!stitched_left.empty()) {
                myCar.straight_left_line = stitched_left;
                smoothed_left_offset_ = 0.05 * (get_y_from_line(stitched_left, 0.5) - final_y_05) + 0.95 * smoothed_left_offset_;
                left_miss_count_ = 0;
                left_lane_exists_ = true;
            } else {
                left_miss_count_++;
                if (left_miss_count_ > 3) left_lane_exists_ = false;
            }

            if (!stitched_right.empty()) {
                myCar.straight_right_line = stitched_right;
                smoothed_right_offset_ = 0.05 * (get_y_from_line(stitched_right, 0.5) - final_y_05) + 0.95 * smoothed_right_offset_;
                right_miss_count_ = 0;
                right_lane_exists_ = true;
            } else {
                right_miss_count_++;
                if (right_miss_count_ > 3) right_lane_exists_ = false;
            }

        } else {
            myCar.is_centered = false;
        }
    }

    void objects_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        reset_sensor_data();
        if (msg->poses.empty() || myCar.straight_ego_line.empty()) return;

        for (const auto& pose : msg->poses) {
            double ox = pose.position.x, oy = pose.position.y, os = pose.position.z;
            if (std::abs(ox) < 0.1 && std::abs(oy) < 0.2) continue;
           
            myCar.obstacles.push_back({ox, oy, os});
           
            double rot_ox = ox, rot_oy = oy;
            rotate_pt(rot_ox, rot_oy, -smoothed_yaw_offset_);

            double expected_y = get_y_from_line(myCar.straight_ego_line, rot_ox);
            double lat_diff = rot_oy - expected_y;
           
            int obs_lane = -1;
            if (std::abs(lat_diff) < 0.22) obs_lane = 1;
            else if (lat_diff >= 0.22 && lat_diff < 0.80) obs_lane = 0;
            else if (lat_diff <= -0.22 && lat_diff > -0.80) obs_lane = 2;

            if (obs_lane != -1) {  
                if (rot_ox >= -0.4 && rot_ox < myCar.lane_obs_dist[obs_lane]) {
                    myCar.lane_obs_dist[obs_lane] = rot_ox;
                    myCar.lane_obs_speed[obs_lane] = os;
                }
                if (rot_ox < -0.1 && rot_ox > myCar.lane_rear_obs_dist[obs_lane]) {
                    myCar.lane_rear_obs_dist[obs_lane] = rot_ox;
                    myCar.lane_rear_obs_speed[obs_lane] = os;
                }
            }
        }
    }

    void timer_callback() {
        auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        if (dt > 0.2 || dt <= 0.0) dt = 0.1;
        last_time_ = now;

        DecisionResult decision = brain.decide(myCar);
       
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "base_link";
        path_msg.header.stamp = now;

        double max_visible_x = 2.0; 

        if (myCar.straight_ego_line.empty()) {  
            path_pub_->publish(path_msg);
            viz_path_pub_->publish(path_msg);
            return;
        }

        if (is_changing_lane_) driven_dist_ += (myCar.ego_speed * dt);
        else driven_dist_ = 0.0;  

        std::vector<Obstacle> straight_obstacles;
        for (auto obs : myCar.obstacles) {
            double ox = obs.x, oy = obs.y;
            rotate_pt(ox, oy, -smoothed_yaw_offset_);
            straight_obstacles.push_back({ox, oy, obs.speed});
        }

        std::vector<double> candidate_lengths = {0.7};
        std::vector<Tentacle> left_tentacles, right_tentacles;  

        auto generate_tentacles = [&](const std::vector<geometry_msgs::msg::Point>& target_line, double default_offset, std::vector<Tentacle>& out_tentacles) {  
            for (double L : candidate_lengths) {
                Tentacle t;
                t.L_total = L;
                t.collision = false;  
               
                for (double local_x = 0.0; local_x <= max_visible_x; local_x += 0.01) {
                    double world_x = driven_dist_ + local_x;
                    double progress = std::clamp(world_x / L, 0.0, 1.0);
                    double dodge_blend = 0.5 * (1.0 - std::cos(progress * M_PI));
                   
                    double sy = get_y_from_line(myCar.straight_ego_line, local_x);
                    double ty = sy + default_offset;
                    if (!target_line.empty()) {
                        ty = get_y_from_line(target_line, local_x);
                    }
                   
                    double path_y = sy + (ty - sy) * dodge_blend;
                   
                    double ego_v = std::max(myCar.ego_speed, 0.1);
                    double time_to_reach = local_x / ego_v;

                    for (const auto& obs : straight_obstacles) {
                        double rel_v = obs.speed - myCar.ego_speed;
                        double future_obs_x = obs.x + (rel_v * time_to_reach);

                        if (future_obs_x > -0.5 && future_obs_x < max_visible_x) {
                            if (std::hypot(future_obs_x - local_x, obs.y - path_y) < 0.20) {
                                t.collision = true; break;
                            }
                        }
                    }

                    double real_x = local_x;
                    double real_y = path_y;
                    rotate_pt(real_x, real_y, smoothed_yaw_offset_);

                    geometry_msgs::msg::PoseStamped p;
                    p.pose.position.x = real_x;
                    p.pose.position.y = real_y;
                    t.poses.push_back(p);  
                }
                t.cost = std::abs(0.7 - L);
                out_tentacles.push_back(t);  
            }
        };

        bool is_going_left = (is_changing_lane_ && active_change_dir_ == LaneDir::LEFT);
        bool is_going_right = (is_changing_lane_ && active_change_dir_ == LaneDir::RIGHT);
        
        bool keep_left = left_lane_exists_ || is_going_left;
        bool keep_right = right_lane_exists_ || is_going_right;

        if (!is_changing_lane_) {
            if (keep_left) generate_tentacles(myCar.straight_left_line, smoothed_left_offset_, left_tentacles);
            if (keep_right) generate_tentacles(myCar.straight_right_line, smoothed_right_offset_, right_tentacles);  
           
            if (decision.target_lane_dir != LaneDir::CENTER) {
                std::vector<Tentacle>* t_list = (decision.target_lane_dir == LaneDir::LEFT) ? &left_tentacles : &right_tentacles;
                bool safe = false;
                for (const auto& t : *t_list) {
                    if (!t.collision) { safe = true; break; }
                }

                if (safe) {
                    is_changing_lane_ = true;
                    active_change_dir_ = decision.target_lane_dir;
                    locked_total_L_ = 0.7; 
                    locked_offset_ = (active_change_dir_ == LaneDir::LEFT) ? smoothed_left_offset_ : smoothed_right_offset_;
                    driven_dist_ = 0.0;
                } else {
                    brain.cancel_lane_change();
                }  
            }
        }

        std::vector<geometry_msgs::msg::PoseStamped> final_path_poses;  

        if (is_changing_lane_) {
            if (driven_dist_ < 0.2) {
                // [동민님의 핵심 로직: 0.2m Blind Throw]
                double navi_slope = (active_change_dir_ == LaneDir::LEFT) ? 0.6 : -0.6; 
                for (double local_x = 0.0; local_x <= max_visible_x; local_x += 0.01) {
                    double sy = get_y_from_line(myCar.straight_ego_line, local_x);
                    double path_y = (local_x <= 0.2) ? sy + (navi_slope * local_x) : sy + (navi_slope * 0.2); 
                    
                    double real_x = local_x; double real_y = path_y;
                    rotate_pt(real_x, real_y, smoothed_yaw_offset_);
                    
                    geometry_msgs::msg::PoseStamped p;
                    p.pose.position.x = real_x; p.pose.position.y = real_y;
                    final_path_poses.push_back(p);
                }
            } else {
                // [동민님의 핵심 로직: Catch Phase - 평상시 ego_line으로 자연스럽게 복귀]
                for (double x = 0.0; x <= max_visible_x; x += 0.01) {
                    geometry_msgs::msg::PoseStamped p;
                    p.pose.position.x = x; p.pose.position.y = get_y_from_line(myCar.ego_line, x);
                    final_path_poses.push_back(p);  
                }
            }

            if (driven_dist_ >= 0.3) {
                is_changing_lane_ = false;          
                active_change_dir_ = LaneDir::CENTER;
                brain.complete_lane_change();       
            }
        } else {
            // 평상시: 언제나 가장 튼튼하고 완벽한 ego_line을 추종
            for (double x = 0.0; x <= max_visible_x; x += 0.01) {
                geometry_msgs::msg::PoseStamped p;
                p.pose.position.x = x; p.pose.position.y = get_y_from_line(myCar.ego_line, x);
                final_path_poses.push_back(p);  
            }
        }

        visualization_msgs::msg::MarkerArray candidate_markers;  
       
        if (!is_changing_lane_) {
            visualization_msgs::msg::Marker delete_frozen;
            delete_frozen.header.frame_id = "base_link"; delete_frozen.header.stamp = now;
            delete_frozen.ns = "frozen_path"; delete_frozen.id = 0;
            delete_frozen.action = visualization_msgs::msg::Marker::DELETE;
            candidate_markers.markers.push_back(delete_frozen);

            // 나비(텐타클) 그리기
            auto draw_unlocked_markers = [&](const std::vector<Tentacle>& t_list, const std::string& ns) {
                for (size_t i = 0; i < candidate_lengths.size(); ++i) {
                    visualization_msgs::msg::Marker m;
                    m.header.frame_id = "base_link"; m.header.stamp = now;
                    m.ns = ns; m.id = i;
                    if (t_list.empty()) {
                        m.action = visualization_msgs::msg::Marker::DELETE;
                        candidate_markers.markers.push_back(m);
                        continue;
                    }
                    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
                    m.action = visualization_msgs::msg::Marker::ADD;
                    m.pose.orientation.w = 1.0;
                    const Tentacle& t = t_list[i];
                    for (const auto& p : t.poses) {
                        geometry_msgs::msg::Point pt;
                        pt.x = p.pose.position.x; pt.y = p.pose.position.y; pt.z = 0.0;
                        m.points.push_back(pt);
                    }
                    m.scale.x = 0.04;
                    if (t.collision) {
                        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.6;
                    } else {
                        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.6;
                    }
                    candidate_markers.markers.push_back(m);
                }
            };
            draw_unlocked_markers(left_tentacles, "tentacles_left");
            draw_unlocked_markers(right_tentacles, "tentacles_right");
        } else {
            // 강제 조향 시에는 노란색 Throw 궤적 그리기
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "base_link"; m.header.stamp = now;
            m.ns = "frozen_path"; m.id = 0;
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;
            m.scale.x = 0.12;
           
            for (const auto& p : final_path_poses) {
                geometry_msgs::msg::Point pt;
                pt.x = p.pose.position.x; pt.y = p.pose.position.y; pt.z = 0.0;
                m.points.push_back(pt);
            }
            candidate_markers.markers.push_back(m);
        }
        candidates_pub_->publish(candidate_markers);  
       
        double final_dodge_speed = decision.target_speed;

        for (size_t i = 0; i < final_path_poses.size(); ++i) {
            final_path_poses[i].header = path_msg.header;
            if (decision.state == BehaviorState::EMERGENCY_BRAKE) {
                final_path_poses[i].pose.position.z = 0.0;
            } else {
                final_path_poses[i].pose.position.z = final_dodge_speed;
            }
           
            if (i < final_path_poses.size() - 1) {
                double dx = final_path_poses[i+1].pose.position.x - final_path_poses[i].pose.position.x;
                double dy = final_path_poses[i+1].pose.position.y - final_path_poses[i].pose.position.y;
                double yaw = std::atan2(dy, dx);
                final_path_poses[i].pose.orientation.z = std::sin(yaw / 2.0);
                final_path_poses[i].pose.orientation.w = std::cos(yaw / 2.0);
            } else if (i > 0) {
                final_path_poses[i].pose.orientation = final_path_poses[i-1].pose.orientation;
            }
            path_msg.poses.push_back(final_path_poses[i]);
        }

        path_pub_->publish(path_msg);
       
        nav_msgs::msg::Path viz_path = path_msg;
        for (auto& p : viz_path.poses) p.pose.position.z = 0.0;
        viz_path_pub_->publish(viz_path);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlanningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
