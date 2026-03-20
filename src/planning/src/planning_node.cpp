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

    double lane_obs_dist[3];
    double lane_obs_speed[3];
    double lane_rear_obs_dist[3];
    double lane_rear_obs_speed[3];

    double ego_speed;    
    bool   is_centered;  
    std::vector<Obstacle> obstacles;

    double curve_kappa = 0.0;    
    double vision_length = 2.0;  
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

struct FrenetObstacle {
    double s; double d; double speed;
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
        cooldown_timer_ = 5;
        committed_dir_ = LaneDir::CENTER;
    }

    void cancel_lane_change() {
        lc_lock_timer_ = 0;
        cooldown_timer_ = 5;
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
       
        // 🌟 레이싱 튜닝: 최고 속도를 4.0m/s로 상향하여 직선 속도감 극대화
        double MAX_CRUISE_SPEED = 4.0;
        double raw_target_v = MAX_CRUISE_SPEED;
       
        // =========================================================
        // 🌟 1. 랩타임 단축을 위한 다이나믹 코너링 감속 로직
        // =========================================================
        double curve_speed_limit = MAX_CRUISE_SPEED;
       
        // 곡률이 감지되거나 시야가 2.5m 이하로 좁아지기 시작하면 코너 진입 준비
        if (v.curve_kappa > 0.1 || v.vision_length < 2.5) {
            curve_speed_limit = 2.8; // 일반 커브는 관성을 살려 2.8m/s로 빠르게 돌아나감
           
            // 곡률이 0.8을 넘어가는 숏코너 (1차선 헤어핀 등)
            if (v.curve_kappa > 0.8) {
                // 한계 곡률(1.5)까지 2.8m/s에서 1.2m/s로 비례 감속 (언더스티어 마지노선)
                double mapped_v = 2.8 - ((v.curve_kappa - 0.8) / (1.5 - 0.8)) * (2.8 - 1.2);
                curve_speed_limit = std::min(curve_speed_limit, std::clamp(mapped_v, 1.2, 2.8));
            }
            // 시야가 1.5m 이하로 짧아지는 맹점 코너
            else if (v.vision_length < 1.5) {
                double mapped_vision = 1.2 + ((v.vision_length - 0.5) / (1.5 - 0.5)) * (2.8 - 1.2);
                curve_speed_limit = std::min(curve_speed_limit, std::clamp(mapped_vision, 1.2, 2.8));
            }
        }
       
        raw_target_v = std::min(raw_target_v, curve_speed_limit);

        // =========================================================
        // 🌟 2. 장애물 방어 (ACC) 로직 (빠른 속도에 맞춰 안전거리 상향)
        // =========================================================
        bool left_blocked = v.left_line.empty() || (v.lane_obs_dist[0] < 15.0);
        bool right_blocked = v.right_line.empty() || (v.lane_obs_dist[2] < 15.0);
        bool all_blocked = (my_front_dist < 15.0) && left_blocked && right_blocked;

        if (all_blocked) {
            raw_target_v = std::min(raw_target_v, 0.8); // 다 막혔을 때도 완전히 기어가진 않고 0.8 유지
        } else if (my_front_dist < 15.0) {
            double follow_margin = 3.0; // 속도가 빨라졌으므로 여유 간격을 3.0m로 상향
            if (my_front_dist > follow_margin) {
                double acc_v = std::clamp(my_front_speed + (my_front_dist - follow_margin) * 0.6, 0.0, MAX_CRUISE_SPEED);
                raw_target_v = std::min(raw_target_v, acc_v);
            } else {
                double acc_v = my_front_speed * 0.7; // 좁혀지면 앞차보다 확실히 느리게
                raw_target_v = std::min(raw_target_v, acc_v);
            }
        }

        double rel_speed_front = v.ego_speed - my_front_speed;
        double dynamic_trigger_dist = 15.0 + std::max(0.0, rel_speed_front) * 4.5;

        if (v.is_centered && lc_lock_timer_ == 0 && cooldown_timer_ == 0 &&
            my_front_dist < dynamic_trigger_dist && my_front_speed < MAX_CRUISE_SPEED * 0.9) {
           
            LaneDir best_dir = LaneDir::CENTER;
            double my_score = my_front_dist + (my_front_speed * 3.0);

            if (all_blocked) {
                my_score += 9999.0;
            }
           
            double best_score = my_score + 1.0;
           
            int check_indices[] = {0, 2};
            LaneDir check_dirs[] = {LaneDir::LEFT, LaneDir::RIGHT};
           
            for (int i = 0; i < 2; ++i) {
                int idx = check_indices[i];
                if (idx == 0 && v.left_line.empty()) continue;
                if (idx == 2 && v.right_line.empty()) continue;
               
                double target_front_x = v.lane_obs_dist[idx];
                double target_rear_x = std::abs(v.lane_rear_obs_dist[idx]);
               
                double rel_speed_target_rear = v.lane_rear_obs_speed[idx] - v.ego_speed;
                double dynamic_rear_margin = 0.5 + std::max(0.0, rel_speed_target_rear) * 1.5;

                if (target_front_x < 1.5 || target_rear_x < dynamic_rear_margin) continue;

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
                lc_lock_timer_ = 80;
            }
        }

        double rel_speed = v.ego_speed - my_front_speed;
        double emergency_dist = 0.8; // 긴급 제동 거리도 살짝 늘려 안정성 확보
        if (my_front_dist < emergency_dist || ((rel_speed > 0.5) && ((my_front_dist / rel_speed) < 0.6))) {
            result.state = BehaviorState::EMERGENCY_BRAKE;
            raw_target_v = 0.0;
        }

        if (result.state == BehaviorState::EMERGENCY_BRAKE) {
            current_target_v_ = 0.0;
        } else {
            // 속도가 빨라졌으므로 가감속을 좀 더 반응성 있게 변경
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
        myCar.curve_kappa = 0.0;
        myCar.vision_length = 2.0;
    }

    void rotate_pt(double &x, double &y, double theta) {
        double nx = x * std::cos(theta) - y * std::sin(theta);
        double ny = x * std::sin(theta) + y * std::cos(theta);
        x = nx; y = ny;
    }

    double get_y_from_line(const std::vector<geometry_msgs::msg::Point>& line, double target_x) {
        if (line.empty()) return 0.0;
        for (size_t i = 0; i < line.size() - 1; ++i) {
            if ((line[i].x <= target_x && line[i+1].x >= target_x) ||
                (line[i].x >= target_x && line[i+1].x <= target_x)) {
                double dx = line[i+1].x - line[i].x;
                if (std::abs(dx) < 1e-6) return line[i].y;
                return line[i].y + (target_x - line[i].x) / dx * (line[i+1].y - line[i].y);
            }
        }
        return line.front().y;
    }

    double get_path_length(const std::vector<geometry_msgs::msg::Point>& path) {
        double len = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            len += std::hypot(path[i].x - path[i-1].x, path[i].y - path[i-1].y);
        }
        return len;
    }

    bool get_frenet_pos(const std::vector<geometry_msgs::msg::Point>& ref_path, double target_s, double target_d, double &out_x, double &out_y, double &out_yaw) {
        if (ref_path.empty()) return false;
       
        double cur_s = 0.0;
        for (size_t i = 0; i < ref_path.size() - 1; ++i) {
            double dx = ref_path[i+1].x - ref_path[i].x;
            double dy = ref_path[i+1].y - ref_path[i].y;
            double dist = std::hypot(dx, dy);

            if (cur_s + dist >= target_s) {
                double ratio = (dist > 1e-6) ? (target_s - cur_s) / dist : 0.0;
                double rx = ref_path[i].x + ratio * dx;
                double ry = ref_path[i].y + ratio * dy;
                double ryaw = std::atan2(dy, dx);
               
                out_x = rx - target_d * std::sin(ryaw);
                out_y = ry + target_d * std::cos(ryaw);
                out_yaw = ryaw;
                return true;
            }
            cur_s += dist;
        }

        double last_yaw = 0.0;
        size_t n = ref_path.size();
        if (n >= 2) {
            last_yaw = std::atan2(ref_path[n-1].y - ref_path[n-2].y, ref_path[n-1].x - ref_path[n-2].x);
        }
        double excess = target_s - cur_s;
        double rx = ref_path.back().x + excess * std::cos(last_yaw);
        double ry = ref_path.back().y + excess * std::sin(last_yaw);
       
        out_x = rx - target_d * std::sin(last_yaw);
        out_y = ry + target_d * std::cos(last_yaw);
        out_yaw = last_yaw;
        return true;
    }

    void get_frenet_sd(const std::vector<geometry_msgs::msg::Point>& ref_path, double x, double y, double &out_s, double &out_d) {
        if (ref_path.size() < 2) { out_s = x; out_d = y; return; }
       
        double min_dist = 99999.0;
        int best_i = 0;
        for (size_t i = 0; i < ref_path.size(); ++i) {
            double dist = std::hypot(ref_path[i].x - x, ref_path[i].y - y);
            if (dist < min_dist) { min_dist = dist; best_i = i; }
        }

        int p1 = best_i;
        int p2 = best_i < (int)ref_path.size() - 1 ? best_i + 1 : best_i - 1;
        if (best_i > 0 && best_i < (int)ref_path.size() - 1) {
            double d1 = std::hypot(ref_path[best_i-1].x - x, ref_path[best_i-1].y - y);
            double d2 = std::hypot(ref_path[best_i+1].x - x, ref_path[best_i+1].y - y);
            if (d1 < d2) p2 = best_i - 1;
        }
        if (p1 > p2) std::swap(p1, p2);

        double dx = ref_path[p2].x - ref_path[p1].x;
        double dy = ref_path[p2].y - ref_path[p1].y;
        double len2 = dx*dx + dy*dy;
       
        double t = 0.0;
        if (len2 > 1e-6) t = ((x - ref_path[p1].x)*dx + (y - ref_path[p1].y)*dy) / len2;
        t = std::clamp(t, 0.0, 1.0);
       
        double px = ref_path[p1].x + t * dx;
        double py = ref_path[p1].y + t * dy;
       
        double cross = dx * (y - ref_path[p1].y) - dy * (x - ref_path[p1].x);
        out_d = std::hypot(x - px, y - py);
        if (cross < 0) out_d = -out_d;

        out_s = 0.0;
        for (int i = 0; i < p1; ++i) {
            out_s += std::hypot(ref_path[i+1].x - ref_path[i].x, ref_path[i+1].y - ref_path[i].y);
        }
        out_s += std::hypot(px - ref_path[p1].x, py - ref_path[p1].y);
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
           
        RCLCPP_INFO(this->get_logger(), "🏆 대회 레이싱 최적화 완료: 빠른 돌파를 위한 다이나믹 스피드 세팅 적용!");
    }

private:
    void lanes_callback(const perception::msg::Lanes::SharedPtr msg) {  
        if (msg->lanes.empty()) return;

        int best_ego_idx = -1; double min_y_abs = 999.0;
        for (size_t i = 0; i < msg->lanes.size(); ++i) {
            double y_int = std::abs(get_y_from_line(msg->lanes[i].points, 0.5));
            if (y_int < min_y_abs) { min_y_abs = y_int; best_ego_idx = i; }
        }

        if (best_ego_idx != -1 && min_y_abs < 0.35) {  
            myCar.ego_line = msg->lanes[best_ego_idx].points;
            myCar.is_centered = true;
           
            double raw_yaw = 0.0;
            if (myCar.ego_line.size() >= 3) {
                int lookahead_idx = std::min(10, (int)myCar.ego_line.size() - 1);
                double dx = myCar.ego_line[lookahead_idx].x - myCar.ego_line[0].x;
                double dy = myCar.ego_line[lookahead_idx].y - myCar.ego_line[0].y;
                raw_yaw = std::atan2(dy, dx);
            }
            smoothed_yaw_offset_ = 0.05 * raw_yaw + 0.95 * smoothed_yaw_offset_;

            double max_k = 0.0;
            if (myCar.ego_line.size() >= 3) {
                for (size_t i = 1; i < myCar.ego_line.size() - 1; ++i) {
                    double dx1 = myCar.ego_line[i].x - myCar.ego_line[i-1].x;
                    double dy1 = myCar.ego_line[i].y - myCar.ego_line[i-1].y;
                    double dx2 = myCar.ego_line[i+1].x - myCar.ego_line[i].x;
                    double dy2 = myCar.ego_line[i+1].y - myCar.ego_line[i].y;
                   
                    double len1 = std::hypot(dx1, dy1);
                    double len2 = std::hypot(dx2, dy2);
                   
                    if (len1 > 1e-3 && len2 > 1e-3) {
                        double yaw1 = std::atan2(dy1, dx1);
                        double yaw2 = std::atan2(dy2, dx2);
                        double dyaw = yaw2 - yaw1;
                       
                        while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
                        while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
                       
                        double k = std::abs(dyaw) / len1;
                        if (k > max_k) max_k = k;
                    }
                }
            }
            myCar.curve_kappa = max_k;
           
            myCar.vision_length = myCar.ego_line.empty() ? 0.0 : myCar.ego_line.back().x;

            double best_left_dist = 999.0;
            double best_right_dist = 999.0;
            bool found_left = false;
            bool found_right = false;
            std::vector<geometry_msgs::msg::Point> temp_left, temp_right;

            double ego_s_05 = 0.5, ego_s_12 = 1.2;

            for (size_t i = 0; i < msg->lanes.size(); ++i) {  
                if ((int)i == best_ego_idx) continue;
                if (msg->lanes[i].points.size() < 3) continue;
               
                auto cand_line = msg->lanes[i].points;
               
                double cx, cy, cyaw;
                if (get_frenet_pos(cand_line, 1.0, 0.0, cx, cy, cyaw)) {
                    double dum_s, lat_dist;
                    get_frenet_sd(myCar.ego_line, cx, cy, dum_s, lat_dist);
                   
                    if (lat_dist > 0.20 && lat_dist < 2.0) {  
                        if (lat_dist < best_left_dist) {
                            best_left_dist = lat_dist;
                            temp_left = cand_line;
                            found_left = true;
                        }
                    } else if (lat_dist < -0.20 && lat_dist > -2.0) {
                        if (std::abs(lat_dist) < best_right_dist) {
                            best_right_dist = std::abs(lat_dist);
                            temp_right = cand_line;
                            found_right = true;
                        }
                    }
                }
            }

            if (found_left) {
                myCar.left_line = temp_left;
                smoothed_left_offset_ = 0.05 * best_left_dist + 0.95 * smoothed_left_offset_;
                left_miss_count_ = 0;
                left_lane_exists_ = true;
            } else {
                left_miss_count_++;
                if (left_miss_count_ > 3) {
                    myCar.left_line.clear();
                    left_lane_exists_ = false;
                }
            }

            if (found_right) {
                myCar.right_line = temp_right;
                smoothed_right_offset_ = 0.05 * (-best_right_dist) + 0.95 * smoothed_right_offset_;
                right_miss_count_ = 0;
                right_lane_exists_ = true;
            } else {
                right_miss_count_++;
                if (right_miss_count_ > 3) {
                    myCar.right_line.clear();
                    right_lane_exists_ = false;
                }
            }
        } else {
            myCar.is_centered = false;
        }
    }

    void objects_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        reset_sensor_data();
        if (msg->poses.empty() || myCar.ego_line.empty()) return;

        for (const auto& pose : msg->poses) {
            double ox = pose.position.x, oy = pose.position.y, os = pose.position.z;
            if (std::abs(ox) < 0.1 && std::abs(oy) < 0.2) continue;
           
            myCar.obstacles.push_back({ox, oy, os});
           
            double obs_s, obs_d;
            get_frenet_sd(myCar.ego_line, ox, oy, obs_s, obs_d);
           
            int obs_lane = -1;
            if (std::abs(obs_d) < 0.22) obs_lane = 1;
            else if (obs_d >= 0.22 && obs_d < 0.80) obs_lane = 0;
            else if (obs_d <= -0.22 && obs_d > -0.80) obs_lane = 2;

            if (obs_lane != -1) {  
                if (obs_s >= -0.4 && obs_s < myCar.lane_obs_dist[obs_lane]) {
                    myCar.lane_obs_dist[obs_lane] = obs_s;
                    myCar.lane_obs_speed[obs_lane] = os;
                }
                if (obs_s < -0.1 && obs_s > myCar.lane_rear_obs_dist[obs_lane]) {
                    myCar.lane_rear_obs_dist[obs_lane] = obs_s;
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

        double max_visible_s = 2.0;  
        double longest_vision = 0.0;
       
        if (!myCar.left_line.empty()) longest_vision = std::max(longest_vision, get_path_length(myCar.left_line));
        if (!myCar.ego_line.empty()) longest_vision = std::max(longest_vision, get_path_length(myCar.ego_line));
        if (!myCar.right_line.empty()) longest_vision = std::max(longest_vision, get_path_length(myCar.right_line));
       
        if (longest_vision > 0.1) {
            max_visible_s = std::clamp(longest_vision, 1.0, 2.0);
        }

        if (myCar.ego_line.empty()) {  
            for (double x = 0; x <= max_visible_s; x += 0.2) {
                geometry_msgs::msg::PoseStamped p;
                p.header = path_msg.header; p.pose.position.x = x; p.pose.position.y = 0.0;
                path_msg.poses.push_back(p);
            }
            path_pub_->publish(path_msg);
            viz_path_pub_->publish(path_msg);
            return;
        }

        if (is_changing_lane_) driven_dist_ += (myCar.ego_speed * dt);
        else driven_dist_ = 0.0;  

        std::vector<FrenetObstacle> frenet_obstacles;
        for (auto obs : myCar.obstacles) {
            double s, d;
            get_frenet_sd(myCar.ego_line, obs.x, obs.y, s, d);
            frenet_obstacles.push_back({s, d, obs.speed});
        }

        std::vector<double> candidate_lengths = {1.0, 1.4, 1.8};
        std::vector<Tentacle> left_tentacles, right_tentacles;  

        auto generate_tentacles = [&](const std::vector<geometry_msgs::msg::Point>& target_line, double default_offset, std::vector<Tentacle>& out_tentacles, bool is_active_target) {  
           
            if (target_line.empty() && !is_active_target) return;

            for (double L : candidate_lengths) {
                Tentacle t;
                t.L_total = L;
                t.collision = false;  
               
                for (double local_s = 0.0; local_s <= max_visible_s; local_s += 0.1) {
                    double world_s = driven_dist_ + local_s;
                    double progress = std::clamp(world_s / L, 0.0, 1.0);
                    double dodge_blend = 0.5 * (1.0 - std::cos(progress * M_PI));
                   
                    double target_d = default_offset;
                    if (!target_line.empty()) {
                        double tx, ty, tyaw;
                        get_frenet_pos(target_line, local_s, 0.0, tx, ty, tyaw);
                       
                        if (local_s > get_path_length(target_line) + 0.3) {
                            target_d = default_offset;
                        } else {
                            double dum_s;
                            get_frenet_sd(myCar.ego_line, tx, ty, dum_s, target_d);
                        }
                    }
                   
                    double path_d = 0.0 + (target_d - 0.0) * dodge_blend;
                   
                    double real_x, real_y, path_yaw;
                    get_frenet_pos(myCar.ego_line, local_s, path_d, real_x, real_y, path_yaw);

                    double ego_v = std::max(myCar.ego_speed, 1.0);
                    double time_to_reach = local_s / ego_v;

                    for (const auto& obs : frenet_obstacles) {
                        double rel_v = obs.speed - myCar.ego_speed;
                        double future_obs_s = obs.s + (rel_v * time_to_reach);

                        if (future_obs_s > -0.5 && future_obs_s < max_visible_s) {
                            if (std::hypot(future_obs_s - local_s, obs.d - path_d) < 0.20) {
                                t.collision = true; break;
                            }
                        }
                    }

                    geometry_msgs::msg::PoseStamped p;
                    p.pose.position.x = real_x;
                    p.pose.position.y = real_y;
                    t.poses.push_back(p);  
                }
                t.cost = std::abs(2.2 - L);
                out_tentacles.push_back(t);  
            }
        };

        bool is_going_left = (is_changing_lane_ && active_change_dir_ == LaneDir::LEFT);
        bool is_going_right = (is_changing_lane_ && active_change_dir_ == LaneDir::RIGHT);
       
        bool keep_left = left_lane_exists_ || is_going_left;
        bool keep_right = right_lane_exists_ || is_going_right;

        if (keep_left) generate_tentacles(myCar.left_line, smoothed_left_offset_, left_tentacles, is_going_left);
        if (keep_right) generate_tentacles(myCar.right_line, smoothed_right_offset_, right_tentacles, is_going_right);  

        int blue_tentacles_count = 0;
        if (decision.target_lane_dir != LaneDir::CENTER) {
            std::vector<Tentacle>* t_list = (decision.target_lane_dir == LaneDir::LEFT) ? &left_tentacles : &right_tentacles;
            for(const auto& t : *t_list) if(!t.collision) blue_tentacles_count++;
        }  

        if (decision.target_lane_dir != LaneDir::CENTER && !is_changing_lane_) {
            std::vector<Tentacle>* t_list = (decision.target_lane_dir == LaneDir::LEFT) ? &left_tentacles : &right_tentacles;
            double best_L = -1.0;
            double min_cost = 9999.0;
           
            for (const auto& t : *t_list) {
                if (!t.collision) {
                    double opportunistic_cost = (blue_tentacles_count <= 2) ? std::abs(1.5 - t.L_total) : t.cost;
                    if (opportunistic_cost < min_cost) {
                        min_cost = opportunistic_cost;
                        best_L = t.L_total;
                    }
                }
            }

            if (best_L > 0.0) {
                is_changing_lane_ = true;
                active_change_dir_ = decision.target_lane_dir;
                locked_total_L_ = best_L;
                driven_dist_ = 0.0;
            } else {
                brain.cancel_lane_change();
            }  
        }

        visualization_msgs::msg::MarkerArray candidate_markers;  
       
        auto draw_markers = [&](const std::vector<Tentacle>& t_list, const std::string& ns, bool is_active_side) {
            for (size_t i = 0; i < candidate_lengths.size(); ++i) {
                visualization_msgs::msg::Marker m;
                m.header.frame_id = "base_link"; m.header.stamp = now;
                m.ns = ns;
                m.id = i;
               
                if (t_list.empty() || i >= t_list.size()) {
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

                if (is_changing_lane_ && is_active_side && std::abs(t.L_total - locked_total_L_) < 0.1) {
                    m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;
                    m.scale.x = 0.12;
                } else {
                    m.scale.x = 0.04;
                    if (t.collision) {
                        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.6;
                    } else {
                        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.6;
                    }
                }
                candidate_markers.markers.push_back(m);
            }
        };

        draw_markers(left_tentacles, "tentacles_left", is_going_left);
        draw_markers(right_tentacles, "tentacles_right", is_going_right);
        candidates_pub_->publish(candidate_markers);  

        std::vector<geometry_msgs::msg::PoseStamped> final_path_poses;  

        if (is_changing_lane_) {
            std::vector<Tentacle>* active_t_list = (active_change_dir_ == LaneDir::LEFT) ? &left_tentacles : &right_tentacles;  
           
            bool path_still_safe = false;
            for (const auto& t : *active_t_list) {
                if (std::abs(t.L_total - locked_total_L_) < 0.1) {
                    final_path_poses = t.poses;
                    if (!t.collision) path_still_safe = true;
                    break;
                }  
            }

            if (!path_still_safe && driven_dist_ < locked_total_L_ * 0.2) {
                is_changing_lane_ = false;
                active_change_dir_ = LaneDir::CENTER;
                brain.cancel_lane_change();  
            }

            double current_ego_y = get_y_from_line(myCar.ego_line, 0.5);
            bool swap_detected = false;
           
            if (driven_dist_ > locked_total_L_ * 0.4) {
                if (active_change_dir_ == LaneDir::LEFT && current_ego_y > 0.10) swap_detected = true;
                else if (active_change_dir_ == LaneDir::RIGHT && current_ego_y < -0.10) swap_detected = true;
            }

            if (swap_detected || driven_dist_ >= locked_total_L_) {
                is_changing_lane_ = false;
                active_change_dir_ = LaneDir::CENTER;
                brain.complete_lane_change();  
            }
        }
       
        if (!is_changing_lane_) {
            for (double s = 0.0; s <= max_visible_s; s += 0.2) {
                double real_x, real_y, real_yaw;
                get_frenet_pos(myCar.ego_line, s, 0.0, real_x, real_y, real_yaw);
               
                geometry_msgs::msg::PoseStamped p;
                p.pose.position.x = real_x;
                p.pose.position.y = real_y;
                final_path_poses.push_back(p);  
            }
        }
       
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