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

// 🌟 차량의 현재 주행 상태를 정의하는 열거형
enum class BehaviorState { KEEP_LANE_CRUISE, LANE_CHANGE, EMERGENCY_BRAKE };
// 🌟 목표 차선의 방향 (왼쪽: -1, 중앙/유지: 0, 오른쪽: 1)
enum class LaneDir { LEFT = -1, CENTER = 0, RIGHT = 1 };

// 장애물의 X, Y 좌표와 속도 정보를 담는 구조체
struct Obstacle {
    double x; double y; double speed;
};

// 🌟 차량의 현재 환경 인식 정보를 모두 모아두는 '상황판' 구조체
struct VehicleInfo {
    // 원본 차선 점 데이터
    std::vector<geometry_msgs::msg::Point> left_line;
    std::vector<geometry_msgs::msg::Point> ego_line;
    std::vector<geometry_msgs::msg::Point> right_line;
   
    // 차량의 조향각을 보정하여 일직선으로 쭉 펴놓은 차선 데이터 (계산 편의성 목적)
    std::vector<geometry_msgs::msg::Point> straight_ego_line;
    std::vector<geometry_msgs::msg::Point> straight_left_line;  
    std::vector<geometry_msgs::msg::Point> straight_right_line;

    // 각 차선(0:왼쪽, 1:내차선, 2:오른쪽)의 전방/후방 장애물 거리와 속도
    double lane_obs_dist[3];
    double lane_obs_speed[3];
    double lane_rear_obs_dist[3];
    double lane_rear_obs_speed[3];

    double ego_speed;     // 내 차량의 현재 속도
    bool   is_centered;   // 내가 차선 중앙을 잘 물고 있는지 여부
    std::vector<Obstacle> obstacles; // 전체 장애물 리스트
};

// 브레인(DecisionMaker)이 내린 최종 명령을 담는 구조체
struct DecisionResult {
    BehaviorState state;        
    LaneDir target_lane_dir;
    double target_speed;
};

// 🌟 회피 기동의 핵심! 나비(촉수) 하나의 정보를 담는 구조체
struct Tentacle {
    double L_total; // 나비가 뻗어나갈 목표 거리 (얼마나 길고 부드럽게 꺾을 것인가)
    bool collision; // 이 나비를 따라가면 장애물과 부딪히는지 여부 (사망 판정)
    double cost;    // 이 나비의 점수 (낮을수록 좋음. 보통 1.8m처럼 길고 부드러운 것을 선호)
    std::vector<geometry_msgs::msg::PoseStamped> poses; // 0.1m 간격으로 찍힌 나비의 실제 점(궤적)들
};

// 🧠 차량의 뇌 역할을 하는 의사결정 클래스
class DecisionMaker {
private:
    double current_target_v_ = 0.0;
    int lc_lock_timer_ = 0; // 차선 변경을 시작하면 딴짓 못하게 락을 거는 타이머
    int cooldown_timer_ = 0; // 차선 변경이 끝난 후 잠시 대기하는 쿨타임
    LaneDir committed_dir_ = LaneDir::CENTER; // 현재 결심한 차선 변경 방향

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
            lc_lock_timer_--; // 락 타이머 감소
        } else {
            committed_dir_ = LaneDir::CENTER; // 락이 풀리면 기본은 중앙 유지
        }

        result.target_lane_dir = committed_dir_;
        result.state = (lc_lock_timer_ > 0) ? BehaviorState::LANE_CHANGE : BehaviorState::KEEP_LANE_CRUISE;

        // 현재 진행하려는 차선(기본은 내 차선)의 앞차 정보 가져오기
        int active_idx = (committed_dir_ == LaneDir::LEFT) ? 0 : ((committed_dir_ == LaneDir::RIGHT) ? 2 : 1);
        double my_front_dist = v.lane_obs_dist[active_idx];
        double my_front_speed = v.lane_obs_speed[active_idx];
       
        double MAX_CRUISE_SPEED = 3.5;
        double raw_target_v = MAX_CRUISE_SPEED;
       
        // 🎯 적응형 순항 제어 (ACC) 로직: 앞차와의 거리에 맞춰 속도 조절
        if (my_front_dist < 15.0) {
            double follow_margin = 2.5; // 유지하고 싶은 안전거리 (2.5m)
            if (my_front_dist > follow_margin) {
                // 안전거리보다 멀면 앞차 속도에 맞춰가며 서서히 접근
                raw_target_v = std::clamp(my_front_speed + (my_front_dist - follow_margin) * 0.5, 0.0, MAX_CRUISE_SPEED);
            } else {
                // 안전거리 이내로 좁혀지면 앞차보다 살짝 느리게 가서 거리 벌리기
                raw_target_v = my_front_speed * 0.8;
            }
        }

        // 상대 속도를 기반으로 차선을 바꿀 다이나믹 거리 계산 (빠르게 다가가면 더 멀리서부터 차선 변경 준비)
        double rel_speed_front = v.ego_speed - my_front_speed;
        double dynamic_trigger_dist = 15.0 + std::max(0.0, rel_speed_front) * 4.0;

        // 🎯 차선 변경 점수(Score) 계산 로직
        // 조건: 중앙을 잘 물고 있고, 쿨타임이 없고, 앞이 막혔을 때
        if (v.is_centered && lc_lock_timer_ == 0 && cooldown_timer_ == 0 &&
            my_front_dist < dynamic_trigger_dist && my_front_speed < MAX_CRUISE_SPEED * 0.9) {
           
            LaneDir best_dir = LaneDir::CENTER;
            // 내 차선의 점수 = 뚫린 거리 + (앞차 속도 * 가중치)
            double my_score = my_front_dist + (my_front_speed * 3.0);
           
            double best_score = my_score + 1.0; // 변경하려면 내 차선보다 최소 1점은 높아야 함
           
            int check_indices[] = {0, 2}; // 좌측(0), 우측(2) 차선 확인
            LaneDir check_dirs[] = {LaneDir::LEFT, LaneDir::RIGHT};
           
            for (int i = 0; i < 2; ++i) {
                int idx = check_indices[i];
                if (idx == 0 && v.left_line.empty()) continue; // 왼쪽 선이 없으면 패스
                if (idx == 2 && v.right_line.empty()) continue; // 오른쪽 선이 없으면 패스
               
                double target_front_x = v.lane_obs_dist[idx];
                double target_rear_x = std::abs(v.lane_rear_obs_dist[idx]);
               
                // 타겟 차선 뒤차의 접근 속도에 따라 안전 마진 확보
                double rel_speed_target_rear = v.lane_rear_obs_speed[idx] - v.ego_speed;
                double dynamic_rear_margin = 0.5 + std::max(0.0, rel_speed_target_rear) * 1.5;

                // 타겟 차선에 들어갈 공간이 안 나오면(앞이 막히거나 뒤차가 너무 빠르면) 패스
                if (target_front_x < 1.0 || target_rear_x < dynamic_rear_margin) continue;

                // 타겟 차선의 점수 산출
                double score = target_front_x + (v.lane_obs_speed[idx] * 4.0);
                if (score > best_score) {
                    best_score = score;
                    best_dir = check_dirs[i]; // 점수가 가장 높은 차선으로 결정
                }
            }

            // 변경할 차선이 결정되면 락 타이머를 걸고 방향을 확정
            if (best_dir != LaneDir::CENTER) {
                result.state = BehaviorState::LANE_CHANGE;
                result.target_lane_dir = best_dir;
                committed_dir_ = best_dir;
                lc_lock_timer_ = 80;
            }
        }

        // 🎯 긴급 제동 (AEB) 로직
        double rel_speed = v.ego_speed - my_front_speed;
        double emergency_dist = 0.5; // 0.5m 이내로 들어오거나, 쾅 부딪히기 직전이면
        if (my_front_dist < emergency_dist || ((rel_speed > 0.5) && ((my_front_dist / rel_speed) < 0.5))) {
            result.state = BehaviorState::EMERGENCY_BRAKE;
            raw_target_v = 0.0; // 풀브레이크
        }

        // 속도 스무딩 (부드러운 가감속)
        if (result.state == BehaviorState::EMERGENCY_BRAKE) {
            current_target_v_ = 0.0;
        } else {
            double alpha = (raw_target_v < current_target_v_) ? 0.4 : 0.1; // 감속은 팍(0.4), 가속은 스무스하게(0.1)
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

    double smoothed_left_offset_ = 0.45;    // 차선 폭
    double smoothed_right_offset_ = -0.45;
    double smoothed_yaw_offset_ = 0.0;

    void reset_sensor_data() {
        for(int i=0; i<3; i++) {
            myCar.lane_obs_dist[i] = 30.0; myCar.lane_obs_speed[i] = 0.0;
            myCar.lane_rear_obs_dist[i] = -30.0; myCar.lane_rear_obs_speed[i] = 0.0;
        }
        myCar.obstacles.clear();
    }

    // 🌟 좌표 회전 함수: 차량이 비뚤어지게 달릴 때, 차선과 장애물을 모두 똑바로 정렬시킵니다.
    void rotate_pt(double &x, double &y, double theta) {
        double nx = x * std::cos(theta) - y * std::sin(theta);
        double ny = x * std::sin(theta) + y * std::cos(theta);
        x = nx; y = ny;
    }

    // 🌟 선형 보간법(Linear Interpolation) 함수: 점과 점 사이를 선으로 이어 특정 x 위치에서의 y(차선 횡방향 위치)를 계산
    double get_y_from_line(const std::vector<geometry_msgs::msg::Point>& line, double target_x) {
        if (line.empty()) return 0.0;
        if (target_x < line.front().x) {
            if (line.size() >= 2) {
                double dx = line[1].x - line[0].x;
                double dy = line[1].y - line[0].y;
                if (std::abs(dx) > 1e-4) {
                    double slope = std::clamp(dy / dx, -0.25, 0.25);
                    return line[0].y + slope * (target_x - line[0].x);
                }
            }
            return line.front().y;
        }
        // X값이 주어지면 어느 두 점 사이에 있는지 찾아서 비율로 Y값을 추정합니다.
        for (size_t i = 0; i < line.size() - 1; ++i) {
            if ((line[i].x <= target_x && line[i+1].x >= target_x) ||
                (line[i].x >= target_x && line[i+1].x <= target_x)) {
                double dx = line[i+1].x - line[i].x;
                if (std::abs(dx) < 1e-6) return line[i].y;
                return line[i].y + (target_x - line[i].x) / dx * (line[i+1].y - line[i].y);
            }
        }
        // 끝점을 넘어갔을 때 예측하는 로직
        int n = line.size();
        if (n >= 2 && target_x > line.back().x) {
            double dx = line[n-1].x - line[n-2].x;
            double dy = line[n-1].y - line[n-2].y;
            if (std::abs(dx) > 1e-4) {
                double slope = std::clamp(dy / dx, -0.25, 0.25);
                return line[n-1].y + slope * (target_x - line[n-1].x);
            }
        }
        return line.back().y;
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
           
        RCLCPP_INFO(this->get_logger(), "플레닝 노드 구동 완료 (원본 로직 완벽 복구본)");
    }

private:
    // 📸 카메라 데이터 수신 (차선 인식)
    void lanes_callback(const perception::msg::Lanes::SharedPtr msg) {  
        if (msg->lanes.empty()) return;

        // 1. 내 차선 찾기: Y 절편(좌우 치우침)이 가장 0에 가까운 차선을 내 차선으로 간주
        int best_ego_idx = -1; double min_y_abs = 999.0;
        for (size_t i = 0; i < msg->lanes.size(); ++i) {
            double y_int = std::abs(get_y_from_line(msg->lanes[i].points, 0.5));
            if (y_int < min_y_abs) { min_y_abs = y_int; best_ego_idx = i; }
        }

        // 차선을 잘 물고 있다면 (0.35 이내)
        if (best_ego_idx != -1 && min_y_abs < 0.35) {  
            myCar.ego_line = msg->lanes[best_ego_idx].points;
            myCar.is_centered = true;
           
            // 2. 조향각 오차 보정 (Yaw Smoothing)
            // 내 차선이 앞쪽에서 얼마나 기울어 있는지(raw_yaw) 파악하여, 차량 좌표계를 살짝 돌려 연산을 똑바로 맞춤
            double raw_yaw = 0.0;
            if (myCar.ego_line.size() >= 3) {
                int lookahead_idx = std::min(10, (int)myCar.ego_line.size() - 1);
                double dx = myCar.ego_line[lookahead_idx].x - myCar.ego_line[0].x;
                double dy = myCar.ego_line[lookahead_idx].y - myCar.ego_line[0].y;
                raw_yaw = std::atan2(dy, dx);
            }
            smoothed_yaw_offset_ = 0.05 * raw_yaw + 0.95 * smoothed_yaw_offset_;

            // 회전 보정된 내 차선 생성
            myCar.straight_ego_line.clear();
            for (auto pt : myCar.ego_line) {
                double nx = pt.x, ny = pt.y;
                rotate_pt(nx, ny, -smoothed_yaw_offset_);
                pt.x = nx; pt.y = ny;
                myCar.straight_ego_line.push_back(pt);
            }

            double ego_rot_y_05 = get_y_from_line(myCar.straight_ego_line, 0.5); // 내 차선의 기준 앞 0.5m
            double ego_rot_y_12 = get_y_from_line(myCar.straight_ego_line, 1.2); // 내 차선의 기준 앞 1.2m

            double best_left_dist = 999.0;
            double best_right_dist = 999.0;
            bool found_left = false;
            bool found_right = false;
            std::vector<geometry_msgs::msg::Point> temp_left, temp_right;

            // 3. 나머지 차선들을 검사하여 좌측 차선인지, 우측 차선인지 분류
            for (size_t i = 0; i < msg->lanes.size(); ++i) {  
                if ((int)i == best_ego_idx) continue;
               
                if (msg->lanes[i].points.size() < 3) continue;
                double max_x = 0.0;
                for (const auto& p : msg->lanes[i].points) {
                    if (p.x > max_x) max_x = p.x;
                }
                if (max_x < 1.0) continue;

                // 후보 차선도 똑같이 회전 보정 적용
                std::vector<geometry_msgs::msg::Point> straight_cand_line;
                for (auto pt : msg->lanes[i].points) {
                    double nx = pt.x, ny = pt.y;
                    rotate_pt(nx, ny, -smoothed_yaw_offset_);
                    pt.x = nx; pt.y = ny;
                    straight_cand_line.push_back(pt);
                }

                double target_rot_y_05 = get_y_from_line(straight_cand_line, 0.5);
                double target_rot_y_12 = get_y_from_line(straight_cand_line, 1.2);
               
                double lat_dist_05 = target_rot_y_05 - ego_rot_y_05;
                double lat_dist_12 = target_rot_y_12 - ego_rot_y_12;
               
                // 내 차선과 기울기가 너무 달라서 교차할 것 같으면 버림 (0.3 오차 허용)
                if (std::abs(lat_dist_05 - lat_dist_12) > 0.3) continue;

                double lat_dist = lat_dist_05;
               
                // 가로 거리가 양수(+)이면 왼쪽 차선, 음수(-)이면 오른쪽 차선으로 분류
                if (lat_dist > 0.20 && lat_dist < 1.5) {  
                    if (lat_dist < best_left_dist) {
                        best_left_dist = lat_dist;
                        temp_left = straight_cand_line;
                        found_left = true;
                    }
                } else if (lat_dist < -0.20 && lat_dist > -1.5) {
                    if (std::abs(lat_dist) < best_right_dist) {
                        best_right_dist = std::abs(lat_dist);
                        temp_right = straight_cand_line;
                        found_right = true;
                    }
                }
            }

            // 왼쪽 차선 등록 (가려져서 못 봤을 경우 3번까지는 유지해주는 미스 카운트 로직 적용)
            if (found_left) {
                myCar.left_line = msg->lanes[best_ego_idx].points;
                myCar.straight_left_line = temp_left;
                smoothed_left_offset_ = 0.05 * best_left_dist + 0.95 * smoothed_left_offset_;
                left_miss_count_ = 0;
                left_lane_exists_ = true;
            } else {
                left_miss_count_++;
                if (left_miss_count_ > 3) {
                    myCar.left_line.clear();
                    myCar.straight_left_line.clear();
                    left_lane_exists_ = false;
                }
            }

            // 오른쪽 차선 등록
            if (found_right) {
                myCar.right_line = msg->lanes[best_ego_idx].points;
                myCar.straight_right_line = temp_right;
                smoothed_right_offset_ = 0.05 * (-best_right_dist) + 0.95 * smoothed_right_offset_;
                right_miss_count_ = 0;
                right_lane_exists_ = true;
            } else {
                right_miss_count_++;
                if (right_miss_count_ > 3) {
                    myCar.right_line.clear();
                    myCar.straight_right_line.clear();
                    right_lane_exists_ = false;
                }
            }
        } else {
            myCar.is_centered = false; // 차선을 너무 심하게 벗어남
        }
    }

    // 📸 장애물 데이터 수신
    void objects_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        reset_sensor_data();
        if (msg->poses.empty() || myCar.straight_ego_line.empty()) return;

        for (const auto& pose : msg->poses) {
            double ox = pose.position.x, oy = pose.position.y, os = pose.position.z;
            if (std::abs(ox) < 0.1 && std::abs(oy) < 0.2) continue; // 나 자신(Ego)이 스캔된 경우 무시
           
            myCar.obstacles.push_back({ox, oy, os});
           
            // 장애물 위치도 차선과 똑같이 회전 보정
            double rot_ox = ox, rot_oy = oy;
            rotate_pt(rot_ox, rot_oy, -smoothed_yaw_offset_);

            double expected_y = get_y_from_line(myCar.straight_ego_line, rot_ox);
            double lat_diff = rot_oy - expected_y;  // 장애물이 내 차선 중심(y)에서 얼마나 좌우로 떨어져 있는지 계산
           
            // 장애물이 속한 차선 판별 (0.22 이내는 내 차선, 그 이상은 좌/우 차선)
            int obs_lane = -1;
            if (std::abs(lat_diff) < 0.22) obs_lane = 1;
            else if (lat_diff >= 0.22 && lat_diff < 0.80) obs_lane = 0;
            else if (lat_diff <= -0.22 && lat_diff > -0.80) obs_lane = 2;

            // 각 차선별로 "가장 가까운 앞차"와 "가장 가까운 뒤차"를 찾아서 기록
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

    // 🕒 핵심 플래닝 루프 (50ms 마다 실행)
    void timer_callback() {
        auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        if (dt > 0.2 || dt <= 0.0) dt = 0.1;
        last_time_ = now;

        // 브레인(DecisionMaker)을 호출하여 이번 턴에 뭘 할지(차선유지/변경) 결정
        DecisionResult decision = brain.decide(myCar);
       
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "base_link";
        path_msg.header.stamp = now;

        // 🌟 나비가 뻗어나갈 최대 거리 설정 (전방 2m까지만 탐색)
        double max_visible_x = 2.0;  
        if (!myCar.straight_ego_line.empty()) {
            max_visible_x = std::min(2.0, myCar.straight_ego_line.back().x);
        }

        // 차선 인식이 아예 안 되면 그냥 앞을 향해 2m짜리 직선 더미 경로를 뿌려주고 종료
        if (myCar.straight_ego_line.empty()) {  
            for (double x = 0; x <= max_visible_x; x += 0.2) {
                geometry_msgs::msg::PoseStamped p;
                p.header = path_msg.header; p.pose.position.x = x; p.pose.position.y = 0.0;
                path_msg.poses.push_back(p);
            }
            path_pub_->publish(path_msg);
            viz_path_pub_->publish(path_msg);
            return;
        }

        // 차선 변경 중이면 얼마나 달렸는지(진척도) 누적 추적
        if (is_changing_lane_) driven_dist_ += (myCar.ego_speed * dt);
        else driven_dist_ = 0.0;  

        std::vector<Obstacle> straight_obstacles;
        for (auto obs : myCar.obstacles) {
            double ox = obs.x, oy = obs.y;
            rotate_pt(ox, oy, -smoothed_yaw_offset_);
            straight_obstacles.push_back({ox, oy, obs.speed});
        }

        // 🌟 텐터클(나비) 알고리즘: 짧은 변경(1.0), 중간(1.4), 부드러운 변경(1.8) 후보군 생성
        std::vector<double> candidate_lengths = {1.0, 1.4, 1.8};
        std::vector<Tentacle> left_tentacles, right_tentacles;  

        // 나비를 그려주는 내부 람다 함수
        auto generate_tentacles = [&](const std::vector<geometry_msgs::msg::Point>& target_line, double default_offset, std::vector<Tentacle>& out_tentacles) {  
            for (double L : candidate_lengths) {
                Tentacle t;
                t.L_total = L;
                t.collision = false;  
               
                // 0.1m 촘촘한 간격으로 점을 찍으며 전진
                for (double local_x = 0.0; local_x <= max_visible_x; local_x += 0.1) {
                    double world_x = driven_dist_ + local_x;
                    double progress = std::clamp(world_x / L, 0.0, 1.0); // 0.0(시작) ~ 1.0(도착)
                   
                    // 🌟 환상적인 Cosine 보간법: 0.0 -> 1.0 으로 가는 비율을 코사인 곡선으로 둥글게 깎음 (S자 곡선 생성의 핵심)
                    double dodge_blend = 0.5 * (1.0 - std::cos(progress * M_PI));
                    double sy = get_y_from_line(myCar.straight_ego_line, local_x); // 내 차선 Y 좌표
                   
                    double ty = sy + default_offset; // 타겟 차선 Y 좌표
                    if (!target_line.empty()) {
                        ty = get_y_from_line(target_line, local_x);
                    }
                   
                    // 내 차선(sy)에서 타겟 차선(ty)으로 dodge_blend 비율만큼 부드럽게 점을 밀어버림!
                    double path_y = sy + (ty - sy) * dodge_blend;
                   
                    // 🌟 미래 시공간 예측 충돌 검사
                    double ego_v = std::max(myCar.ego_speed, 1.0);
                    double time_to_reach = local_x / ego_v; // 내가 이 점에 도달할 시간

                    for (const auto& obs : straight_obstacles) {
                        double rel_v = obs.speed - myCar.ego_speed;
                        double future_obs_x = obs.x + (rel_v * time_to_reach); // 그 시간 뒤에 상대 차가 가있을 위치

                        if (future_obs_x > -0.5 && future_obs_x < max_visible_x) {
                            // 미래에 내 나비의 점과 상대 차의 거리가 20cm 이내로 좁혀지면 이 나비는 사망(collision = true) 판정
                            if (std::hypot(future_obs_x - local_x, obs.y - path_y) < 0.20) {
                                t.collision = true; break;
                            }
                        }
                    }

                    // 계산된 점을 다시 원래 맵 기준(시야 각도 복구)으로 되돌려서 저장
                    double real_x = local_x;
                    double real_y = path_y;
                    rotate_pt(real_x, real_y, smoothed_yaw_offset_);

                    geometry_msgs::msg::PoseStamped p;
                    p.pose.position.x = real_x;
                    p.pose.position.y = real_y;
                    t.poses.push_back(p);  
                }
                // 나비의 점수. 2.2m에 가까울수록 점수가 낮아(좋아)서 긴 나비를 우선적으로 선택하게 함.
                t.cost = std::abs(2.2 - L);
                out_tentacles.push_back(t);  
            }
        };

        bool is_going_left = (is_changing_lane_ && active_change_dir_ == LaneDir::LEFT);
        bool is_going_right = (is_changing_lane_ && active_change_dir_ == LaneDir::RIGHT);
       
        bool keep_left = left_lane_exists_ || is_going_left;
        bool keep_right = right_lane_exists_ || is_going_right;

        // 양쪽 차선에 나비(후보군)들을 쫙 뿌립니다.
        if (keep_left) generate_tentacles(myCar.straight_left_line, smoothed_left_offset_, left_tentacles);
        if (keep_right) generate_tentacles(myCar.straight_right_line, smoothed_right_offset_, right_tentacles);  

        // 🌟 타겟 방향의 나비들 중 충돌 없는 파란색 나비가 몇 개인지 카운트
        int blue_tentacles_count = 0;
        if (decision.target_lane_dir != LaneDir::CENTER) {
            std::vector<Tentacle>* t_list = (decision.target_lane_dir == LaneDir::LEFT) ? &left_tentacles : &right_tentacles;
            for(const auto& t : *t_list) if(!t.collision) blue_tentacles_count++;
        }  

        // 차선 변경을 결심했다면, 만들어둔 나비 중 가장 좋은 놈을 고르기
        if (decision.target_lane_dir != LaneDir::CENTER && !is_changing_lane_) {
            std::vector<Tentacle>* t_list = (decision.target_lane_dir == LaneDir::LEFT) ? &left_tentacles : &right_tentacles;
            double best_L = -1.0;
            double min_cost = 9999.0;
           
            for (const auto& t : *t_list) {
                if (!t.collision) {
                    // 기회가 많을 땐 가장 부드러운 놈(1.5m 등)을 고르고, 급할 땐 아무거나 안전한 놈으로!
                    double opportunistic_cost = (blue_tentacles_count <= 2) ? std::abs(1.5 - t.L_total) : t.cost;
                    if (opportunistic_cost < min_cost) {
                        min_cost = opportunistic_cost;
                        best_L = t.L_total; // 우승 나비 당첨
                    }
                }
            }

            // 우승한 나비의 길이를 확정(Lock)하고 변경을 시작
            if (best_L > 0.0) {
                is_changing_lane_ = true;
                active_change_dir_ = decision.target_lane_dir;
                locked_total_L_ = best_L;
                driven_dist_ = 0.0;
            } else {
                // 부딪힐 나비밖에 없으면 차선 변경을 취소
                brain.cancel_lane_change();
            }  
        }

        // Rviz 에 나비(경로)를 시각화해서 뿌려주는 함수 (개발용)
        visualization_msgs::msg::MarkerArray candidate_markers;  
        auto draw_markers = [&](const std::vector<Tentacle>& t_list, const std::string& ns, bool is_active_side) {
            for (size_t i = 0; i < candidate_lengths.size(); ++i) {
                visualization_msgs::msg::Marker m;
                m.header.frame_id = "base_link"; m.header.stamp = now;
                m.ns = ns;
                m.id = i;
               
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

                // 확정된 궤적은 굵은 노란색, 충돌 위험은 빨간색, 안전한 후보는 얇은 청록색으로 표시
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

        // 🌟 차량을 구동시키는 실제 최종 궤적 조립 구간 🌟
        if (is_changing_lane_) {
            std::vector<Tentacle>* active_t_list = (active_change_dir_ == LaneDir::LEFT) ? &left_tentacles : &right_tentacles;  
           
            bool path_still_safe = false;
            for (const auto& t : *active_t_list) {
                if (std::abs(t.L_total - locked_total_L_) < 0.1) {
                    final_path_poses = t.poses; // 아까 당첨된 나비를 꺼내서 최종 궤적으로 배정!
                    if (!t.collision) path_still_safe = true;
                    break;
                }  
            }

            // 차선 변경 초반(20% 진행)에 갑자기 막히면, 과감하게 포기하고 원래 차선으로 복귀
            if (!path_still_safe && driven_dist_ < locked_total_L_ * 0.2) {
                is_changing_lane_ = false;
                active_change_dir_ = LaneDir::CENTER;
                brain.cancel_lane_change();  
            }

            // 카메라가 새로운 차선에 완전히 넘어간 것을 감지하면 (swap_detected), 차선 변경 행위 종료 판정
            double current_ego_y = get_y_from_line(myCar.ego_line, 0.5);
            bool swap_detected = false;
            if (driven_dist_ > locked_total_L_ * 0.4) {
                if (active_change_dir_ == LaneDir::LEFT && current_ego_y > 0.10) swap_detected = true;
                else if (active_change_dir_ == LaneDir::RIGHT && current_ego_y < -0.10) swap_detected = true;
            }

            // 거리 끝까지 달렸거나, 카메라가 차선을 넘어갔다고 확인하면
            if (swap_detected || driven_dist_ >= locked_total_L_) {
                is_changing_lane_ = false;
                active_change_dir_ = LaneDir::CENTER;
                brain.complete_lane_change();  // 쿨타임 돌리면서 차선 유지 크루즈 모드로 복귀!
            }
        }
       
        // 차선 변경 중이 아니라면, 안전하게 내 차선(ego_line)을 따라가는 궤적 배정
        if (!is_changing_lane_) {
            for (double x = 0.0; x <= max_visible_x; x += 0.2) {
                geometry_msgs::msg::PoseStamped p;
                p.pose.position.x = x;
                p.pose.position.y = get_y_from_line(myCar.ego_line, x);
                final_path_poses.push_back(p);  
            }
        }
       
        double final_dodge_speed = decision.target_speed;

        // 최종 궤적에 속도(z)와 차량 헤딩 각도(yaw)를 입혀서 Path 메시지로 포장
        for (size_t i = 0; i < final_path_poses.size(); ++i) {
            final_path_poses[i].header = path_msg.header;
            if (decision.state == BehaviorState::EMERGENCY_BRAKE) {
                final_path_poses[i].pose.position.z = 0.0; // 풀브레이크
            } else {
                final_path_poses[i].pose.position.z = final_dodge_speed;
            }
           
            // 점과 점 사이의 각도를 계산해서 차량 앞머리가 부드럽게 돌아가도록 유도
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

        path_pub_->publish(path_msg); // 자율주행 컨트롤러로 명령 전송
       
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