#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

class LaneCandidateMaskNode : public rclcpp::Node {
public:
  LaneCandidateMaskNode() : Node("lane_candidate_mask_node") {
    // ==========================================================
    // ⭐ [파라미터 선언부] 모든 핵심 변수를 실시간 조절 가능
    // ==========================================================
    declare_parameter<int>("strict_white_v_min", 140); // 진짜 차선 기준 (엄격)
    declare_parameter<int>("loose_white_v_min", 110);  // 배경 덩어리 검출용 (넉넉하게)
    declare_parameter<int>("white_s_max", 60);  
    
    declare_parameter<int>("black_v_min", 0);  
    declare_parameter<int>("black_v_max", 100); 
    
    // 거대한 배경 덩어리 사냥꾼 파라미터 (Top-Hat)
    declare_parameter<int>("tophat_size", 15); // 얇은 선을 지우고 덩어리만 남기는 커널 크기
    declare_parameter<int>("blast_size", 17);  // 남은 덩어리를 뚱뚱하게 팽창시키는 커널 크기

    // ⭐ [NEW] 잔챙이 노이즈 사냥꾼 파라미터 (선생님 아이디어)
    // 5로 설정하면 5x5픽셀(약 1.7cm) 이하의 얇은 빛 가닥들은 모조리 지워집니다.
    declare_parameter<int>("noise_eraser_size", 5); 

    declare_parameter<int>("yellow_fat_radius", 15); 
    declare_parameter<int>("black_fat_radius", 10);   
    declare_parameter<int>("seed_row_from_bottom", 50);

    // ==========================================================
    // 토픽 Subscriber & Publisher 세팅
    // ==========================================================
    sub_ = image_transport::create_subscription(this, "/perception/bev/image",
        std::bind(&LaneCandidateMaskNode::cb, this, std::placeholders::_1), "raw");

    pub_lane_mask_ = image_transport::create_publisher(this, "/perception/lane/mask");
    pub_roi_ = image_transport::create_publisher(this, "/perception/debug/roi");
    pub_fat_black_ = image_transport::create_publisher(this, "/perception/debug/fat_black");
    pub_white_ = image_transport::create_publisher(this, "/perception/debug/white");
    pub_black_ = image_transport::create_publisher(this, "/perception/debug/black");
    pub_yellow_ = image_transport::create_publisher(this, "/perception/debug/yellow"); 
    pub_overlay_ = image_transport::create_publisher(this, "/perception/debug/overlay");
    
    // 디버깅용 110 그물망 & 배경 덩어리 송출
    pub_loose_white_ = image_transport::create_publisher(this, "/perception/debug/loose_white");
    pub_massive_white_ = image_transport::create_publisher(this, "/perception/debug/massive_white");

    pub_border_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("/perception/debug/bev_border", 10);
  }

private:
  void cb(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    cv::Mat bev = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (bev.empty()) return;

    int h = bev.rows;
    int w = bev.cols;

    cv::Mat hsv;
    cv::cvtColor(bev, hsv, cv::COLOR_BGR2HSV);

    // ==========================================================
    // 1. 색상 마스크 추출 (이중 필터링)
    // ==========================================================
    cv::Mat loose_white_mask, strict_white_mask, black_mask, yellow_mask;
    
    // ① 배경 덩어리를 넉넉하게 잡기 위한 '느슨한' 흰색 마스크
    cv::inRange(hsv, cv::Scalar(0, 0, get_parameter("loose_white_v_min").as_int()), 
                     cv::Scalar(180, get_parameter("white_s_max").as_int(), 255), loose_white_mask);
    
    // ② 진짜 차선만 뽑아내기 위한 '엄격한' 흰색 마스크
    cv::inRange(hsv, cv::Scalar(0, 0, get_parameter("strict_white_v_min").as_int()), 
                     cv::Scalar(180, get_parameter("white_s_max").as_int(), 255), strict_white_mask);
    
    cv::inRange(hsv, cv::Scalar(0, 0, get_parameter("black_v_min").as_int()), 
                     cv::Scalar(180, 255, get_parameter("black_v_max").as_int()), black_mask);
    
    cv::inRange(hsv, cv::Scalar(15, 80, 100), cv::Scalar(35, 255, 255), yellow_mask);

    // ==========================================================
    // 2. 이중 필터링 + 동적 커널 사이즈를 활용한 배경 원천 차단
    // ==========================================================
    int t_size = get_parameter("tophat_size").as_int();
    cv::Mat tophat_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(t_size, t_size));
    
    cv::Mat massive_white;
    // '느슨한' 흰색 마스크에서 얇은 선을 지우고 거대한 덩어리만 남김
    cv::morphologyEx(loose_white_mask, massive_white, cv::MORPH_OPEN, tophat_kernel);
    
    int b_size = get_parameter("blast_size").as_int();
    cv::Mat blast_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(b_size, b_size));
    
    // 포획한 덩어리를 지정된 크기만큼 빵빵하게 부풀림
    cv::dilate(massive_white, massive_white, blast_kernel);

    cv::Mat white_mask;
    // '엄격한' 흰색 마스크에서 뚱뚱해진 거대한 배경 덩어리를 완전히 빼버림
    cv::subtract(strict_white_mask, massive_white, white_mask);

    // ==========================================================
    // ⭐ [NEW] 잔챙이 노이즈 지우개 (선생님 아이디어 적용)
    // 거대한 배경을 도려낸 후, 여전히 남아있는 '차선보다 얇은' 빛 가닥들을 지웁니다.
    // ==========================================================
    int eraser_size = get_parameter("noise_eraser_size").as_int();
    if (eraser_size > 0) {
        // 커널 크기가 짝수면 에러가 날 수 있으므로, 홀수로 보정해주는 센스!
        if (eraser_size % 2 == 0) eraser_size += 1; 
        
        cv::Mat eraser_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(eraser_size, eraser_size));
        // 열기(Open) 연산: 작은 도장보다 얇은 것들은 모두 소멸시킵니다.
        cv::morphologyEx(white_mask, white_mask, cv::MORPH_OPEN, eraser_kernel);
    }
    // ==========================================================

    // ==========================================================
    // 3. 팽창(Fat) 방어벽 & 도화지 세팅
    // ==========================================================
    cv::Mat fat_yellow;
    int y_r = get_parameter("yellow_fat_radius").as_int();
    int y_size = y_r * 2 + 1; 
    cv::Mat yellow_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(y_size, y_size));
    cv::dilate(yellow_mask, fat_yellow, yellow_kernel);

    cv::Mat fat_black;
    int b_r = get_parameter("black_fat_radius").as_int();
    int b_size_black = b_r * 2 + 1;
    cv::Mat black_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(b_size_black, b_size_black));
    cv::dilate(black_mask, fat_black, black_kernel);

    cv::Mat expandable = fat_black.clone();
    expandable.setTo(0, fat_yellow); 
    cv::rectangle(expandable, cv::Point(0, 0), cv::Point(w - 1, h - 1), cv::Scalar(0), 5);

    cv::Mat bev_gray, bev_valid_mask;
    cv::cvtColor(bev, bev_gray, cv::COLOR_BGR2GRAY);
    cv::threshold(bev_gray, bev_valid_mask, 0, 255, cv::THRESH_BINARY); 
    cv::bitwise_and(expandable, bev_valid_mask, expandable);

    // ==========================================================
    // 4. BEV 5각형 테두리 RViz 마커 생성
    // ==========================================================
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bev_valid_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
        auto largest_contour = *std::max_element(contours.begin(), contours.end(),
            [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                return cv::contourArea(a) < cv::contourArea(b);
            });

        visualization_msgs::msg::Marker border_marker;
        border_marker.header.frame_id = "base_link";
        border_marker.header.stamp = msg->header.stamp;
        border_marker.ns = "bev_border";
        border_marker.id = 0;
        border_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        border_marker.action = visualization_msgs::msg::Marker::ADD;
        border_marker.scale.x = 0.05; 
        
        border_marker.color.r = 1.0f; border_marker.color.g = 0.5f; border_marker.color.b = 0.0f; border_marker.color.a = 1.0f;
        border_marker.lifetime = rclcpp::Duration::from_seconds(0.0);

        double bev_x_min = 0.12, bev_x_max = 2.0;
        double bev_y_min = -0.85, bev_y_max = 0.85;

        for (const auto& pt : largest_contour) {
            geometry_msgs::msg::Point p;
            p.x = bev_x_max - ((double)pt.y / h) * (bev_x_max - bev_x_min);
            p.y = bev_y_max - ((double)pt.x / w) * (bev_y_max - bev_y_min);
            p.z = 0.0;
            border_marker.points.push_back(p);
        }
        if (!largest_contour.empty()) {
            geometry_msgs::msg::Point p;
            p.x = bev_x_max - ((double)largest_contour[0].y / h) * (bev_x_max - bev_x_min);
            p.y = bev_y_max - ((double)largest_contour[0].x / w) * (bev_y_max - bev_y_min);
            p.z = 0.0;
            border_marker.points.push_back(p);
        }
        pub_border_marker_->publish(border_marker);
    }

    // ==========================================================
    // 5. 잉크 확산(FloodFill)을 통한 주행 영역(ROI) 추출 및 최종 필터링
    // ==========================================================
    cv::Mat roi = cv::Mat::zeros(h, w, CV_8UC1);
    int seed_x = w / 2;
    int seed_y = h - get_parameter("seed_row_from_bottom").as_int();

    bool found = false;
    for(int y = h - 1; y > h / 2; --y) {
        if(expandable.at<uchar>(y, seed_x) > 0) {
            seed_y = y;
            found = true; 
            break;
        }
    }

    if(found) {
        cv::Mat flood_mask = cv::Mat::zeros(h + 2, w + 2, CV_8UC1); 
        cv::floodFill(expandable, flood_mask, cv::Point(seed_x, seed_y), cv::Scalar(255),
                      0, cv::Scalar(0), cv::Scalar(0), 
                      8 | (255 << 8) | cv::FLOODFILL_MASK_ONLY);
        roi = flood_mask(cv::Rect(1, 1, w, h)).clone();
    }

    cv::Mat lane_final;
    // 완벽하게 정제된 white_mask와 잉크 확산 ROI를 겹침
    cv::bitwise_and(white_mask, roi, lane_final);

    // ==========================================================
    // 6. ROS 이미지 토픽으로 송출 (Publish)
    // ==========================================================
    auto h_msg = msg->header;
    pub_lane_mask_.publish(*cv_bridge::CvImage(h_msg, "mono8", lane_final).toImageMsg());
    pub_roi_.publish(*cv_bridge::CvImage(h_msg, "mono8", roi).toImageMsg());
    pub_white_.publish(*cv_bridge::CvImage(h_msg, "mono8", white_mask).toImageMsg());
    pub_black_.publish(*cv_bridge::CvImage(h_msg, "mono8", black_mask).toImageMsg());
    pub_fat_black_.publish(*cv_bridge::CvImage(h_msg, "mono8", expandable).toImageMsg());
    pub_yellow_.publish(*cv_bridge::CvImage(h_msg, "mono8", fat_yellow).toImageMsg());
    
    // 디버깅용 토픽 송출
    pub_loose_white_.publish(*cv_bridge::CvImage(h_msg, "mono8", loose_white_mask).toImageMsg());
    pub_massive_white_.publish(*cv_bridge::CvImage(h_msg, "mono8", massive_white).toImageMsg());

    // 종합 오버레이 이미지 그리기
    cv::Mat overlay = bev.clone();
    overlay.setTo(cv::Scalar(255, 0, 0), roi);          
    overlay.setTo(cv::Scalar(0, 0, 255), fat_yellow);   
    overlay.setTo(cv::Scalar(0, 255, 0), lane_final);   
    pub_overlay_.publish(*cv_bridge::CvImage(h_msg, "bgr8", overlay).toImageMsg());
  }

  // Publisher 및 Subscriber 변수 선언부
  image_transport::Subscriber sub_;
  image_transport::Publisher pub_lane_mask_, pub_roi_, pub_fat_black_, pub_white_, pub_black_, pub_yellow_, pub_overlay_;
  image_transport::Publisher pub_loose_white_, pub_massive_white_; 
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_border_marker_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneCandidateMaskNode>());
  rclcpp::shutdown();
  return 0;
}