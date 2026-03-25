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
    // ⭐ [종합 컨트롤 타워] 모든 핵심 파라미터를 여기서 관리합니다!
    // ==========================================================
    
    // ----------------------------------------------------------
    // 1. 흰색 차선 (White) 파라미터
    // ----------------------------------------------------------
    declare_parameter<int>("strict_white_v_min", 105); 
    declare_parameter<int>("loose_white_v_min", 90);  
    declare_parameter<int>("white_s_max", 25);         

    // ----------------------------------------------------------
    // 2. 검은 도로 (Black) 파라미터
    // ----------------------------------------------------------
    declare_parameter<int>("black_v_min", 0);  
    declare_parameter<int>("black_v_max", 125);        

    // ----------------------------------------------------------
    // 3. 노란색 차선 (Yellow) 파라미터
    // ----------------------------------------------------------
    declare_parameter<int>("yellow_h_min", 20);  
    declare_parameter<int>("yellow_h_max", 35);  
    declare_parameter<int>("yellow_s_min", 70);  
    declare_parameter<int>("yellow_s_max", 180); 
    declare_parameter<int>("yellow_v_min", 145); 
    declare_parameter<int>("yellow_v_max", 255); 

    // ----------------------------------------------------------
    // 4. 노이즈 사냥꾼 & 형태학적(Morphology) 필터 사이즈
    // ----------------------------------------------------------
    declare_parameter<int>("tophat_size", 20);       
    declare_parameter<int>("blast_size", 25);        
    declare_parameter<int>("noise_eraser_size", 3);  
    
    // ----------------------------------------------------------
    // 5. 팽창(Fat) 방어벽 & 잉크 확산(FloodFill) 세팅
    // ----------------------------------------------------------
    declare_parameter<int>("yellow_fat_radius", 6); 
    declare_parameter<int>("black_fat_radius", 10);  
    declare_parameter<int>("seed_row_from_bottom", 50); 

    // ----------------------------------------------------------
    // 6. BEV(탑뷰) 물리적 거리 파라미터 (단위: 미터)
    // ----------------------------------------------------------
    declare_parameter<double>("bev_x_min", 0.12); 
    declare_parameter<double>("bev_x_max", 2.0);  
    declare_parameter<double>("bev_y_min", -0.85); 
    declare_parameter<double>("bev_y_max", 0.85);  

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
    
    pub_loose_white_ = image_transport::create_publisher(this, "/perception/debug/loose_white");
    pub_massive_white_ = image_transport::create_publisher(this, "/perception/debug/massive_white");
    pub_strict_white_ = image_transport::create_publisher(this, "/perception/debug/strict_white");

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
    
    cv::inRange(hsv, cv::Scalar(0, 0, get_parameter("loose_white_v_min").as_int()), 
                     cv::Scalar(180, get_parameter("white_s_max").as_int(), 255), loose_white_mask);
    
    cv::inRange(hsv, cv::Scalar(0, 0, get_parameter("strict_white_v_min").as_int()), 
                     cv::Scalar(180, get_parameter("white_s_max").as_int(), 255), strict_white_mask);
    
    cv::inRange(hsv, cv::Scalar(0, 0, get_parameter("black_v_min").as_int()), 
                     cv::Scalar(180, 255, get_parameter("black_v_max").as_int()), black_mask);
    
    cv::inRange(hsv, 
        cv::Scalar(get_parameter("yellow_h_min").as_int(), get_parameter("yellow_s_min").as_int(), get_parameter("yellow_v_min").as_int()), 
        cv::Scalar(get_parameter("yellow_h_max").as_int(), get_parameter("yellow_s_max").as_int(), get_parameter("yellow_v_max").as_int()), 
        yellow_mask);

    // ==========================================================
    // 2. 이중 필터링 + 동적 커널 사이즈를 활용한 배경 원천 차단
    // ==========================================================
    int t_size = get_parameter("tophat_size").as_int();
    cv::Mat tophat_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(t_size, t_size));
    
    cv::Mat massive_white;
    cv::morphologyEx(loose_white_mask, massive_white, cv::MORPH_OPEN, tophat_kernel);
    
    int b_size = get_parameter("blast_size").as_int();
    cv::Mat blast_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(b_size, b_size));
    cv::dilate(massive_white, massive_white, blast_kernel);

    cv::Mat white_mask;
    cv::subtract(strict_white_mask, massive_white, white_mask);

    int eraser_size = get_parameter("noise_eraser_size").as_int();
    if (eraser_size > 0) {
        if (eraser_size % 2 == 0) eraser_size += 1; 
        cv::Mat eraser_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(eraser_size, eraser_size));
        cv::morphologyEx(white_mask, white_mask, cv::MORPH_OPEN, eraser_kernel);
    }

    // ==========================================================
    // 3. 팽창(Fat) 방어벽 세팅
    // ==========================================================
    cv::Mat fat_yellow;
    int y_r = get_parameter("yellow_fat_radius").as_int();
    int y_size = y_r * 2 + 1; 
    cv::Mat yellow_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(y_size, y_size));
    cv::dilate(yellow_mask, fat_yellow, yellow_kernel);

    cv::Mat fat_black;
    int b_r = get_parameter("black_fat_radius").as_int();
    int b_size_black = b_r * 2 + 1;
    cv::Mat black_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(b_size_black, b_size_black));
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

        double bev_x_min = get_parameter("bev_x_min").as_double();
        double bev_x_max = get_parameter("bev_x_max").as_double();
        double bev_y_min = get_parameter("bev_y_min").as_double();
        double bev_y_max = get_parameter("bev_y_max").as_double();

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
    cv::bitwise_and(white_mask, roi, lane_final);

    // ⭐ [NEW] 차선 연결을 위한 최종 팽창 (선생님 아이디어 적용!)
    // 끊어진 차선을 이어주기 위해 3픽셀 두께로 살짝 뚱뚱하게 만듭니다.
    int final_fat_size = 5; 
    cv::Mat final_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(final_fat_size, final_fat_size));
    cv::dilate(lane_final, lane_final, final_kernel);

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
    
    pub_loose_white_.publish(*cv_bridge::CvImage(h_msg, "mono8", loose_white_mask).toImageMsg());
    pub_massive_white_.publish(*cv_bridge::CvImage(h_msg, "mono8", massive_white).toImageMsg());
    pub_strict_white_.publish(*cv_bridge::CvImage(h_msg, "mono8", strict_white_mask).toImageMsg());

    cv::Mat overlay = bev.clone();
    overlay.setTo(cv::Scalar(255, 0, 0), roi);          
    overlay.setTo(cv::Scalar(0, 0, 255), fat_yellow);   
    overlay.setTo(cv::Scalar(0, 255, 0), lane_final);   
    pub_overlay_.publish(*cv_bridge::CvImage(h_msg, "bgr8", overlay).toImageMsg());
  }

  image_transport::Subscriber sub_;
  image_transport::Publisher pub_lane_mask_, pub_roi_, pub_fat_black_, pub_white_, pub_black_, pub_yellow_, pub_overlay_;
  image_transport::Publisher pub_loose_white_, pub_massive_white_, pub_strict_white_; 
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_border_marker_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneCandidateMaskNode>());
  rclcpp::shutdown();
  return 0;
}
