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
    // ⭐ 파라미터 세팅
    // ==========================================================
    declare_parameter<int>("strict_white_v_min", 90);  
    declare_parameter<int>("loose_white_v_min", 75);   
    declare_parameter<int>("white_s_max", 40);         

    declare_parameter<int>("sobel_thresh", 220); 
    declare_parameter<int>("sobel_dilate_size", 11); 

    declare_parameter<int>("black_v_min", 0);  
    declare_parameter<int>("black_v_max", 130);        

    declare_parameter<int>("yellow_h_min", 20);  
    declare_parameter<int>("yellow_h_max", 35);  
    declare_parameter<int>("yellow_s_min", 70);  
    declare_parameter<int>("yellow_s_max", 180); 
    declare_parameter<int>("yellow_v_min", 130); 
    declare_parameter<int>("yellow_v_max", 255); 

    declare_parameter<int>("tophat_size", 27);       
    declare_parameter<int>("blast_size", 30);        
    declare_parameter<int>("noise_eraser_size", 5);  
    
    declare_parameter<int>("yellow_fat_x", 8); 
    declare_parameter<int>("yellow_fat_y", 30); 
    
    // ⭐ [NEW] 노란 차선 천장 연장 파라미터 등록
    declare_parameter<bool>("extend_yellow_top", true); 
    
    declare_parameter<int>("black_fat_radius", 10);  
    declare_parameter<int>("seed_row_from_bottom", 25); 

    declare_parameter<double>("bev_x_min", 0.12); 
    declare_parameter<double>("bev_x_max", 2.0);  
    declare_parameter<double>("bev_y_min", -0.85); 
    declare_parameter<double>("bev_y_max", 0.85);  

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
    
    pub_sobel_raw_ = image_transport::create_publisher(this, "/perception/debug/sobel_raw");
    pub_sobel_fat_ = image_transport::create_publisher(this, "/perception/debug/sobel_fat");

    pub_border_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("/perception/debug/bev_border", 10);
  }

private:
  void cb(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    cv::Mat bev_original = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (bev_original.empty()) return;

    cv::Mat bev;
    cv::resize(bev_original, bev, cv::Size(320, 240), 0, 0, cv::INTER_LINEAR);

    int h = bev.rows; 
    int w = bev.cols; 

    float scale = 0.5f;
    int eraser_size = std::max(1, (int)(get_parameter("noise_eraser_size").as_int() * scale));
    if (eraser_size % 2 == 0) eraser_size += 1; 
    
    int t_size = std::max(1, (int)(get_parameter("tophat_size").as_int() * scale));
    int b_size = std::max(1, (int)(get_parameter("blast_size").as_int() * scale));
    
    int y_x = std::max(1, (int)(get_parameter("yellow_fat_x").as_int() * scale));
    int y_y = std::max(1, (int)(get_parameter("yellow_fat_y").as_int() * scale));
    
    int b_r = std::max(1, (int)(get_parameter("black_fat_radius").as_int() * scale));
    
    int sd_size = std::max(1, (int)(get_parameter("sobel_dilate_size").as_int() * scale));
    if (sd_size % 2 == 0) sd_size += 1; 
    
    int seed_row = std::max(1, (int)(get_parameter("seed_row_from_bottom").as_int() * scale));

    cv::Mat hsv, loose_white_mask, strict_white_mask, black_mask, yellow_mask;
    cv::cvtColor(bev, hsv, cv::COLOR_BGR2HSV);
    
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
    // ⭐ [NEW] 노란색 차선 최상단 인식 및 천장 연장 방어벽 로직
    // ==========================================================
    bool extend_yellow_top = get_parameter("extend_yellow_top").as_bool();
    if (extend_yellow_top) {
        int top_y = -1;
        int top_x = -1;
        
        // y=0(화면 맨 위)부터 y=h(맨 아래)까지 스캔하며 가장 먼저 등장하는 노란색 점 탐색
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                if (yellow_mask.at<uchar>(y, x) > 0) {
                    top_y = y;
                    top_x = x;
                    break;
                }
            }
            if (top_y != -1) break; // 찾았으면 루프 즉시 탈출
        }
        
        // 발견된 점이 있다면, 그 좌표부터 화면 맨 위(y=0)까지 하얀색(255) 선을 쫙 그어줌!
        if (top_y != -1 && top_y > 0) {
            // 이 얇은 선은 바로 아래의 dilate를 만나서 거대한 기둥으로 부풀려집니다.
            cv::line(yellow_mask, cv::Point(top_x, top_y), cv::Point(top_x, 0), cv::Scalar(255), 1);
        }
    }

    cv::Mat fat_yellow, fat_black;
    cv::Mat yellow_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(y_x * 2 + 1, y_y * 2 + 1));
    cv::dilate(yellow_mask, fat_yellow, yellow_kernel);

    cv::Mat black_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(b_r * 2 + 1, b_r * 2 + 1));
    cv::dilate(black_mask, fat_black, black_kernel);

    cv::Mat expandable = fat_black.clone(); 
    expandable.setTo(0, fat_yellow);        
    cv::rectangle(expandable, cv::Point(0, 0), cv::Point(w - 1, h - 1), cv::Scalar(0), 5); 

    cv::Mat bev_gray, bev_valid_mask;
    cv::cvtColor(bev, bev_gray, cv::COLOR_BGR2GRAY);
    cv::threshold(bev_gray, bev_valid_mask, 0, 255, cv::THRESH_BINARY); 
    cv::bitwise_and(expandable, bev_valid_mask, expandable);

    cv::Mat roi = cv::Mat::zeros(h, w, CV_8UC1);
    int seed_x = w / 2;
    int seed_y = h - seed_row; 

    bool found = false;
    for(int y = h - 1; y > h / 2; --y) {
        if(expandable.at<uchar>(y, seed_x) > 0) {
            seed_y = y; found = true; break;
        }
    }

    if(found) {
        cv::Mat flood_mask = cv::Mat::zeros(h + 2, w + 2, CV_8UC1); 
        cv::floodFill(expandable, flood_mask, cv::Point(seed_x, seed_y), cv::Scalar(255),
                      0, cv::Scalar(0), cv::Scalar(0), 8 | (255 << 8) | cv::FLOODFILL_MASK_ONLY);
        roi = flood_mask(cv::Rect(1, 1, w, h)).clone();
    }

    cv::bitwise_and(loose_white_mask, roi, loose_white_mask);
    cv::bitwise_and(strict_white_mask, roi, strict_white_mask);

    cv::Mat grad_x, grad_y, abs_grad_x, abs_grad_y, sobel_combined;
    cv::Sobel(bev_gray, grad_x, CV_16S, 1, 0, 3);
    cv::convertScaleAbs(grad_x, abs_grad_x);
    cv::Sobel(bev_gray, grad_y, CV_16S, 0, 1, 3);
    cv::convertScaleAbs(grad_y, abs_grad_y);
    cv::addWeighted(abs_grad_x, 1.0, abs_grad_y, 1.0, 0, sobel_combined);

    cv::Mat sobel_raw_mask;
    int s_thresh = get_parameter("sobel_thresh").as_int(); 
    cv::threshold(sobel_combined, sobel_raw_mask, s_thresh, 255, cv::THRESH_BINARY);
    
    cv::bitwise_and(sobel_raw_mask, roi, sobel_raw_mask); 

    cv::Mat sobel_mask;
    cv::Mat fat_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(sd_size, sd_size));
    cv::dilate(sobel_raw_mask, sobel_mask, fat_kernel);

    cv::Mat eraser_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(eraser_size, eraser_size));
    cv::morphologyEx(loose_white_mask, loose_white_mask, cv::MORPH_OPEN, eraser_kernel);
    cv::morphologyEx(strict_white_mask, strict_white_mask, cv::MORPH_OPEN, eraser_kernel);
    cv::morphologyEx(sobel_mask, sobel_mask, cv::MORPH_OPEN, eraser_kernel); 

    cv::Mat tophat_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(t_size, t_size));
    cv::Mat massive_white;
    cv::morphologyEx(loose_white_mask, massive_white, cv::MORPH_OPEN, tophat_kernel);
    
    cv::Mat blast_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(b_size, b_size));
    cv::dilate(massive_white, massive_white, blast_kernel);

    cv::Mat temp_mask, white_mask, lane_final;
    cv::subtract(strict_white_mask, massive_white, temp_mask);
    cv::bitwise_and(temp_mask, sobel_mask, white_mask);
    
    if (eraser_size > 0) {
        cv::morphologyEx(white_mask, white_mask, cv::MORPH_OPEN, eraser_kernel);
    }

    int final_fat_size = std::max(1, (int)(5 * scale));
    if (final_fat_size % 2 == 0) final_fat_size += 1;
    cv::Mat final_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(final_fat_size, final_fat_size));
    cv::dilate(white_mask, lane_final, final_kernel); 

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
    
    pub_sobel_raw_.publish(*cv_bridge::CvImage(h_msg, "mono8", sobel_raw_mask).toImageMsg());
    pub_sobel_fat_.publish(*cv_bridge::CvImage(h_msg, "mono8", sobel_mask).toImageMsg());

    cv::Mat overlay = bev.clone();
    overlay.setTo(cv::Scalar(255, 0, 0), roi);          
    overlay.setTo(cv::Scalar(0, 0, 255), fat_yellow);   
    overlay.setTo(cv::Scalar(0, 255, 0), lane_final);   
    pub_overlay_.publish(*cv_bridge::CvImage(h_msg, "bgr8", overlay).toImageMsg());
  }

  image_transport::Subscriber sub_;
  image_transport::Publisher pub_lane_mask_, pub_roi_, pub_fat_black_, pub_white_, pub_black_, pub_yellow_, pub_overlay_;
  image_transport::Publisher pub_loose_white_, pub_massive_white_, pub_strict_white_; 
  image_transport::Publisher pub_sobel_raw_, pub_sobel_fat_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_border_marker_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneCandidateMaskNode>());
  rclcpp::shutdown();
  return 0;
}