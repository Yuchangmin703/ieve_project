#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class LaneCandidateMaskNode : public rclcpp::Node {
public:
  LaneCandidateMaskNode() : Node("lane_candidate_mask_node") {
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
    declare_parameter<bool>("extend_yellow_top", true);
    declare_parameter<int>("black_fat_radius", 10);
    declare_parameter<int>("seed_row_from_bottom", 25);

    // 파라미터 1회 캐싱 (매 프레임 get_parameter() mutex 제거)
    cache_params();

    // 처리 해상도 고정 (320x240)
    proc_w_ = 320;
    proc_h_ = 240;
    constexpr float scale = 0.5f;

    // 모폴로지 커널 사전 생성 (매 프레임 getStructuringElement 제거)
    int es = std::max(1, (int)(eraser_size_ * scale));
    if (es % 2 == 0) es += 1;
    eraser_size_scaled_ = es;

    int sds = std::max(1, (int)(sobel_dilate_size_ * scale));
    if (sds % 2 == 0) sds += 1;

    int yx = std::max(1, (int)(yellow_fat_x_ * scale));
    int yy = std::max(1, (int)(yellow_fat_y_ * scale));
    int br = std::max(1, (int)(black_fat_radius_ * scale));

    int ts = std::max(1, (int)(tophat_size_ * scale));
    int bs = std::max(1, (int)(blast_size_ * scale));

    seed_row_scaled_ = std::max(1, (int)(seed_row_from_bottom_ * scale));

    int ffs = std::max(1, (int)(5 * scale));
    if (ffs % 2 == 0) ffs += 1;

    yellow_kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(yx * 2 + 1, yy * 2 + 1));
    black_kernel_  = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(br * 2 + 1, br * 2 + 1));
    eraser_kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(es, es));
    sobel_fat_kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(sds, sds));
    tophat_kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ts, ts));
    blast_kernel_  = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(bs, bs));
    final_kernel_  = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ffs, ffs));

    // HSV 범위 Scalar 사전 생성
    loose_white_lo_ = cv::Scalar(0, 0, loose_white_v_min_);
    loose_white_hi_ = cv::Scalar(180, white_s_max_, 255);
    strict_white_lo_ = cv::Scalar(0, 0, strict_white_v_min_);
    strict_white_hi_ = cv::Scalar(180, white_s_max_, 255);
    black_lo_ = cv::Scalar(0, 0, black_v_min_);
    black_hi_ = cv::Scalar(180, 255, black_v_max_);
    yellow_lo_ = cv::Scalar(yellow_h_min_, yellow_s_min_, yellow_v_min_);
    yellow_hi_ = cv::Scalar(yellow_h_max_, yellow_s_max_, yellow_v_max_);

    sub_ = image_transport::create_subscription(this, "/perception/bev/image",
        std::bind(&LaneCandidateMaskNode::cb, this, std::placeholders::_1), "raw");
    pub_lane_mask_ = image_transport::create_publisher(this, "/perception/lane/mask");
  }

private:
  void cache_params() {
    strict_white_v_min_ = get_parameter("strict_white_v_min").as_int();
    loose_white_v_min_ = get_parameter("loose_white_v_min").as_int();
    white_s_max_ = get_parameter("white_s_max").as_int();
    sobel_thresh_ = get_parameter("sobel_thresh").as_int();
    sobel_dilate_size_ = get_parameter("sobel_dilate_size").as_int();
    black_v_min_ = get_parameter("black_v_min").as_int();
    black_v_max_ = get_parameter("black_v_max").as_int();
    yellow_h_min_ = get_parameter("yellow_h_min").as_int();
    yellow_h_max_ = get_parameter("yellow_h_max").as_int();
    yellow_s_min_ = get_parameter("yellow_s_min").as_int();
    yellow_s_max_ = get_parameter("yellow_s_max").as_int();
    yellow_v_min_ = get_parameter("yellow_v_min").as_int();
    yellow_v_max_ = get_parameter("yellow_v_max").as_int();
    tophat_size_ = get_parameter("tophat_size").as_int();
    blast_size_ = get_parameter("blast_size").as_int();
    eraser_size_ = get_parameter("noise_eraser_size").as_int();
    yellow_fat_x_ = get_parameter("yellow_fat_x").as_int();
    yellow_fat_y_ = get_parameter("yellow_fat_y").as_int();
    extend_yellow_top_ = get_parameter("extend_yellow_top").as_bool();
    black_fat_radius_ = get_parameter("black_fat_radius").as_int();
    seed_row_from_bottom_ = get_parameter("seed_row_from_bottom").as_int();
  }

  void cb(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    cv::Mat bev_original = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (bev_original.empty()) return;

    cv::Mat bev;
    cv::resize(bev_original, bev, cv::Size(proc_w_, proc_h_), 0, 0, cv::INTER_LINEAR);

    cv::Mat hsv, loose_white_mask, strict_white_mask, black_mask, yellow_mask;
    cv::cvtColor(bev, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv, loose_white_lo_, loose_white_hi_, loose_white_mask);
    cv::inRange(hsv, strict_white_lo_, strict_white_hi_, strict_white_mask);
    cv::inRange(hsv, black_lo_, black_hi_, black_mask);
    cv::inRange(hsv, yellow_lo_, yellow_hi_, yellow_mask);

    if (extend_yellow_top_) {
        int top_y = -1, top_x = -1;
        for (int y = 0; y < proc_h_; ++y) {
            const uchar* row = yellow_mask.ptr<uchar>(y);
            for (int x = 0; x < proc_w_; ++x) {
                if (row[x] > 0) { top_y = y; top_x = x; break; }
            }
            if (top_y != -1) break;
        }
        if (top_y > 0) {
            cv::line(yellow_mask, cv::Point(top_x, top_y), cv::Point(top_x, 0), cv::Scalar(255), 1);
        }
    }

    cv::Mat fat_yellow, fat_black;
    cv::dilate(yellow_mask, fat_yellow, yellow_kernel_);
    cv::dilate(black_mask, fat_black, black_kernel_);

    cv::Mat expandable = fat_black.clone();
    expandable.setTo(0, fat_yellow);
    cv::rectangle(expandable, cv::Point(0, 0), cv::Point(proc_w_ - 1, proc_h_ - 1), cv::Scalar(0), 5);

    cv::Mat bev_gray, bev_valid_mask;
    cv::cvtColor(bev, bev_gray, cv::COLOR_BGR2GRAY);
    cv::threshold(bev_gray, bev_valid_mask, 0, 255, cv::THRESH_BINARY);
    cv::bitwise_and(expandable, bev_valid_mask, expandable);

    cv::Mat roi = cv::Mat::zeros(proc_h_, proc_w_, CV_8UC1);
    int seed_x = proc_w_ / 2;
    int seed_y = proc_h_ - seed_row_scaled_;

    bool found = false;
    for (int y = proc_h_ - 1; y > proc_h_ / 2; --y) {
        if (expandable.at<uchar>(y, seed_x) > 0) {
            seed_y = y; found = true; break;
        }
    }

    if (found) {
        cv::Mat flood_mask = cv::Mat::zeros(proc_h_ + 2, proc_w_ + 2, CV_8UC1);
        cv::floodFill(expandable, flood_mask, cv::Point(seed_x, seed_y), cv::Scalar(255),
                      0, cv::Scalar(0), cv::Scalar(0), 8 | (255 << 8) | cv::FLOODFILL_MASK_ONLY);
        roi = flood_mask(cv::Rect(1, 1, proc_w_, proc_h_)).clone();
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
    cv::threshold(sobel_combined, sobel_raw_mask, sobel_thresh_, 255, cv::THRESH_BINARY);
    cv::bitwise_and(sobel_raw_mask, roi, sobel_raw_mask);

    cv::Mat sobel_mask;
    cv::dilate(sobel_raw_mask, sobel_mask, sobel_fat_kernel_);

    cv::morphologyEx(loose_white_mask, loose_white_mask, cv::MORPH_OPEN, eraser_kernel_);
    cv::morphologyEx(strict_white_mask, strict_white_mask, cv::MORPH_OPEN, eraser_kernel_);
    cv::morphologyEx(sobel_mask, sobel_mask, cv::MORPH_OPEN, eraser_kernel_);

    cv::Mat massive_white;
    cv::morphologyEx(loose_white_mask, massive_white, cv::MORPH_OPEN, tophat_kernel_);
    cv::dilate(massive_white, massive_white, blast_kernel_);

    cv::Mat temp_mask, white_mask, lane_final;
    cv::subtract(strict_white_mask, massive_white, temp_mask);
    cv::bitwise_and(temp_mask, sobel_mask, white_mask);

    if (eraser_size_scaled_ > 0) {
        cv::morphologyEx(white_mask, white_mask, cv::MORPH_OPEN, eraser_kernel_);
    }

    cv::dilate(white_mask, lane_final, final_kernel_);

    pub_lane_mask_.publish(*cv_bridge::CvImage(msg->header, "mono8", lane_final).toImageMsg());
  }

  // 캐싱된 파라미터
  int strict_white_v_min_, loose_white_v_min_, white_s_max_;
  int sobel_thresh_, sobel_dilate_size_;
  int black_v_min_, black_v_max_;
  int yellow_h_min_, yellow_h_max_, yellow_s_min_, yellow_s_max_, yellow_v_min_, yellow_v_max_;
  int tophat_size_, blast_size_, eraser_size_;
  int yellow_fat_x_, yellow_fat_y_;
  bool extend_yellow_top_;
  int black_fat_radius_, seed_row_from_bottom_;

  // 사전 계산된 값
  int proc_w_, proc_h_;
  int eraser_size_scaled_, seed_row_scaled_;

  // 사전 생성된 커널 (프레임당 재생성 제거)
  cv::Mat yellow_kernel_, black_kernel_, eraser_kernel_;
  cv::Mat sobel_fat_kernel_, tophat_kernel_, blast_kernel_, final_kernel_;

  // 사전 생성된 HSV Scalar
  cv::Scalar loose_white_lo_, loose_white_hi_;
  cv::Scalar strict_white_lo_, strict_white_hi_;
  cv::Scalar black_lo_, black_hi_;
  cv::Scalar yellow_lo_, yellow_hi_;

  image_transport::Subscriber sub_;
  image_transport::Publisher pub_lane_mask_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneCandidateMaskNode>());
  rclcpp::shutdown();
  return 0;
}
