#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>

class BevWarpNode : public rclcpp::Node {
public:
    BevWarpNode() : Node("bev_warp_node") {
        double fx = 499.74665, fy = 712.17893, cx = 325.37045, cy = 246.26333;
        double h_m = 0.2735, pitch_deg = 23.6602;
        double pitch_rad = pitch_deg * CV_PI / 180.0;

        // 가시 범위 설정 (화면 꽉 채우기 및 왜곡 최소화)
        double x_min = 0.12, x_max = 2.0; 
        double y_min = -0.85, y_max = 0.85; 
        int out_w = 480, out_h = 640;

        std::vector<cv::Point2f> src_pts, dst_pts;
        auto world_to_bev_pixel = [&](double wx, double wy) {
            float px = (float)((wy - y_min) / (y_max - y_min) * out_w);
            float py = (float)((x_max - wx) / (x_max - x_min) * out_h);
            return cv::Point2f(px, py);
        };

        std::vector<cv::Point2d> ground_corners = {{x_max, y_min}, {x_max, y_max}, {x_min, y_min}, {x_min, y_max}};
        for (const auto& pt : ground_corners) {
            double cam_x = pt.y;
            double cam_y = -pt.x * sin(pitch_rad) + h_m * cos(pitch_rad);
            double cam_z =  pt.x * cos(pitch_rad) + h_m * sin(pitch_rad);
            float u = (float)((fx * cam_x / cam_z) + cx);
            float v = (float)((fy * cam_y / cam_z) + cy);
            src_pts.push_back(cv::Point2f(u, v));
            dst_pts.push_back(world_to_bev_pixel(pt.x, pt.y));
        }

        M_ = cv::getPerspectiveTransform(src_pts, dst_pts);
        out_size_ = cv::Size(out_w, out_h);

        sub_ = image_transport::create_subscription(this, "/perception/camera/undistorted", 
            std::bind(&BevWarpNode::cb, this, std::placeholders::_1), "raw");
        pub_ = image_transport::create_publisher(this, "/perception/bev/image");
    }

private:
    void cb(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        if (frame.empty()) return;
        cv::Mat bev_image;
        cv::warpPerspective(frame, bev_image, M_, out_size_, cv::INTER_LINEAR);
        pub_.publish(*cv_bridge::CvImage(msg->header, "bgr8", bev_image).toImageMsg());
    }
    cv::Mat M_; cv::Size out_size_;
    image_transport::Subscriber sub_; image_transport::Publisher pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BevWarpNode>());
    rclcpp::shutdown();
    return 0;
}