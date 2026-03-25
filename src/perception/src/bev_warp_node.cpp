#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vector>

class BevWarpNode : public rclcpp::Node {
public:
    BevWarpNode() : Node("bev_warp_node") {
        // 1. 카메라 타입 파라미터 (Launch에서 전달받음)
        declare_parameter<std::string>("camera_type", "HW40");
        std::string cam_type = get_parameter("camera_type").as_string();
        
        // 2. 패키지 경로에서 YAML Config 파일 찾기
        std::string pkg_share = ament_index_cpp::get_package_share_directory("perception");
        std::string config_path = (cam_type == "HW40") ? 
            pkg_share + "/config/hw40_params.yaml" : pkg_share + "/config/usb2_params.yaml";
        
        YAML::Node cfg = YAML::LoadFile(config_path);

        // [YAML에서 파라미터 불러오기]
        double fx = cfg["intrinsics"]["fx"].as<double>();
        double fy = cfg["intrinsics"]["fy"].as<double>();
        double cx = cfg["intrinsics"]["cx"].as<double>();
        double cy = cfg["intrinsics"]["cy"].as<double>();

        double pitch_deg = cfg["extrinsics"]["pitch_deg"].as<double>();
        double h_m = cfg["extrinsics"]["height_m"].as<double>();
        double pitch_rad = pitch_deg * CV_PI / 180.0;

        double x_min = cfg["bev"]["x_min"].as<double>();
        double x_max = cfg["bev"]["x_max"].as<double>();
        double y_min = cfg["bev"]["y_min"].as<double>();
        double y_max = cfg["bev"]["y_max"].as<double>();
        int out_w = cfg["bev"]["output_width"].as<int>();
        int out_h = cfg["bev"]["output_height"].as<int>();

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
        
        RCLCPP_INFO(get_logger(), "BEV Warp Node Initialized with %s config.", cam_type.c_str());
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