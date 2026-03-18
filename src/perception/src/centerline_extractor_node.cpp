#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

#include "perception/msg/lane.hpp"
#include "perception/msg/lanes.hpp"

#include <algorithm>
#include <vector>
#include <string>

class CenterlineExtractorNode : public rclcpp::Node {
public:

  CenterlineExtractorNode() : Node("centerline_extractor_node")
  {

    declare_parameter<int>("width",480);
    declare_parameter<int>("height",640);
    declare_parameter<double>("meters_per_pixel",0.005);
    declare_parameter<double>("x_offset_m",0.15);
    declare_parameter<std::string>("frame_id","base_link");

    width_ = get_parameter("width").as_int();
    height_ = get_parameter("height").as_int();

    sub_ = image_transport::create_subscription(
      this,
      "/perception/lane/mask",
      std::bind(&CenterlineExtractorNode::cb,this,std::placeholders::_1),
      "raw");

    pub_debug_img_ =
      image_transport::create_publisher(
        this,
        "/perception/lane/centerline_debug");

    pub_lanes_msg_ =
      create_publisher<perception::msg::Lanes>(
        "/perception/lane/lanes",10);

    // ⭐ 추가된 부분: RViz 3D 출력을 위한 PointCloud2 퍼블리셔
    pub_pc2_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/lane/pointcloud", 10);

    RCLCPP_INFO(get_logger(),"Centerline extractor ready");
  }

private:

  struct LaneInternal
  {
    std::vector<geometry_msgs::msg::Point> pts;
  };

  void cb(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {

    cv::Mat mask =
      cv_bridge::toCvShare(msg,"mono8")->image;

    if(mask.empty())
      return;

    if(mask.cols != width_ || mask.rows != height_)
      cv::resize(mask,mask,cv::Size(width_,height_));

    // ------------------------------------------------
    // 1️⃣ Skeleton extraction
    // ------------------------------------------------

    cv::Mat skeleton;
    cv::ximgproc::thinning(mask,skeleton,cv::ximgproc::THINNING_ZHANGSUEN);

    cv::Mat debug;
    cv::cvtColor(skeleton,debug,cv::COLOR_GRAY2BGR);

    // ------------------------------------------------
    // parameters
    // ------------------------------------------------

    double mpp = get_parameter("meters_per_pixel").as_double();
    double x_offset = get_parameter("x_offset_m").as_double();
    std::string frame_id = get_parameter("frame_id").as_string();

    int mid = width_/2;

    const double Y_THRESH = 0.25;
    const double X_THRESH = 0.6;

    std::vector<LaneInternal> lanes;

    // ------------------------------------------------
    // 2️⃣ skeleton pixel scanning
    // ------------------------------------------------

    for(int v = height_-1; v >=0; v--)
    {

      const uchar* row = skeleton.ptr<uchar>(v);

      for(int u = 0; u < width_; u++)
      {

        if(row[u]==0)
          continue;

        // bev_warp_node.cpp에 설정된 실제 물리 영역과 동일하게 세팅
        double bev_x_min = 0.12;
        double bev_x_max = 2.0;
        double bev_y_min = -0.85;
        double bev_y_max = 0.85;

        // 실제 카메라 위치(렌즈)가 앞범퍼(base_link)보다 얼마나 뒤에 있는지 물리적 보정 (예: 카메라가 범퍼 뒤 10cm에 있다면 0.1)
        double camera_physical_offset = 0.0; // 차량 세팅에 맞게 조절하세요!

        // BEV 역변환 공식 적용 (진짜 물리적 위치)
        double x = bev_x_max - ((double)v / height_) * (bev_x_max - bev_x_min) + camera_physical_offset;
        double y = bev_y_max - ((double)u / width_) * (bev_y_max - bev_y_min);
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = 0;

        cv::circle(debug,cv::Point(u,v),2,cv::Scalar(0,0,255),-1);

        bool assigned=false;

        for(auto& lane : lanes)
        {

          if(lane.pts.empty())
            continue;

          auto& last = lane.pts.back();

          if(std::abs(p.y-last.y) < Y_THRESH &&
             std::abs(p.x-last.x) < X_THRESH)
          {

            lane.pts.push_back(p);
            assigned=true;
            break;
          }
        }

        if(!assigned)
        {
          LaneInternal new_lane;
          new_lane.pts.push_back(p);
          lanes.push_back(new_lane);
        }
      }
    }

    // ------------------------------------------------
    // 3️⃣ remove short lanes (noise)
    // ------------------------------------------------

    lanes.erase(
      std::remove_if(
        lanes.begin(),
        lanes.end(),
        [](const LaneInternal& l)
        {
          return l.pts.size() < 30;
        }),
      lanes.end());

    // ------------------------------------------------
    // 4️⃣ sort left → right
    // ------------------------------------------------

    std::sort(
      lanes.begin(),
      lanes.end(),
      [](const LaneInternal& a,const LaneInternal& b)
      {

        double sumA=0;
        double sumB=0;

        for(auto& p : a.pts) sumA+=p.y;
        for(auto& p : b.pts) sumB+=p.y;

        double meanA = sumA/a.pts.size();
        double meanB = sumB/b.pts.size();

        return meanA > meanB;
      });

    // ------------------------------------------------
    // 5️⃣ publish lanes message
    // ------------------------------------------------

    perception::msg::Lanes lanes_msg;
    lanes_msg.header = msg->header;

    for(auto& lane : lanes)
    {
      perception::msg::Lane lane_msg;
      lane_msg.points = lane.pts;
      lanes_msg.lanes.push_back(lane_msg);
    }

    pub_lanes_msg_->publish(lanes_msg);

    // ------------------------------------------------
    // ⭐ 6️⃣ publish PointCloud2 for RViz
    // ------------------------------------------------
    sensor_msgs::msg::PointCloud2 pc2_msg;
    pc2_msg.header.stamp = msg->header.stamp;
    pc2_msg.header.frame_id = frame_id; // "base_link" 기준

    // PointCloud2 구조체 설정 (x, y, z 공간)
    sensor_msgs::PointCloud2Modifier modifier(pc2_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    // 점 개수 계산 후 메모리 할당
    size_t total_points = 0;
    for(const auto& lane : lanes) total_points += lane.pts.size();
    modifier.resize(total_points);

    // x, y, z 좌표 채워넣기
    sensor_msgs::PointCloud2Iterator<float> iter_x(pc2_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pc2_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pc2_msg, "z");

    for(const auto& lane : lanes) {
        for(const auto& p : lane.pts) {
            *iter_x = p.x;
            *iter_y = p.y;
            *iter_z = 0.0;
            ++iter_x; ++iter_y; ++iter_z;
        }
    }

    // 최종 PointCloud2 메시지 발행
    pub_pc2_->publish(pc2_msg);

    // ------------------------------------------------
    // debug image
    // ------------------------------------------------

    auto out =
      cv_bridge::CvImage(
        msg->header,
        "bgr8",
        debug).toImageMsg();

    pub_debug_img_.publish(*out);
  }

  int width_;
  int height_;

  image_transport::Subscriber sub_;
  image_transport::Publisher pub_debug_img_;

  rclcpp::Publisher<perception::msg::Lanes>::SharedPtr pub_lanes_msg_;
  
  // ⭐ 추가된 부분: PointCloud2 퍼블리셔 멤버 변수
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc2_;
};

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<CenterlineExtractorNode>());
  rclcpp::shutdown();
  return 0;
}