#ifndef OBJ_TRACK_ROS__ROS2_CAMERAS
#define OBJ_TRACK_ROS__ROS2_CAMERAS

#include "rclcpp/rclcpp.hpp"
#include <m3t/tracker.h>
#include <m3t/generator.h>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/imgproc.hpp"
#include "tf2_ros/buffer.h"
#include "Eigen/Dense"

namespace obj_track_ros
{

  class Ros2ColorCamera : public m3t::ColorCamera
  {
  public:
    Ros2ColorCamera(rclcpp::Node *node,
                    const std::string &name,
                    const std::string &img_topic,
                    const std::string &info_topic,
                    const std::string &frame,
                    bool publish_overlay);

    bool image_ready = false;
    bool intrinsics_ready = false;
    bool publish_overlay = false;

    bool SetUp() override;

    bool UpdateImage(bool synchronized) override;

    void setImage(const sensor_msgs::msg::Image::SharedPtr msg);

    void setCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    bool is_ready();

    void publishOverlay(cv::Mat overlay);

    void updatePose(std::shared_ptr<tf2_ros::Buffer> buffer, std::string base_frame);

    std_msgs::msg::Header getHeader();

  private:
    std_msgs::msg::Header header;
    std::string frame;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub;
  };

  class Ros2DepthCamera : public m3t::DepthCamera
  {
  public:
    Ros2DepthCamera(rclcpp::Node *node,
                    const std::string &name,
                    const std::string &img_topic,
                    const std::string &info_topic,
                    const std::string &frame,
                    float depth_scale);

    bool image_ready = false;
    bool intrinsics_ready = false;

    bool SetUp() override;

    bool UpdateImage(bool synchronized) override;

    void setImage(const sensor_msgs::msg::Image::SharedPtr msg);

    void setCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    bool is_ready();

    void updatePose(std::shared_ptr<tf2_ros::Buffer> buffer, std::string base_frame);

    std_msgs::msg::Header getHeader();

  private:
    std::string frame;
    std_msgs::msg::Header header;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
  };
}

#endif