#ifndef OBJ_TRACK_ROS__ROS2_CAMERAS
#define OBJ_TRACK_ROS__ROS2_CAMERAS

#include "rclcpp/rclcpp.hpp"
#include <m3t/tracker.h>
#include <m3t/generator.h>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace obj_track_ros
{

class Ros2ColorCamera : public m3t::ColorCamera {
  public:
    Ros2ColorCamera(const std::string & name);
    bool image_ready = false;
    bool intrinsics_ready = false;

    bool SetUp() override;
    
    bool UpdateImage(bool synchronized) override;

    void setImage(cv::Mat img);

    void setCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    bool is_ready();
};
}

#endif