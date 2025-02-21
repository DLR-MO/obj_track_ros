#include "obj_track_ros/ros2_cameras.hpp"

using std::placeholders::_1;

namespace obj_track_ros
{

  Ros2ColorCamera::Ros2ColorCamera(
      rclcpp::Node *node,
      const std::string &name,
      const std::string &img_topic,
      const std::string &info_topic) : ColorCamera(name)
  {
    img_sub = node->create_subscription<sensor_msgs::msg::Image>(img_topic, 10, std::bind(&Ros2ColorCamera::setImage, this, _1));
    info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>(info_topic, 10, std::bind(&Ros2ColorCamera::setCameraInfo, this, _1));
  }

  bool Ros2ColorCamera::SetUp()
  {
    set_up_ = true;
    return true;
  }

  bool Ros2ColorCamera::UpdateImage(bool synchronized)
  {
    return true;
  }

  void Ros2ColorCamera::setImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto cvim = cv_bridge::toCvCopy(msg, msg->encoding);
    cv::cvtColor(cvim->image, cvim->image, cv::COLOR_BGR2RGB);
    image_ = cvim->image;
    image_ready = true;
  }

  void Ros2ColorCamera::setCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    intrinsics_.fu = msg->k[0];
    intrinsics_.fv = msg->k[4];
    intrinsics_.ppu = msg->k[2];
    intrinsics_.ppv = msg->k[5];
    intrinsics_.width = msg->width;
    intrinsics_.height = msg->height;
    intrinsics_ready = true;
  }

  bool Ros2ColorCamera::is_ready()
  {
    return image_ready && intrinsics_ready;
  }
}
