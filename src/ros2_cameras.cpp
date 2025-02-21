#include "obj_track_ros/ros2_cameras.hpp"


namespace obj_track_ros
{

    Ros2ColorCamera::Ros2ColorCamera(const std::string & name) : ColorCamera(name) {}

    bool Ros2ColorCamera::SetUp()
    {
      set_up_ = true;
      return true;
    }
    
    bool Ros2ColorCamera::UpdateImage(bool synchronized)
    {
      return true;
    }

    void Ros2ColorCamera::setImage(cv::Mat img)
    {
      image_ = img;
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
