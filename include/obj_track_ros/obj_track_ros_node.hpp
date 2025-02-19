#ifndef OBJ_TRACK_ROS_HPP
#define OBJ_TRACK_ROS_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/imgproc.hpp"
#include "image_transport/image_transport.hpp"

#include <m3t/tracker.h>
#include <m3t/generator.h>

namespace obj_track_ros
{

class Ros2ColorCamera : public m3t::ColorCamera {
  public:
    Ros2ColorCamera(const std::string & name) : ColorCamera(name) {}
    bool image_ready = false;
    bool intrinsics_ready = false;

    bool SetUp() override
    {
      set_up_ = true;
      return true;
    }
    
    bool UpdateImage(bool synchronized) override
    {
      return true;
    }

    void setImage(cv::Mat img)
    {
      image_ = img;
      image_ready = true;
    }

    void setCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
      intrinsics_.fu = msg->k[0];
      intrinsics_.fv = msg->k[4];
      intrinsics_.ppu = msg->k[2];
      intrinsics_.ppv = msg->k[5];
      intrinsics_.width = msg->width;
      intrinsics_.height = msg->height;
      intrinsics_ready = true;

    }

    bool is_ready()
    {
      return image_ready && intrinsics_ready;
    }
};

class ObjTrackRosNode : 
  public rclcpp::Node, 
  public m3t::Publisher, 
  public m3t::Subscriber
{
  public:
    ObjTrackRosNode();
    bool UpdatePublisher(int iteration) override;
    bool UpdateSubscriber(int iteration) override;
    bool SetUp() override;
    std::shared_ptr<Ros2ColorCamera> camera_color;

  private:
    void timer_callback();
    void receive_image(const sensor_msgs::msg::Image::SharedPtr msg);
    void receive_cam_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub;
    size_t count_;

};

}


#endif