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

#include <m3t/tracker.h>
#include <m3t/generator.h>

namespace obj_track_ros
{

class Ros2ColorCamera : public m3t::ColorCamera {
  public:
    Ros2ColorCamera(const std::string & name) : ColorCamera(name) {}
    bool is_ready = false;

    bool SetUp() override
    {
      set_up_ = true;
      return true;
    }
    
    bool UpdateImage(bool synchronized) override
    {
      std::cout << "update image" << std::endl; 
      return true;
    }

    void setImage(cv::Mat img)
    {
      image_ = img;
      is_ready = true;
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
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
    size_t count_;

};

}


#endif