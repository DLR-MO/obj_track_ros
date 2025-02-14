#ifndef OBJ_TRACK_ROS_HPP
#define OBJ_TRACK_ROS_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <m3t/tracker.h>

namespace obj_track_ros
{

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

  private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

}


#endif