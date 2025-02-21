#ifndef OBJ_TRACK_ROS_HPP
#define OBJ_TRACK_ROS_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include <m3t/tracker.h>
#include <m3t/generator.h>

#include "obj_track_ros/ros2_cameras.hpp"

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
    std::shared_ptr<Ros2ColorCamera> camera_color;
    std::shared_ptr<Ros2DepthCamera> camera_depth;
};

}


#endif