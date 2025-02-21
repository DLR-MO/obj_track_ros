#ifndef OBJ_TRACK_ROS_HPP
#define OBJ_TRACK_ROS_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/imgproc.hpp"

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

  private:
    void receive_image(const sensor_msgs::msg::Image::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub;
    size_t count_;

};

}


#endif