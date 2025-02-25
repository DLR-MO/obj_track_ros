#ifndef OBJ_TRACK_ROS_HPP
#define OBJ_TRACK_ROS_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "obj_track_ros/msg/tracked_object.hpp"
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

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
    std::shared_ptr<m3t::Tracker> getTracker();
    void waitForCameras();
  private:
    void configureCameras(const std::vector<std::string> & camera_configs);
    void receiveTrackedBody(const obj_track_ros::msg::TrackedObject::SharedPtr msg);
    std::shared_ptr<m3t::FullNormalRenderer> normal_renderer;
    std::vector<std::shared_ptr<Ros2ColorCamera>> color_cameras;
    std::vector<std::shared_ptr<Ros2DepthCamera>> depth_cameras;
    std::shared_ptr<m3t::Tracker> tracker;
    std::shared_ptr<m3t::RendererGeometry> geometry;
    std::vector<std::shared_ptr<m3t::FocusedBasicDepthRenderer>> color_renderers;
    std::vector<std::shared_ptr<m3t::FocusedBasicDepthRenderer>> depth_renderers;
    rclcpp::Subscription<obj_track_ros::msg::TrackedObject>::SharedPtr tracked_obj_sub;
};

}


#endif