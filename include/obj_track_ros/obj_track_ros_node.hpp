// SPDX-FileCopyrightText: 2025 Fabian Wieczorek, German Aerospace Center (DLR)
// SPDX-License-Identifier: MIT

#ifndef OBJ_TRACK_ROS_HPP
#define OBJ_TRACK_ROS_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "obj_track_ros/msg/tracked_object.hpp"
#include "obj_track_ros/msg/tracker_control.hpp"
#include "obj_track_ros/msg/track_result.hpp"
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"

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
    std::shared_ptr<m3t::Tracker> getTracker();
    void waitForCameras();
  private:
    void configureCameras(const std::vector<std::string> & camera_configs);
    void receiveTrackedBody(const obj_track_ros::msg::TrackedObject::SharedPtr msg);
    void receiveTrackerControl(const obj_track_ros::msg::TrackerControl::SharedPtr msg);
    void createBody(const obj_track_ros::msg::TrackedObject::SharedPtr msg, m3t::Transform3fA link2world_pose);
    std::string base_frame;
    std_msgs::msg::Header getLatestImageHeader();
    std::vector<std::shared_ptr<m3t::FullNormalRenderer>> normal_renderers;
    std::vector<std::shared_ptr<Ros2ColorCamera>> color_cameras;
    std::vector<std::shared_ptr<Ros2DepthCamera>> depth_cameras;
    std::shared_ptr<m3t::Tracker> tracker;
    std::shared_ptr<m3t::RendererGeometry> geometry;
    std::vector<std::shared_ptr<m3t::FocusedBasicDepthRenderer>> color_renderers;
    std::vector<std::shared_ptr<m3t::FocusedBasicDepthRenderer>> depth_renderers;
    rclcpp::Subscription<obj_track_ros::msg::TrackedObject>::SharedPtr tracked_obj_sub;
    rclcpp::Subscription<obj_track_ros::msg::TrackerControl>::SharedPtr tracker_control_sub;
    rclcpp::Publisher<obj_track_ros::msg::TrackResult>::SharedPtr track_result_pub;
    std::unordered_map<std::string, obj_track_ros::msg::TrackedObject::SharedPtr> name_to_tracked_obj;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
};

}


#endif
