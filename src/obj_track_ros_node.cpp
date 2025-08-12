// SPDX-FileCopyrightText: 2025 Fabian Wieczorek, German Aerospace Center (DLR)
// SPDX-License-Identifier: MIT

#include "obj_track_ros/obj_track_ros_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace m3t
{
  template <typename T>
  extern bool AddPtrIfNameNotExists(const T &ptr, std::vector<T> *dest_ptrs);
}

namespace obj_track_ros
{
  ObjTrackRosNode::ObjTrackRosNode() : Node("obj_track_ros"), Publisher("obj_track_ros"), Subscriber("obj_track_ros")
  {
    auto cam_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    cam_descriptor.description = "List of camera config files to use for tracking";
    cam_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
    auto cam_defaults = rclcpp::ParameterValue(std::vector<std::string>());
    declare_parameter("camera_configs", cam_defaults, cam_descriptor);

    auto frame_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    frame_descriptor.description = "The base frame of the world";
    frame_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    auto frame_defaults = rclcpp::ParameterValue("world");
    declare_parameter("base_frame", frame_defaults, frame_descriptor);
    base_frame = get_parameter("base_frame").as_string();

    tracker = std::make_shared<m3t::Tracker>("tracker");
    geometry = std::make_shared<m3t::RendererGeometry>("geometry");
    configureCameras(get_parameter("camera_configs").as_string_array());

    tracked_obj_sub = create_subscription<msg::TrackedObject>("/tracker/objects", 10, std::bind(&ObjTrackRosNode::receiveTrackedBody, this, _1));
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    tracker_control_sub = create_subscription<msg::TrackerControl>("/tracker/control", 10, std::bind(&ObjTrackRosNode::receiveTrackerControl, this, _1));
    track_result_pub = create_publisher<msg::TrackResult>("/tracker/result", 1);
  }

  void ObjTrackRosNode::configureCameras(const std::vector<std::string> &camera_configs)
  {
    for (const std::string &filename : camera_configs)
    {
      YAML::Node root = YAML::LoadFile(filename);
      for (std::size_t i = 0; i < root.size(); i++)
      {
        auto name = root[i]["name"].as<std::string>();
        auto type = root[i]["type"].as<std::string>();
        auto frame = root[i]["frame"].as<std::string>();
        auto image_topic = root[i]["image_topic"].as<std::string>();
        auto info_topic = root[i]["info_topic"].as<std::string>();
        auto publish_overlay = root[i]["publish_overlay"].as<bool>(false);

        if (type == "Ros2ColorCamera")
        {
          auto camera = std::make_shared<obj_track_ros::Ros2ColorCamera>(this, name, image_topic, info_topic, frame, publish_overlay);
          color_cameras.push_back(camera);
          auto renderer = std::make_shared<m3t::FocusedBasicDepthRenderer>(name + "_depth_renderer", geometry, camera);
          color_renderers.push_back(renderer);
          if (publish_overlay)
          {
            auto renderer = std::make_shared<m3t::FullNormalRenderer>(name + "_normal_renderer", geometry, camera);
            normal_renderers.push_back(renderer);
          }
        }
        else if (type == "Ros2DepthCamera")
        {
          float scale = root[i]["scale"].as<float>(1.0);
          auto camera = std::make_shared<obj_track_ros::Ros2DepthCamera>(this, name, image_topic, info_topic, frame, scale);
          depth_cameras.push_back(camera);
          auto renderer = std::make_shared<m3t::FocusedBasicDepthRenderer>(name + "_depth_renderer", geometry, camera);
          depth_renderers.push_back(renderer);
        }
      }
    }
  }

  void ObjTrackRosNode::waitForCameras()
  {
    RCLCPP_INFO(get_logger(), "Wait for cameras...");
    rclcpp::GenericRate rate(100.0);
    bool cameras_ready = false;
    while (rclcpp::ok() && !cameras_ready)
    {
      rclcpp::spin_some(shared_from_this());
      cameras_ready = true;
      for (auto &camera : color_cameras)
      {
        cameras_ready = cameras_ready && camera->is_ready();
      }
      for (auto &camera : depth_cameras)
      {
        cameras_ready = cameras_ready && camera->is_ready();
      }
      rate.sleep();
    }
    RCLCPP_INFO(get_logger(), "All cameras ready");
  }

  std_msgs::msg::Header ObjTrackRosNode::getLatestImageHeader()
  {
    std_msgs::msg::Header latest;
    for (auto &camera : color_cameras)
    {
      auto current = camera->getHeader();
      if (current.stamp.sec > latest.stamp.sec || current.stamp.sec == latest.stamp.sec && current.stamp.nanosec > latest.stamp.nanosec)
      {
        latest = current;
      }
    }
    for (auto &camera : depth_cameras)
    {
      auto current = camera->getHeader();
      if (current.stamp.sec > latest.stamp.sec || current.stamp.sec == latest.stamp.sec && current.stamp.nanosec > latest.stamp.nanosec)
      {
        latest = current;
      }
    }
    return latest;
  }

  void ObjTrackRosNode::receiveTrackedBody(const obj_track_ros::msg::TrackedObject::SharedPtr msg)
  {
    if (msg->command == msg->COMMAND_SET)
    {
      m3t::Transform3fA link2world_pose;
      for (int i = 0; i < 16; i++)
      {
        link2world_pose(i / 4, i % 4) = msg->detector_world_pose[i];
      }

      std::shared_ptr<m3t::Body> targetBody = nullptr;
      for (auto body : tracker->body_ptrs())
      {
        if (body->name() == msg->name)
          targetBody = body;
      }

      if (targetBody == nullptr)
      {
        createBody(msg, link2world_pose);
      }
      else
      {
        targetBody->set_body2world_pose(link2world_pose);
      }
    }
    else if(msg->command == msg->COMMAND_DELETE)
    {
      // TODO implement body deletion
      RCLCPP_WARN_STREAM(get_logger(), "Delete bodies not implemented yet!");
    }
  }

  void ObjTrackRosNode::createBody(const obj_track_ros::msg::TrackedObject::SharedPtr msg, m3t::Transform3fA link2world_pose)
  {
    m3t::Transform3fA geometry2body_pose;
    for (int i = 0; i < 16; i++)
    {
      geometry2body_pose(i / 4, i % 4) = msg->geometry2body_pose[i];
    }

    auto body = std::make_shared<m3t::Body>(
        msg->name,
        msg->geometry_path,
        msg->geometry_unit_in_meter,
        msg->geometry_counterclockwise,
        msg->geometry_enable_culling,
        geometry2body_pose);

    geometry->AddBody(body);

    auto link{std::make_shared<m3t::Link>(msg->name + "_link", body)};

    const std::filesystem::path body_region_model_path{"/tmp/" + msg->name + "_region_model.bin"};
    auto region{std::make_shared<m3t::RegionModel>(msg->name + "_region", body, body_region_model_path)};

    const std::filesystem::path body_depth_model_path{"/tmp/" + msg->name + "_depth_model.bin"};
    auto depth{std::make_shared<m3t::DepthModel>(msg->name + "_depth", body, body_depth_model_path)};

    for (int i = 0; i < color_cameras.size(); i++)
    {
      color_renderers[i]->AddReferencedBody(body);
      auto region_modality{std::make_shared<m3t::RegionModality>(msg->name + "_region_modal_" + color_cameras[i]->name(), body, color_cameras[i], region)};
      region_modality->ModelOcclusions(color_renderers[i]);
      link->AddModality(region_modality);
    }

    for (int i = 0; i < depth_cameras.size(); i++)
    {
      depth_renderers[i]->AddReferencedBody(body);
      auto depth_modality{std::make_shared<m3t::DepthModality>(msg->name + "_depth_modal_" + depth_cameras[i]->name(), body, depth_cameras[i], depth)};
      depth_modality->ModelOcclusions(depth_renderers[i]);
      link->AddModality(depth_modality);
    }

    auto optimizer{std::make_shared<m3t::Optimizer>(msg->name + "_optimizer", link)};
    tracker->AddOptimizer(optimizer);

    auto detector{std::make_shared<m3t::StaticDetector>(msg->name + "_detector", optimizer, link2world_pose)};
    tracker->AddDetector(detector);

    tracker->SetUp();
    for (auto &renderer : normal_renderers)
    {
      renderer->SetUp();
    }

    name_to_tracked_obj[msg->name] = msg;

    RCLCPP_INFO(get_logger(), ("Added body " + msg->name).c_str());
    tracker->QuitTrackerProcess();
  }

  bool ObjTrackRosNode::UpdateSubscriber(int iteration)
  {
    for (auto &camera : color_cameras)
    {
      camera->updatePose(tf_buffer, base_frame);
    }
    for (auto &camera : depth_cameras)
    {
      camera->updatePose(tf_buffer, base_frame);
    }
    return true;
  }

  bool ObjTrackRosNode::UpdatePublisher(int iteration)
  {
    int renderer_index = 0;
    if (normal_renderers.size() > 0)
    {
      for (int i = 0; i < color_cameras.size(); i++)
      {
        if (color_cameras[i]->publish_overlay && normal_renderers[renderer_index]->set_up())
        {
          auto renderer = normal_renderers[renderer_index];
          renderer->set_camera2world_pose(renderer->camera_ptr()->camera2world_pose());
          renderer->StartRendering();
          renderer->FetchNormalImage();
          auto image = renderer->normal_image();
          color_cameras[i]->publishOverlay(image);
          renderer_index += 1;
        }
      }
    }

    for (auto &body : tracker->body_ptrs())
    {
      auto transform = body->body2world_pose();
      auto frame = name_to_tracked_obj[body->name()]->frame;
      auto latest_header = getLatestImageHeader();

      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = latest_header.stamp;
      t.header.frame_id = base_frame;
      t.child_frame_id = frame.c_str();
      Eigen::Quaternionf quat(transform.linear());
      t.transform.rotation.x = quat.x();
      t.transform.rotation.y = quat.y();
      t.transform.rotation.z = quat.z();
      t.transform.rotation.w = quat.w();
      auto pos = transform.translation();
      t.transform.translation.x = pos.x();
      t.transform.translation.y = pos.y();
      t.transform.translation.z = pos.z();

      tf_broadcaster->sendTransform(t);

      msg::TrackResult result;
      result.header = t.header;
      result.name = body->name();
      result.frame = frame;
      result.filename = body->geometry_path();
      result.pose.orientation.x = quat.x();
      result.pose.orientation.y = quat.y();
      result.pose.orientation.z = quat.z();
      result.pose.orientation.w = quat.w();
      result.pose.position.x = pos.x();
      result.pose.position.y = pos.y();
      result.pose.position.z = pos.z();
      for (int i = 0; i < 16; i++)
      {
        result.transform[i] = transform(i / 4, i % 4);
      }
      track_result_pub->publish(result);
    }

    rclcpp::spin_some(shared_from_this());
    return true;
  }

  void ObjTrackRosNode::receiveTrackerControl(const obj_track_ros::msg::TrackerControl::SharedPtr msg)
  {
    if (msg->is_stopped)
    {
      tracker->StopTracking();
      RCLCPP_INFO(get_logger(), "Tracking stopped");
    }
    else
    {
      tracker->StartTracking();
      RCLCPP_INFO(get_logger(), "Tracking started");
    }
  }

  bool ObjTrackRosNode::SetUp()
  {
    return true;
  }

  std::shared_ptr<m3t::Tracker> ObjTrackRosNode::getTracker()
  {
    return tracker;
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<obj_track_ros::ObjTrackRosNode> node = std::make_shared<obj_track_ros::ObjTrackRosNode>();
  node->getTracker()->AddPublisher(node);
  node->getTracker()->AddSubscriber(node);
  node->waitForCameras();

  RCLCPP_INFO(node->get_logger(), "Setup tracker");
  if (!node->getTracker()->SetUp())
  {
    rclcpp::shutdown();
    return -1;
  };

  RCLCPP_INFO(node->get_logger(), "Start tracker process");
  while (true)
  {
    if (!node->getTracker()->RunTrackerProcess(true, true))
    {
      rclcpp::shutdown();
      return -1;
    }
    RCLCPP_INFO(node->get_logger(), "Restart tracker process");
  }

  rclcpp::shutdown();
  return 0;
}
