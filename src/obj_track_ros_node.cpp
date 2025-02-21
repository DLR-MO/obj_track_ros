#include "obj_track_ros/obj_track_ros_node.hpp"

using namespace std::chrono_literals;

namespace obj_track_ros
{
  ObjTrackRosNode::ObjTrackRosNode() : Node("obj_track_ros"), Publisher("obj_track_ros"), Subscriber("obj_track_ros")
  {
    camera_color = std::make_shared<obj_track_ros::Ros2ColorCamera>(this, "color_camera", "ee_camera_color", "ee_camera_color_info");
    camera_depth = std::make_shared<obj_track_ros::Ros2DepthCamera>(this, "depth_camera", "ee_camera_depth", "ee_camera_depth_info", 0.1);
  }

  bool ObjTrackRosNode::UpdatePublisher(int iteration)
  {
    rclcpp::spin_some(shared_from_this());
    return true;
  }

  bool ObjTrackRosNode::SetUp()
  {
    return true;
  }

  bool ObjTrackRosNode::UpdateSubscriber(int iteration)
  {
    return true;
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<obj_track_ros::ObjTrackRosNode> node = std::make_shared<obj_track_ros::ObjTrackRosNode>();

  RCLCPP_INFO(node->get_logger(), "Wait for image message...");
  rclcpp::GenericRate rate(100.0);
  while (rclcpp::ok() && !node->camera_color->is_ready() && !node->camera_depth->is_ready())
  {
    rclcpp::spin_some(node->shared_from_this());
    rate.sleep();
  }

  auto tracker{std::make_shared<m3t::Tracker>("tracker")};
  auto renderer_geometry{
      std::make_shared<m3t::RendererGeometry>("renderer_geometry")};

  const std::filesystem::path mug_config_path{
      "/home/wiec_fa/ws/src/obj_track_ros/config/mug.yaml"};
  auto mug{std::make_shared<m3t::Body>("mug", mug_config_path)};
  renderer_geometry->AddBody(mug);

  auto color_depth_renderer{std::make_shared<m3t::FocusedBasicDepthRenderer>(
      "color_depth_renderer", renderer_geometry, node->camera_color)};
  color_depth_renderer->AddReferencedBody(mug);
  auto depth_depth_renderer{std::make_shared<m3t::FocusedBasicDepthRenderer>(
      "depth_depth_renderer", renderer_geometry, node->camera_depth)};
  depth_depth_renderer->AddReferencedBody(mug);


  const std::filesystem::path mug_region_model_path{
      "/home/wiec_fa/ws/src/obj_track_ros/config/mug_region.bin"};
  auto region{std::make_shared<m3t::RegionModel>("mug_region", mug, mug_region_model_path)};
  auto region_modality{std::make_shared<m3t::RegionModality>("mug_region_modal", mug, node->camera_color, region)};
  region_modality->ModelOcclusions(color_depth_renderer);

  const std::filesystem::path mug_depth_model_path{
      "/home/wiec_fa/ws/src/obj_track_ros/config/mug_depth.bin"};
  auto depth{std::make_shared<m3t::DepthModel>("mug_depth", mug, mug_depth_model_path)};
  auto depth_modality{std::make_shared<m3t::DepthModality>("mug_depth_modal", mug, node->camera_depth, depth)};
  depth_modality->ModelOcclusions(depth_depth_renderer);

  
  auto link{std::make_shared<m3t::Link>("mug_link", mug)};
  link->AddModality(depth_modality);
  link->AddModality(region_modality);

  auto optimizer{std::make_shared<m3t::Optimizer>("mug_optimizer", link)};
  tracker->AddOptimizer(optimizer);

  auto normal_viewer{std::make_shared<m3t::NormalColorViewer>(
      "normal_viewer", node->camera_color, renderer_geometry)};
  normal_viewer->set_opacity(0.5);

  tracker->AddViewer(normal_viewer);

  const std::filesystem::path mug_detector_path{
      "/home/wiec_fa/ws/src/obj_track_ros/config/mug_detector.yaml"};
  auto detector{std::make_shared<m3t::StaticDetector>(
      "detector", mug_detector_path, optimizer)};
  tracker->AddDetector(detector);

  tracker->AddPublisher(node);
  tracker->AddSubscriber(node);

  if (!tracker->SetUp())
  {
    rclcpp::shutdown();
    return -1;
  };
  if (!tracker->RunTrackerProcess(true, true))
  {
    rclcpp::shutdown();
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}
