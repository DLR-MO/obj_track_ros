#include "obj_track_ros/obj_track_ros_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
// using namespace cv;

namespace obj_track_ros
{
  ObjTrackRosNode::ObjTrackRosNode() : Node("obj_track_ros"), Publisher("obj_track_ros"), Subscriber("obj_track_ros"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    img_sub = this->create_subscription<sensor_msgs::msg::Image>("ee_camera_color", 10, std::bind(&ObjTrackRosNode::receive_image, this, _1));
    cam_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/ee_camera_color_info", 10, std::bind(&ObjTrackRosNode::receive_cam_info, this, _1));
    timer_ = this->create_wall_timer(500ms, std::bind(&ObjTrackRosNode::timer_callback, this));
    camera_color = std::make_shared<obj_track_ros::Ros2ColorCamera>("color_camera");
}

  void ObjTrackRosNode::timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    publisher_->publish(message);
  }

  void ObjTrackRosNode::receive_image(const sensor_msgs::msg::Image::SharedPtr msg) 
  {
    auto cvim = cv_bridge::toCvCopy(msg, msg->encoding);
    cv::cvtColor(cvim->image, cvim->image, cv::COLOR_BGR2RGB);
    camera_color->setImage(cvim->image);
  }

  void ObjTrackRosNode::receive_cam_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg) 
  {
    camera_color->setCameraInfo(msg);
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<obj_track_ros::ObjTrackRosNode> node = std::make_shared<obj_track_ros::ObjTrackRosNode>();
  
  RCLCPP_INFO(node->get_logger(), "Wait for image message...");
  rclcpp::GenericRate rate(100.0);
  while(rclcpp::ok() && !node->camera_color->is_ready())
  {
    rclcpp::spin_some(node->shared_from_this());
    rate.sleep();
  }
  
  auto tracker{std::make_shared<m3t::Tracker>("tracker")};
  auto renderer_geometry{
    std::make_shared<m3t::RendererGeometry>("renderer_geometry")};

  const std::filesystem::path mug_config_path {
    "/home/wiec_fa/ws/src/obj_track_ros/config/mug.yaml"
  };
  auto mug{std::make_shared<m3t::Body>("mug", mug_config_path)};
  renderer_geometry->AddBody(mug);

  const std::filesystem::path mug_model_path {
    "/home/wiec_fa/ws/src/obj_track_ros/config/mug.bin"
  };
  auto region { std::make_shared<m3t::RegionModel>("mug_region", mug, mug_model_path) };
  auto region_modality { std::make_shared<m3t::RegionModality>("mug_region_modal", mug, node->camera_color, region) };
  
  auto link{std::make_shared<m3t::Link>("mug_link", mug)};
  link->AddModality(region_modality);

  auto optimizer{
    std::make_shared<m3t::Optimizer>("mug_optimizer", link)};
  tracker->AddOptimizer(optimizer);

  auto normal_viewer{std::make_shared<m3t::NormalColorViewer>(
    "normal_viewer", node->camera_color, renderer_geometry)};
  normal_viewer->set_opacity(0.5);

  tracker->AddViewer(normal_viewer);

  const std::filesystem::path mug_detector_path {
    "/home/wiec_fa/ws/src/obj_track_ros/config/mug_detector.yaml"
  };
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
