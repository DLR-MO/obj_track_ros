#include "obj_track_ros/obj_track_ros_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace obj_track_ros
{


  ObjTrackRosNode::ObjTrackRosNode() : Node("obj_track_ros"), Publisher("obj_track_ros"), Subscriber("obj_track_ros"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    img_sub = this->create_subscription<sensor_msgs::msg::Image>("ee_camera_color", 10, std::bind(&ObjTrackRosNode::receive_image, this, _1));
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
    camera_color->setImage(cvim->image);
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
  while(rclcpp::ok() && !node->camera_color->is_ready)
  {
    rclcpp::spin_some(node->shared_from_this());
    rate.sleep();
  }
  
  auto tracker{std::make_shared<m3t::Tracker>("tracker")};

  const std::filesystem::path configfile_path {
    "/home/wiec_fa/ws/src/obj_track_ros/submodules/3DObjectTracking/M3T/data/pen_paper_demo/config.yaml"
  };
  
  auto color_viewer{std::make_shared<m3t::ImageColorViewer>("color_viewer", node->camera_color)};

  tracker->AddViewer(color_viewer);
  // if (!GenerateConfiguredTracker(configfile_path, &tracker)) {
  //   rclcpp::shutdown();
  //   return -1;
  // }

  tracker->AddPublisher(node);
  if (!tracker->SetUp()) 
  {
    rclcpp::shutdown();
    return -1;
  };
  if (!tracker->RunTrackerProcess(false, false))
  {
    rclcpp::shutdown();
    return -1;
  }

  
  rclcpp::shutdown();
  return 0;
}
