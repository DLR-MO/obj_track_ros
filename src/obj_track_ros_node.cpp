#include "obj_track_ros/obj_track_ros_node.hpp"

using namespace std::chrono_literals;

namespace obj_track_ros
{
  ObjTrackRosNode::ObjTrackRosNode() : Node("minimal_publisher"), Publisher("minimal_publisher"), Subscriber("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&ObjTrackRosNode::timer_callback, this));
  }

  void ObjTrackRosNode::timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  bool ObjTrackRosNode::UpdatePublisher(int iteration)
  {
    rclcpp::spin_some(shared_from_this());
    return false;
  }

  bool ObjTrackRosNode::SetUp()
  {
    return false;
  }

  bool ObjTrackRosNode::UpdateSubscriber(int iteration)
  {
    return false;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<obj_track_ros::ObjTrackRosNode> node = std::make_shared<obj_track_ros::ObjTrackRosNode>();
  std::shared_ptr<m3t::Tracker> tracker;
  tracker->AddPublisher(node);
  rclcpp::shutdown();
  return 0;
}
