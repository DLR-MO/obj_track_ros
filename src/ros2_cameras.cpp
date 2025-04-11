#include "obj_track_ros/ros2_cameras.hpp"

using std::placeholders::_1;

namespace m3t {
  extern cv::Mat CalculateAlphaBlend(const cv::Mat &camera_image, const cv::Mat &renderer_image, float opacity);
}

namespace obj_track_ros
{
  Ros2ColorCamera::Ros2ColorCamera(
      rclcpp::Node *node,
      const std::string &name,
      const std::string &img_topic,
      const std::string &info_topic,
      const std::string &frame,
      bool publish_overlay_) : ColorCamera(name)
  {
    this->publish_overlay = publish_overlay_;
    this->frame = frame;
    img_sub = node->create_subscription<sensor_msgs::msg::Image>(img_topic, 1, std::bind(&Ros2ColorCamera::setImage, this, _1));
    info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>(info_topic, 1, std::bind(&Ros2ColorCamera::setCameraInfo, this, _1));
    if(publish_overlay)
    {
      img_pub = node->create_publisher<sensor_msgs::msg::Image>("obj_track_ros/overlay/" + name, rclcpp::SystemDefaultsQoS());
    }
  }

  bool Ros2ColorCamera::SetUp()
  {
    set_up_ = true;
    return true;
  }

  bool Ros2ColorCamera::UpdateImage(bool synchronized)
  {
    return true;
  }

  void Ros2ColorCamera::setImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto cvim = cv_bridge::toCvCopy(msg, msg->encoding);
    cv::cvtColor(cvim->image, cvim->image, cv::COLOR_BGR2RGB);
    image_ = cvim->image;
    header = msg->header;
    image_ready = true;
  }

  void Ros2ColorCamera::setCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    intrinsics_.fu = msg->k[0];
    intrinsics_.fv = msg->k[4];
    intrinsics_.ppu = msg->k[2];
    intrinsics_.ppv = msg->k[5];
    intrinsics_.width = msg->width;
    intrinsics_.height = msg->height;
    intrinsics_ready = true;
  }

  bool Ros2ColorCamera::is_ready()
  {
    return image_ready && intrinsics_ready;
  }

  void Ros2ColorCamera::publishOverlay(cv::Mat overlay)
  {
    auto im = m3t::CalculateAlphaBlend(image_, overlay, 0.5);
    auto cvim = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, im);
    sensor_msgs::msg::Image msg;
    cvim.toImageMsg(msg);
    img_pub->publish(msg);
  }

  void Ros2ColorCamera::updatePose(std::shared_ptr<tf2_ros::Buffer> buffer, std::string base_frame)
  {
    if(buffer->canTransform(base_frame, frame, rclcpp::Time()))
    {
      auto transform = buffer->lookupTransform(base_frame, frame, rclcpp::Time()).transform;
      auto q = transform.rotation;
      auto p = transform.translation;
      Eigen::Quaternionf quat(q.w, q.x, q.y, q.z);
      m3t::Transform3fA t;
      t = quat.toRotationMatrix();
      t(0, 3) = p.x;
      t(1, 3) = p.y;
      t(2, 3) = p.z;
      set_camera2world_pose(t);
    }
  }

  std_msgs::msg::Header Ros2ColorCamera::getHeader()
  {
    return header;
  }

  Ros2DepthCamera::Ros2DepthCamera(
      rclcpp::Node *node,
      const std::string &name,
      const std::string &img_topic,
      const std::string &info_topic,
      const std::string &frame,
      float depth_scale
      ) : DepthCamera(name)
  {
    this->frame = frame;
    depth_scale_ = depth_scale;
    img_sub = node->create_subscription<sensor_msgs::msg::Image>(img_topic, 1, std::bind(&Ros2DepthCamera::setImage, this, _1));
    info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>(info_topic, 1, std::bind(&Ros2DepthCamera::setCameraInfo, this, _1));
  }

  bool Ros2DepthCamera::SetUp()
  {
    set_up_ = true;
    return true;
  }

  bool Ros2DepthCamera::UpdateImage(bool synchronized)
  {
    return true;
  }

  void Ros2DepthCamera::setImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto cvim = cv_bridge::toCvCopy(msg, msg->encoding);
    cv::cvtColor(cvim->image, cvim->image, cv::COLOR_BGR2RGB);
    image_ = cvim->image;
    header = msg->header;
    image_ready = true;
  }

  void Ros2DepthCamera::setCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    intrinsics_.fu = msg->k[0];
    intrinsics_.fv = msg->k[4];
    intrinsics_.ppu = msg->k[2];
    intrinsics_.ppv = msg->k[5];
    intrinsics_.width = msg->width;
    intrinsics_.height = msg->height;
    intrinsics_ready = true;
  }

  bool Ros2DepthCamera::is_ready()
  {
    return image_ready && intrinsics_ready;
  }

  void Ros2DepthCamera::updatePose(std::shared_ptr<tf2_ros::Buffer> buffer, std::string base_frame)
  {
    if(buffer->canTransform(base_frame, frame, rclcpp::Time()))
    {
      auto transform = buffer->lookupTransform(base_frame, frame, rclcpp::Time()).transform;
      auto q = transform.rotation;
      auto p = transform.translation;
      Eigen::Quaternionf quat(q.x, q.y, q.z, q.w);
      m3t::Transform3fA t;
      t = quat.toRotationMatrix();
      t(0, 3) = p.x;
      t(1, 3) = p.y;
      t(2, 3) = p.z;
      // set_camera2world_pose(t);
      set_world2camera_pose(t);
    }
  }

  std_msgs::msg::Header Ros2DepthCamera::getHeader()
  {
    return header;
  }
}
