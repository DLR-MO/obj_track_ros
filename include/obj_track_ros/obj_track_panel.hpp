// SPDX-FileCopyrightText: 2025 Fabian Wieczorek, German Aerospace Center (DLR)
// SPDX-License-Identifier: MIT

#ifndef OBJ_TRACK_ROS__OBJ_TRACK_PANEL_HPP_
#define OBJ_TRACK_ROS__OBJ_TRACK_PANEL_HPP_

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>
#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "interactive_markers/interactive_marker_server.hpp"
#include "obj_track_ros/msg/tracked_object.hpp"
#include "obj_track_ros/msg/tracker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QList>
#include <QListWidget>
#include <QListWidgetItem>
#include <QMap>
#include <QPushButton>
#include <QSpinBox>
#include <QString>
#include <QStringList>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QWidget>

namespace obj_track_ros {
struct MarkerRecord {
  std::string name;
  std::string file;
  std::string frame;
  double transform[12];
  QListWidgetItem *item;
};

class ObjTrackPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit ObjTrackPanel(QWidget *parent = 0);
  ~ObjTrackPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config &config) override;
  void save(rviz_common::Config config) const override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface>
      node_ptr_;
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<rclcpp::Publisher<msg::TrackedObject>> track_obj_pub;
  std::shared_ptr<rclcpp::Publisher<msg::TrackerControl>> track_control_pub;
  std::shared_ptr<
      rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerFeedback>>
      marker_feedback_sub;

private:
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server;
  QWidget *createMarkerListItem(const QString &label,
                                const MarkerRecord &record);
  QListWidget *markerList;
  QLineEdit *markerName;
  QLineEdit *markerFrame;
  std::vector<MarkerRecord> markers;
  visualization_msgs::msg::InteractiveMarker
  createInteractiveMarker(std::string name, std::string frame,
                          std::string filename, double *transform);
  void addMarker(std::string name, std::string frame, std::string filename,
                 double *transform);
  void onMarkerUpdate(
      const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr msg);
private Q_SLOTS:
  void onTrackObject();
  void stopTracking();
  void startTracking();
};

} // namespace obj_track_ros

#endif // OBJ_TRACK_ROS__OBJ_TRACK_PANEL_HPP_
