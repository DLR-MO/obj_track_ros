#ifndef OBJ_TRACK_ROS__OBJ_TRACK_PANEL_HPP_
#define OBJ_TRACK_ROS__OBJ_TRACK_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/client.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <chrono>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/transform_datatypes.h"

#include "interactive_markers/interactive_marker_server.hpp"
#include "obj_track_ros/msg/tracked_object.hpp"
#include "obj_track_ros/msg/tracker_control.hpp"

#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QGroupBox>
#include <QWidget>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QStringList>
#include <QFileDialog>
#include <QListWidget>
#include <QListWidgetItem>
#include <QList>
#include <QCheckBox>
#include <QMap>

namespace obj_track_ros
{
struct MarkerRecord
{
  std::string name;
  std::string file;
  std::string frame;
  QListWidgetItem* item;
};

class ObjTrackPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit ObjTrackPanel(QWidget * parent = 0);
  ~ObjTrackPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<rclcpp::Publisher<msg::TrackedObject>> track_obj_pub;
  std::shared_ptr<rclcpp::Publisher<msg::TrackerControl>> track_control_pub;

private:
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server;
  QWidget* createMarkerListItem(const QString & label, const MarkerRecord & record);
  QListWidget* markerList;
  QLineEdit* markerName;
  QLineEdit* markerFrame;
  std::vector<MarkerRecord> markers;
  visualization_msgs::msg::InteractiveMarker createInteractiveMarker(std::string name, std::string frame, std::string filename);
  void addMarker(std::string name, std::string frame, std::string filename);

private Q_SLOTS:
  void onTrackObject();
  void stopTracking();
  void startTracking();
};

}  // namespace obj_track_ros

#endif  // OBJ_TRACK_ROS__OBJ_TRACK_PANEL_HPP_
