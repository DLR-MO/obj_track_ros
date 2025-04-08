#ifndef OBJ_TRACK_ROS__OBJ_TRACK_PANEL_HPP_
#define OBJ_TRACK_ROS__OBJ_TRACK_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/client.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

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

namespace obj_track_ros
{
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

private:


private Q_SLOTS:

};

}  // namespace obj_track_ros

#endif  // OBJ_TRACK_ROS__OBJ_TRACK_PANEL_HPP_
