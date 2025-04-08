#include <obj_track_ros/obj_track_panel.hpp>
#include <rviz_common/display_context.hpp>
#include <QVBoxLayout>
#include <QHBoxLayout>

using namespace std::chrono_literals;

namespace obj_track_ros
{
  ObjTrackPanel::ObjTrackPanel(QWidget *parent) : Panel(parent)
  {
    const auto layout = new QVBoxLayout(this);
    const auto addBtn = new QPushButton("Add Marker");
    layout->addWidget(addBtn);

    QObject::connect(addBtn, &QPushButton::released, this, &ObjTrackPanel::addMarker);
  }

  ObjTrackPanel::~ObjTrackPanel() = default;

  void ObjTrackPanel::addMarker()
  {
    visualization_msgs::msg::InteractiveMarker im;
    im.header.frame_id = "odom";
    im.scale = 1;
    im.name = "marker";
    im.description = "marker";

    // make visual marker
    visualization_msgs::msg::Marker m;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 1.0;
    m.color.a = 1.0;

    // make a 6-dof control 
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.markers.push_back(m);
    control.always_visible = true;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
    im.controls.push_back(control);
    control.markers.clear();

    tf2::Quaternion orien(1.0, 0.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    im.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    im.controls.push_back(control);
    orien = tf2::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    im.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    im.controls.push_back(control);
    orien = tf2::Quaternion(0.0, 0.0, 1.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    im.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    im.controls.push_back(control);

    server->insert(im);
    server->applyChanges();
  }

  void ObjTrackPanel::load(const rviz_common::Config &config)
  {
    if (config.getType() == rviz_common::Config::Type::Map)
    {
      for (auto iter = config.mapIterator(); iter.isValid(); iter.advance())
      {
      }
    }
  }

  void ObjTrackPanel::save(rviz_common::Config config) const
  {
    rviz_common::Panel::save(config);
    if (config.getType() == rviz_common::Config::Type::Map)
    {
    }
  }

  void ObjTrackPanel::onInitialize()
  {
    node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();
    node = node_ptr_->get_raw_node();
    server = std::make_unique<interactive_markers::InteractiveMarkerServer>(
        "obj_track_ros",
        node->get_node_base_interface(),
        node->get_node_clock_interface(),
        node->get_node_logging_interface(),
        node->get_node_topics_interface(),
        node->get_node_services_interface());
  }
} // namespace obj_track_ros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(obj_track_ros::ObjTrackPanel, rviz_common::Panel)
