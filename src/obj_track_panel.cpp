#include <obj_track_ros/obj_track_panel.hpp>
#include <rviz_common/display_context.hpp>
#include <QVBoxLayout>
#include <QHBoxLayout>

using namespace std::chrono_literals;

namespace obj_track_ros
{

  ObjTrackPanel::ObjTrackPanel(QWidget *parent) : Panel(parent)
  {

  }

  ObjTrackPanel::~ObjTrackPanel() = default;

  void loadQLineEdit(rviz_common::Config::MapIterator iter, std::string key, QLineEdit* el)
  {
    if(iter.currentKey().toStdString() == key)
    {
      el->setText(iter.currentChild().getValue().toString());
    }
  }

  void ObjTrackPanel::load(const rviz_common::Config & config)
  {
    if(config.getType() == rviz_common::Config::Type::Map)
    {
      for(auto iter = config.mapIterator(); iter.isValid(); iter.advance())
      {

      }
    }
  }

  void ObjTrackPanel::save(rviz_common::Config config) const
  {
    rviz_common::Panel::save(config);
    if(config.getType() == rviz_common::Config::Type::Map)
    {

    }
  }

  void ObjTrackPanel::onInitialize()
  {
    node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();
    node = node_ptr_->get_raw_node();
  }


} // namespace obj_track_ros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(obj_track_ros::ObjTrackPanel, rviz_common::Panel)
