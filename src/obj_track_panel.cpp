#include <obj_track_ros/obj_track_panel.hpp>
#include <rviz_common/display_context.hpp>
#include <QVBoxLayout>
#include <QHBoxLayout>

using namespace std::chrono_literals;

namespace obj_track_ros
{
  QMap<QString, QVariant> marker_record_to_qmap(const MarkerRecord &record)
  {
    QMap<QString, QVariant> map;
    map.insert("name", QString::fromStdString(record.name));
    map.insert("file", QString::fromStdString(record.file));
    return map;
  }

  QWidget *ObjTrackPanel::createMarkerListItem(const QString &label, const MarkerRecord &record)
  {
    auto widget = new QWidget();
    auto layout = new QHBoxLayout();
    auto checkbox = new QCheckBox();
    checkbox->setChecked(true);
    layout->addWidget(checkbox);
    layout->addWidget(new QLabel(label));
    auto deleteBtn = new QPushButton("x");
    layout->addWidget(deleteBtn);
    layout->setStretch(0, 1);
    layout->setContentsMargins(2, 2, 2, 2);
    widget->setLayout(layout);

    QObject::connect(deleteBtn, &QPushButton::released, [=]()
                     { 
      server->erase(record.name);
      server->applyChanges();
      markerList->removeItemWidget(record.item);
      int index = 0;
      for(; markers.at(index).item != record.item; index++);
      markers.erase(std::begin(markers) + index); });

    return widget;
  }

  ObjTrackPanel::ObjTrackPanel(QWidget *parent) : Panel(parent)
  {
    const auto layout = new QVBoxLayout(this);
    const auto markersGroup = new QGroupBox(tr("Markers"));
    const auto markerGroupLayout = new QVBoxLayout();
    markersGroup->setLayout(markerGroupLayout);
    layout->addWidget(markersGroup);

    markerList = new QListWidget();
    markerGroupLayout->addWidget(markerList);

    markerName = new QLineEdit();
    markerGroupLayout->addWidget(markerName);
    markerName->setPlaceholderText("Marker name");

    markerFrame = new QLineEdit();
    markerGroupLayout->addWidget(markerFrame);
    markerFrame->setPlaceholderText("Parent Frame");
    QObject::connect(markerFrame, &QLineEdit::textChanged, this, &ObjTrackPanel::configChanged);

    const auto addBtn = new QPushButton("Track Object");
    markerGroupLayout->addWidget(addBtn);
    QObject::connect(addBtn, &QPushButton::released, this, &ObjTrackPanel::addMarker);
  }

  ObjTrackPanel::~ObjTrackPanel() = default;

  void ObjTrackPanel::addMarker()
  {
    auto name = markerName->text().toStdString();
    QString filename = QFileDialog::getOpenFileName(this, tr("Open OBJ"), "~", tr("Wavefront Object (*.obj)"));
    if (filename == nullptr)
      return;

    visualization_msgs::msg::InteractiveMarker im;
    im.header.frame_id = markerFrame->text().toStdString();
    im.scale = 0.25;
    im.name = name.c_str();
    im.description = im.name;

    // make visual marker
    visualization_msgs::msg::Marker m;
    m.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    m.mesh_resource = ("file://" + filename.toStdString());
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

    const auto item = new QListWidgetItem();
    markerList->addItem(item);
    markerName->clear();

    MarkerRecord record;
    record.name = name;
    record.file = filename.toStdString();
    record.item = item;
    record.frame = im.header.frame_id;
    markers.push_back(record);

    markerList->setItemWidget(item, createMarkerListItem(QString::fromStdString(name), record));
    configChanged();
  }

  void loadQLineEdit(rviz_common::Config::MapIterator iter, std::string key, QLineEdit* el)
  {
    if(iter.currentKey().toStdString() == key)
    {
      el->setText(iter.currentChild().getValue().toString());
    }
  }

  void ObjTrackPanel::load(const rviz_common::Config &config)
  {
    if (config.getType() == rviz_common::Config::Type::Map)
    {
      for (auto iter = config.mapIterator(); iter.isValid(); iter.advance())
      {
        std::cout << "Loadings " << iter.currentKey().toStdString() << std::endl;
        loadQLineEdit(iter, "markerFrame", markerFrame);
      }
    }
  }

  void ObjTrackPanel::save(rviz_common::Config config) const
  {
    rviz_common::Panel::save(config);
    if (config.getType() == rviz_common::Config::Type::Map)
    {
      for (int i=0; i < markers.size(); i++)
      {
        config.mapSetValue( QString::fromStdString("$marker-" + std::to_string(i)), marker_record_to_qmap(markers[i]));
      }
      config.mapSetValue("markerFrame", markerFrame->text());
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
