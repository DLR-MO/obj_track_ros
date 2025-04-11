#include <obj_track_ros/obj_track_panel.hpp>
#include <rviz_common/display_context.hpp>
#include <QVBoxLayout>
#include <QHBoxLayout>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace obj_track_ros
{
  QWidget *ObjTrackPanel::createMarkerListItem(const QString &label, const MarkerRecord &record)
  {
    auto widget = new QWidget();
    auto layout = new QHBoxLayout();
    auto checkbox = new QCheckBox();
    checkbox->setChecked(true);
    layout->addWidget(checkbox);
    layout->addWidget(new QLabel(label));
    auto deleteBtn = new QPushButton("Remove");
    layout->addWidget(deleteBtn);
    layout->setStretch(0, 1);
    layout->setContentsMargins(2, 2, 2, 2);
    widget->setLayout(layout);

    QObject::connect(deleteBtn, &QPushButton::released, [=]()
                     { 
      server->erase(record.name);
      server->applyChanges();
      markerList->removeItemWidget(record.item);
      markerList->takeItem(markerList->row(record.item));
      int index = 0;
      for(; markers.at(index).item != record.item; index++);
      markers.erase(std::begin(markers) + index); 
      configChanged(); });

    return widget;
  }

  ObjTrackPanel::ObjTrackPanel(QWidget *parent) : Panel(parent)
  {
    const auto layout = new QVBoxLayout(this);

    const auto m3tControlGroup = new QGroupBox(tr("Tracking Controls"));
    const auto m3tControlLayout = new QHBoxLayout();
    m3tControlGroup->setLayout(m3tControlLayout);
    const auto stopBtn = new QPushButton(tr("Pause Tracking"));
    m3tControlLayout->addWidget(stopBtn);
    QObject::connect(stopBtn, &QPushButton::released, this, &ObjTrackPanel::stopTracking);
    layout->addWidget(m3tControlGroup);
    const auto startBtn = new QPushButton(tr("Resume Tracking"));
    m3tControlLayout->addWidget(startBtn);
    QObject::connect(startBtn, &QPushButton::released, this, &ObjTrackPanel::startTracking);
    layout->addWidget(m3tControlGroup);

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
    QObject::connect(addBtn, &QPushButton::released, this, &ObjTrackPanel::onTrackObject);
  }

  ObjTrackPanel::~ObjTrackPanel() = default;

  void ObjTrackPanel::stopTracking()
  {
    msg::TrackerControl msg;
    msg.is_stopped = true;
    track_control_pub->publish(msg);
  }

  void ObjTrackPanel::startTracking()
  {
    msg::TrackerControl msg;
    msg.is_stopped = false;
    track_control_pub->publish(msg);
  }

  visualization_msgs::msg::InteractiveMarker ObjTrackPanel::createInteractiveMarker(std::string name, std::string frame, std::string filename, double* transform)
  {
    visualization_msgs::msg::InteractiveMarker im;
    im.header.frame_id = frame;
    im.scale = 0.25;
    im.name = name.c_str();
    im.description = im.name;

    tf2::Matrix3x3 rot;
    rot[0][0] = transform[0];
    rot[0][1] = transform[1];
    rot[0][2] = transform[2];
    rot[1][0] = transform[4];
    rot[1][1] = transform[5];
    rot[1][2] = transform[6];
    rot[2][0] = transform[8];
    rot[2][1] = transform[9];
    rot[2][2] = transform[10];
    tf2::Quaternion quat;
    rot.getRotation(quat);
    im.pose.orientation.x = quat.x();
    im.pose.orientation.y = quat.y();
    im.pose.orientation.z = quat.z();
    im.pose.orientation.w = quat.w();
    im.pose.position.x = transform[3];
    im.pose.position.y = transform[7];
    im.pose.position.z = transform[11];

    // make visual marker
    visualization_msgs::msg::Marker m;
    m.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    m.mesh_resource = "file://" + filename;
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

    return im;
  }

  void ObjTrackPanel::addMarker(std::string name, std::string frame, std::string filename, double* transform)
  {
    auto im = createInteractiveMarker(name, frame, filename, transform);
    server->insert(im);
    server->applyChanges();

    const auto item = new QListWidgetItem();
    markerList->addItem(item);
    markerName->clear();

    MarkerRecord record;
    record.name = name;
    record.file = filename;
    record.item = item;
    record.frame = im.header.frame_id;
    markers.push_back(record);

    const auto markerWidget = createMarkerListItem(QString::fromStdString(name), record);
    markerList->setItemWidget(item, markerWidget);

    msg::TrackedObject msg;
    msg.frame = name;
    msg.name = name;
    msg.geometry_path = filename;
    for(int i=0; i<12; i++)
      msg.detector_world_pose[i] = transform[i];
    track_obj_pub->publish(msg);
  }

  void ObjTrackPanel::onMarkerUpdate(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr msg)
  {
    msg::TrackedObject tracked_obj;
    tracked_obj.name = msg->marker_name;
    tf2::Quaternion quat(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf2::Matrix3x3 rot(quat);
    tracked_obj.detector_world_pose[0] = rot[0][0];
    tracked_obj.detector_world_pose[1] = rot[0][1];
    tracked_obj.detector_world_pose[2] = rot[0][2];
    tracked_obj.detector_world_pose[3] = msg->pose.position.x;
    tracked_obj.detector_world_pose[4] = rot[1][0];
    tracked_obj.detector_world_pose[5] = rot[1][1];
    tracked_obj.detector_world_pose[6] = rot[1][2];
    tracked_obj.detector_world_pose[7] = msg->pose.position.y;
    tracked_obj.detector_world_pose[8] = rot[2][0];
    tracked_obj.detector_world_pose[9] = rot[2][1];
    tracked_obj.detector_world_pose[10] = rot[2][2];
    tracked_obj.detector_world_pose[11] = msg->pose.position.z;
    track_obj_pub->publish(tracked_obj);

    int index = 0;
    for (; markers.at(index).name != msg->marker_name; index++)
      ;
    for (int k = 0; k < 12; k++)
      markers[index].transform[k] = tracked_obj.detector_world_pose[k];
    
    configChanged();
  }

  void ObjTrackPanel::onTrackObject()
  {
    auto name = markerName->text().toStdString();
    QString filename = QFileDialog::getOpenFileName(this, tr("Open OBJ"), "~", tr("Wavefront Object (*.obj)"));
    if (filename == nullptr)
      return;
    double transform[] = {1.0, 0.0, 0.0, 0.0,  0.0, 1.0, 0.0, 0.0,  0.0, 0.0, 1.0, 0.0};
    addMarker(name, markerFrame->text().toStdString(), filename.toStdString(), transform);
    configChanged();
  }

  void loadQLineEdit(rviz_common::Config::MapIterator iter, std::string key, QLineEdit *el)
  {
    if (iter.currentKey().toStdString() == key)
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
        if (iter.currentKey().toStdString().rfind("$marker-", 0) == 0)
        {
          QString name, frame, file;
          iter.currentChild().mapGetString("name", &name);
          iter.currentChild().mapGetString("frame", &frame);
          iter.currentChild().mapGetString("file", &file);
          auto transformConfig = iter.currentChild().mapGetChild("transform");
          double transform[12];
          for (auto iter_t = transformConfig.mapIterator(); iter_t.isValid(); iter_t.advance())
            transform[iter_t.currentKey().toInt()] = iter_t.currentChild().getValue().toDouble();
          addMarker(name.toStdString(), frame.toStdString(), file.toStdString(), transform);
        }
      }
    }
  }

  void ObjTrackPanel::save(rviz_common::Config config) const
  {
    rviz_common::Panel::save(config);
    if (config.getType() == rviz_common::Config::Type::Map)
    {
      for (int i = 0; i < markers.size(); i++)
      {
        auto map = config.mapMakeChild(QString::fromStdString("$marker-" + std::to_string(i)));
        map.mapSetValue("name", QString::fromStdString(markers[i].name));
        map.mapSetValue("frame", QString::fromStdString(markers[i].frame));
        map.mapSetValue("file", QString::fromStdString(markers[i].file));
        auto transform = map.mapMakeChild("transform");
        for (int k = 0; k < 12; k++)
          transform.mapSetValue(QString::fromStdString(std::to_string(k)), markers[i].transform[k]);
      }
      config.mapSetValue("markerFrame", markerFrame->text());
    }
  }

  void ObjTrackPanel::onInitialize()
  {
    node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();
    node = node_ptr_->get_raw_node();
    server = std::make_unique<interactive_markers::InteractiveMarkerServer>("obj_track_ros_marker", node);
    track_obj_pub = node->create_publisher<msg::TrackedObject>("/tracker/objects", 10);
    track_control_pub = node->create_publisher<msg::TrackerControl>("/tracker/control", 10);
    marker_feedback_sub = node->create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>("obj_track_ros_marker/feedback", 10, std::bind(&ObjTrackPanel::onMarkerUpdate, this, _1));
  }
} // namespace obj_track_ros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(obj_track_ros::ObjTrackPanel, rviz_common::Panel)
