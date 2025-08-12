# obj_track_ros

Enables running [3DObjectTracking](https://github.com/DLR-RM/3DObjectTracking) with ROS 2 camera feeds.
With this package you can continuously track the poses of rigid objects specified as mesh geometries.

The ROS 2 node is configured at launch to subscribe to one or multiple camera feeds.
Tracked objects are registered by publishing [TrackedObject](./msg/TrackedObject.msg) messages to the node at runtime.
The results can be observed by subscribing to the [TrackedResult](./msg/TrackedResult.msg) messages, or visualized as overlay in RViz.

![Rviz screenshot showing two images with the tracked object as overlay and the estimated pose being published as tf frame.](img/rviz_screenshot_1.png)

## Installation

This package is currently only provided as source build, and requires the following dependencies to be provided:

* **ROS 2** Jazzy (or newer) with configured rosdeps
* **OpenCV** including the modules:
  * core
  * imgproc
  * highgui
  * imgcodecs
  * calib3d
  * features2d
  * **xfeatures2d** (not installed in debian distribution)

#### Build OpenCV

If you're running on Ubuntu you likely need to compile OpenCV yourself as debian does not ship the module xfeatures2d (from opencv_contrib).
Detailed instructions for a build install of OpenCV can be found at https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html.

This condensed version of the install instructions should work on most systems (run from anywhere outside the colcon workspace):

```bash
# Install minimal prerequisites
sudo apt update && sudo apt install -y cmake g++ wget unzip

# Download and unpack sources
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip
unzip opencv.zip
unzip opencv_contrib.zip

# Create build directory and switch into it
mkdir -p build && cd build

# Configure, including xfeatures2d
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/home/$USER/.local/ -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules -DBUILD_opencv_xfeatures2d=ON ../opencv-4.x

# Build and install
make -j8
make install
```

### Build this package

Clone this repository _recursively_ into a colcon workspace.

```git clone --recursive https://gitlab.dlr.de/mo-repo/rar/obj_track_ros.git```

Install all ROS dependencies using rosdep.

```rosdep install --from-path ./obj_track_ros --ignore-src --rosdistro $ROS_DISTRO```

Finally, build the workspace with `colcon build --symlink-install` as usual.

## Getting Started

See in `/launch` and `/config` for a concrete working example.

The yaml config defines a list of rgb or depth cameras used by the tracking node:

```yaml
- name: color_camera
  type: Ros2ColorCamera
  image_topic: /your_camera_image_topic
  info_topic: /your_camera_info_topic
  frame: the_cameras_frame
  publish_overlay: true

- name: depth_camera
  type: Ros2DepthCamera
  image_topic: /depth_image_topic
  info_topic: /depth_camera_info
  frame: the_cameras_frame
  scale: 1.0
```

During runtime, you can then send `[TrackedObject](./msg/TrackedObject.msg)` messages to start tracking specific objects.

```text
string name
string geometry_path
string frame

float32 geometry_unit_in_meter 1.0
bool geometry_counterclockwise true
bool geometry_enable_culling false
float32[16] geometry2body_pose [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 ]
float32[16] detector_world_pose [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.05, 0.0, 1.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0 ]
```
