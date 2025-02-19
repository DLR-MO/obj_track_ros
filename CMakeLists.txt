cmake_minimum_required(VERSION 3.8)
project(obj_track_ros)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_MODULE_PATH, ${CMAKE_MODULE_PATH} /home/wiec_fa/.local)
include_directories(/home/wiec_fa/.local/include/opencv4)
include_directories(3DObjectTracking/third_party)

find_package(OpenCV 4.11.0 
REQUIRED COMPONENTS core imgproc highgui imgcodecs calib3d features2d xfeatures2d 
OPTIONAL_COMPONENTS cudafeatures2d
)

find_package(eigen3_cmake_module REQUIRED)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(USE_AZURE_KINECT "Use Azure Kinect" OFF)
option(USE_REALSENSE "Use RealSense D435" OFF)
option(USE_GTEST "Use gtest" OFF)
add_subdirectory(submodules/3DObjectTracking/M3T)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

set(SRC_FILES src/obj_track_ros_node.cpp)
set(INCLUDE_DIRS include submodules/3DObjectTracking/M3T/include)

add_executable(object_tracking_node ${SRC_FILES} )
target_include_directories(object_tracking_node PRIVATE ${INCLUDE_DIRS} )
target_link_libraries(object_tracking_node m3t)
set_target_properties(object_tracking_node PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(object_tracking_node PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

ament_target_dependencies(object_tracking_node rclcpp std_msgs sensor_msgs cv_bridge OpenCV image_transport)

install(TARGETS object_tracking_node RUNTIME DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# comment the line when a copyright and license is added to all source files
set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# comment the line when this package is in a git repo and when
# a copyright and license is added to all source files
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()

ament_package()

