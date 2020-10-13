cmake_minimum_required(VERSION 3.5)
project(xivo_ros)

add_compile_options(-std=c++17)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(roscpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  std_msgs
  image_transport
  message_generation
  sensor_msgs
  cv_bridge
  tf
  nav_msgs
  roscpp
  pcl_ros
  geometry_msgs
)


add_message_files(
  FILES
  FeatureData.msg
  FeatureMap.msg
  FullState.msg
  MotionState2dNav.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)


# Declare the message files to be built
catkin_package(CATKIN_DEPENDS
  dynamic_reconfigure
  std_msgs
  message_runtime
  image_transport
  sensor_msgs
  tf
  cv_bridge
  nav_msgs
  geometry_msgs
  roscpp
  CATKIN_DEPENDS
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(
  $ENV{XIVO_ROOT}/common
  $ENV{XIVO_ROOT}/src
  $ENV{XIVO_ROOT}/thirdparty/eigen-3.3.7/include/eigen3
  $ENV{XIVO_ROOT}/thirdparty/jsoncpp/include
  $ENV{XIVO_ROOT}/thirdparty/Pangolin/include
  ${PROJECT_SOURCE_DIR}/include
  ${CATKIN_DEVEL_PREFIX}/include
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV REQUIRED)

link_directories(
  $ENV{XIVO_ROOT}/lib
  $ENV{XIVO_ROOT}/thirdparty/gflags/lib
  $ENV{XIVO_ROOT}/thirdparty/glog/lib
  $ENV{XIVO_ROOT}/thirdparty/jsoncpp/lib
  $ENV{XIVO_ROOT}/thirdparty/Pangolin/lib
  /opt/ros/$ENV{ROS_DISTRO}/lib/x86_64-linux-gnu
  /usr/lib/x86_64-linux-gnu
)


set(libxivo xest xapp)
#if (BUILD_G2O)
#  list(APPEND libxivo xopt)
#endif(BUILD_G2O)


add_executable(mynode
  src/simple_node.cpp
  src/main.cpp
  $ENV{XIVO_ROOT}/src/factory.cpp
)
target_link_libraries(mynode
  ${libxivo}
  common
  gflags
  glog
  jsoncpp
  opencv_core
  opencv_imgcodecs
  opencv_highgui
  opencv_xfeatures2d
  pangolin
  GL
  GLEW
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  stdc++fs)
