cmake_minimum_required(VERSION 3.5)
project(xivo)

if(NOT DEFINED ENV{XIVO_ROOT})
  message(FATAL_ERROR "Environment variable XIVO_ROOT is not set.")
endif()

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
)

add_executable(xivo_ros
  src/simple_node.cpp
  src/main.cpp
)
list(APPEND deps
  xapp
  xest
  opencv_highgui
  opencv_features2d
  opencv_core
  opencv_video
  opencv_imgproc
  opencv_imgcodecs
  opencv_xfeatures2d
  glog
  pangolin
  pthread
  jsoncpp
  GL
  GLEW
  common
  gflags
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  stdc++fs
)
target_link_libraries(xivo_ros ${deps}) 
