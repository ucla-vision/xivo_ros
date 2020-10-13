// 3rdparty
#include "ros/ros.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
// xivo
#include "xivo_ros/simple_node.h"

using namespace xivo;

int main(int argc, char ** argv)
{
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "xivo");

  xivo_ros::SimpleNode node;

  while(ros::ok()) {
    // node.HandleQueue();
    if(ros::ok()) {
      ros::spinOnce();
    }
  }
}
