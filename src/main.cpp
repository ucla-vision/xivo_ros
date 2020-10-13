// 3rdparty
#include "ros/ros.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
// xivo
#include "xivo/simple_node.h"


int main(int argc, char ** argv)
{
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "xivo");

  xivo::SimpleNode node;

  while(ros::ok()) {
    // node.HandleQueue();
    if(ros::ok()) {
      ros::spinOnce();
    }
  }
}
