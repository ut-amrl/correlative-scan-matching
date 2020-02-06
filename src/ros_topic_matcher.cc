#include <csignal>
#include <vector>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>

#include "ros/node_handle.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "CorrelativeScanMatching/CorrScanMatchInputMsg.h"
#include "Eigen/Dense"
#include "pointcloud_helpers.h"
#include "CorrelativeScanMatcher.h"

using std::string;
using std::vector;
using CorrelativeScanMatching::CorrScanMatchInputMsgConstPtr;
using CorrelativeScanMatching::CorrScanMatchInputMsg;
using sensor_msgs::PointCloud2;
using Eigen::Vector2f;

/*
This allows you to either provide a scan_match_topic, in which case it functions as a ros node listening for pairs of scans on that topic.
Alternatively, you can provide a bag file, a lidar_topic, and a pair of timestamps, and it will do scan matching for that pair and then terminate.
*/
DEFINE_string(
  scan_match_topic,
  "",
  "The topic that point clouds to match will be published over.");

void SignalHandler(int signum) {
  printf("Exiting with %d\n", signum);
  ros::shutdown();
  exit(0);
}

void corr_scan_match_callback(const CorrScanMatchInputMsgConstPtr& msg_ptr) {
  const CorrScanMatchInputMsg msg = *msg_ptr;
  PointCloud2 base = msg.base_cloud;
  PointCloud2 match = msg.match_cloud;
  vector<Vector2f> baseCloud = pointcloud_helpers::RosCloudToPointCloud(base);
  vector<Vector2f> matchCloud = pointcloud_helpers::RosCloudToPointCloud(match);
  printf("Successfully converted point clouds\n");
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(*argv);
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Load and pre-process the data.
  if (FLAGS_scan_match_topic.compare("") != 0) {
    ros::init(argc, argv, "correlative_scan_matcher");
    ros::NodeHandle n;
    std::cout << "Waiting for Point Cloud input" << std::endl;
    ros::Subscriber hitl_sub = n.subscribe(FLAGS_scan_match_topic, 10, corr_scan_match_callback);
    ros::spin();
  } else {
    std::cout << "Must specify bag file & lidar topic along, or scan match topic!" << std::endl;
    exit(0);
  }
  return 0;
}
