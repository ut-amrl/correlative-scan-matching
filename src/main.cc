#include <csignal>
#include <vector>

#include "ros/node_handle.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "CorrelativeScanMatcher/CorrScanMatchInputMsg.h"
#include "Eigen/Dense"
#include "pointcloud_helpers.h"

using std::string;
using std::vector;
using CorrelativeScanMatcher::CorrScanMatchInputMsgConstPtr;
using CorrelativeScanMatcher::CorrScanMatchInputMsg;
using sensor_msgs::PointCloud2;
using Eigen::Vector2f;

DEFINE_string(
  scan_match_topic,
  "/corr_scan_match",
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
  printf("Successfully converted point clouds");
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(*argv);
  google::ParseCommandLineFlags(&argc, &argv, false);
  ros::init(argc, argv, "correlative_scan_matcher");
  ros::NodeHandle n;
  signal(SIGINT, SignalHandler);
  // Load and pre-process the data.
  
  std::cout << "Waiting for Point Cloud input" << std::endl;
  ros::Subscriber hitl_sub = n.subscribe(FLAGS_scan_match_topic, 10, corr_scan_match_callback);
  ros::spin();
  return 0;
}
