#include <csignal>
#include <vector>

#include "ros/node_handle.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

using std::string;
using std::vector;

DEFINE_string(
  scan_match_topic,
  "/velodyne_2dscan_high_beams",
  "The topic that point clouds to match will be published over.");

void SignalHandler(int signum) {
  printf("Exiting with %d\n", signum);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(*argv);
  google::ParseCommandLineFlags(&argc, &argv, false);
  ros::init(argc, argv, "correlative_scan_matcher");
  ros::NodeHandle n;
  signal(SIGINT, SignalHandler);
  // Load and pre-process the data.
  
  std::cout << "Waiting for Point Cloud input" << std::endl;
  // ros::Subscriber hitl_sub = n.subscribe(FLAGS_hitl_lc_topic,
  //                                        10,
  //                                        &Solver::HitlCallback,
  //                                        &solver);
  ros::spin();
  return 0;
}
