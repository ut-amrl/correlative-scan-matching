#include <csignal>
#include <vector>

#include "ros/node_handle.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
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
  "",
  "The topic that point clouds to match will be published over.");
DEFINE_string(
  bag_file,
  "",
  "Bag file from which to read scans. Only works if scan_match_topic isn't provided.");
DEFINE_string(
  lidar_topic,
  "/Cobot/Laser",
  "topic within bag file which to read scans. Only works if scan_match_topic isn't provided.");
DEFINE_double(
  base_timestamp,
  0.0,
  "The timestamp in the bag file for the base scan.");
DEFINE_double(
  match_timestamp,
  0.0,
  "The timestamp in the bag file for the scan to match.");

void SignalHandler(int signum) {
  printf("Exiting with %d\n", signum);
  ros::shutdown();
  exit(0);
}

void scan_match_bag_file(string bag_path, double base_timestamp, double match_timestamp) {
  printf("Loading bag file... ");
  std::vector<Vector2f> baseCloud;
  std::vector<Vector2f> matchCloud;
  fflush(stdout);
  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException& exception) {
    printf("Unable to read %s, reason %s:", bag_path.c_str(), exception.what());
    return;
  }
  // Get the topics we want
  vector<string> topics;
  topics.emplace_back(FLAGS_lidar_topic.c_str());
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  // Iterate through the bag
  for (rosbag::View::iterator it = view.begin();
       ros::ok() && it != view.end();
       ++it) {
    const rosbag::MessageInstance &message = *it;
    {
      // Load all the point clouds into memory.
      sensor_msgs::LaserScanPtr laser_scan =
              message.instantiate<sensor_msgs::LaserScan>();
      if (laser_scan != nullptr) {
        // Process the laser scan
        // check if the timestamp lines up
        double scan_time = laser_scan->header.stamp.sec + laser_scan->header.stamp.nsec * 1e-9;

        if (scan_time == base_timestamp) {
          baseCloud = pointcloud_helpers::LaserScanToPointCloud(*laser_scan, laser_scan->range_max);
        } else if (scan_time == match_timestamp) {
          matchCloud = pointcloud_helpers::LaserScanToPointCloud(*laser_scan, laser_scan->range_max);
        }
      }
    }
  }
  bag.close();
  printf("Done.\n");
  fflush(stdout);

  printf("Successfully converted point clouds");
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
  if (FLAGS_bag_file.compare("") != 0 && FLAGS_base_timestamp != 0.0 && FLAGS_match_timestamp != 0.0) {
    scan_match_bag_file(FLAGS_bag_file.c_str(), FLAGS_base_timestamp, FLAGS_match_timestamp);
  } else if(FLAGS_scan_match_topic.compare("") != 0) {
    std::cout << "Waiting for Point Cloud input" << std::endl;
    ros::Subscriber hitl_sub = n.subscribe(FLAGS_scan_match_topic, 10, corr_scan_match_callback);
    ros::spin();
  } else {
    std::cout << "Must specify bag file and timestamps, or scan match topic!";
    exit(0);
  }
  return 0;
}
