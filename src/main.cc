#include <csignal>
#include <vector>

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
  printf("Bag file has %d scans\n", view.size());
  // Iterate through the bag
  for (rosbag::View::iterator it = view.begin();
       it != view.end();
       ++it) {
    const rosbag::MessageInstance &message = *it;
    {
      // Load all the point clouds into memory.
      sensor_msgs::LaserScanPtr laser_scan =
              message.instantiate<sensor_msgs::LaserScan>();
      if (laser_scan != nullptr) {
        
        // Process the laser scan
        // check if the timestamp lines up
        double scan_time = (laser_scan->header.stamp - view.getBeginTime()).toSec();
        if (abs(scan_time - base_timestamp) <= 1e-2) {
          printf("Found Base Scan %f\n", scan_time);
          baseCloud = pointcloud_helpers::LaserScanToPointCloud(*laser_scan, laser_scan->range_max);
        }
        if (abs(scan_time - match_timestamp) <= 1e-2) {
          printf("Found Match Scan %f\n", scan_time);
          matchCloud = pointcloud_helpers::LaserScanToPointCloud(*laser_scan, laser_scan->range_max);
        }

        // printf("%f\n", scan_time);
      }
    }
  }
  bag.close();
  printf("Done.\n");
  fflush(stdout);

  CorrelativeScanMatcher matcher(4, 0.3, .01);
  std::pair<double, std::pair<Eigen::Vector2f, float>> matchResult = matcher.GetTransformation(baseCloud, matchCloud);
  fflush(stdout); 
  double prob = matchResult.first;
  std::pair<Eigen::Vector2f, float> trans = matchResult.second;
  printf("Recovered Relative Translation: (%f, %f), Rotation: %f with score %f\n", trans.first.x(), trans.first.y(), trans.second, prob);
  
  // Try uncertainty stuff
  Eigen::Matrix3f uncertainty = matcher.GetUncertaintyMatrix(baseCloud, matchCloud);
  std::cout << "Uncertainty Matrix:" << uncertainty << std::endl;

  std::cout << "Eigenvalues: " << uncertainty.eigenvalues() << std::endl;
  
  Eigen::Vector3cf eigenvalues = uncertainty.eigenvalues();
  std::vector<float> eigens{eigenvalues[0].real(), eigenvalues[1].real(), eigenvalues[2].real()};
  std::sort(std::begin(eigens), std::end(eigens));
  std::cout << "Condition #: " << eigens[2] / eigens[0] << std::endl;
  std::cout << "Maximum scale: " << eigens[2] << std::endl;
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
  if (FLAGS_bag_file.compare("") != 0 && FLAGS_lidar_topic.compare("") != 0 && FLAGS_base_timestamp != 0.0 && FLAGS_match_timestamp != 0.0) {
    scan_match_bag_file(FLAGS_bag_file.c_str(), FLAGS_base_timestamp, FLAGS_match_timestamp);
  } else if(FLAGS_scan_match_topic.compare("") != 0) {
    ros::init(argc, argv, "correlative_scan_matcher");
    ros::NodeHandle n;
    std::cout << "Waiting for Point Cloud input" << std::endl;
    ros::Subscriber hitl_sub = n.subscribe(FLAGS_scan_match_topic, 10, corr_scan_match_callback);
    ros::spin();
  } else {
    std::cout << "Must specify bag file and timestamps, or scan match topic!" << std::endl;
    exit(0);
  }
  return 0;
}
