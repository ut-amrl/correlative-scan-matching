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

DEFINE_string(
  bag_file,
  "",
  "Bag file from which to read scans.");
DEFINE_string(
  lidar_topic,
  "/Cobot/Laser",
  "topic within bag file which to read scans.");
DEFINE_double(
  trans_range,
  2,
  "The range of possible transformations to consider.");
DEFINE_double(
  laser_range,
  30,
  "The maximum range of scans.");
DEFINE_bool(
  truncate_scan_angles,
  true,
  "If true, truncate angles of scans so we dont get artifacts at the ends from some scanners (default: true)");
DEFINE_double(
  window,
  1.5,
  "The window (in time)  around each scan compare with, to compute avg uncertainty.");
DEFINE_uint64(
  comparisons,
  10,
  "The number of nearby scans to compare with, to compute avg uncertainty.");
DEFINE_string(
  out_dir,
  "uncertainty_info",
  "folder in which to store images/uncertainty stats.");


void SignalHandler(int signum) {
  printf("Exiting with %d\n", signum);
  ros::shutdown();
  exit(0);
}

void bag_uncertainty_calc(string bag_path, double window, string out_dir) {
  printf("Loading bag file... ");
  std::vector<std::pair<double, std::vector<Vector2f>>> baseClouds;
  std::vector<std::pair<double, std::vector<Vector2f>>> matchClouds;
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
  printf("Bag Start time: %f\n", view.getBeginTime().toSec());
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

        // Process the laser scan. Here we take all "even" seconds as base scans
        double scan_time = (laser_scan->header.stamp).toSec();
        if (scan_time > window && scan_time - floor(scan_time) < 1e-2 && int(floor(scan_time)) % 2 == 0) {
          printf("Found Base Scan %f\n", scan_time);
          std::vector<Vector2f> cloud = pointcloud_helpers::LaserScanToPointCloud(*laser_scan, laser_scan->range_max, FLAGS_truncate_scan_angles);
          baseClouds.push_back(std::pair<double, std::vector<Vector2f>>(scan_time, cloud));
        } else {
          std::vector<Vector2f> cloud = pointcloud_helpers::LaserScanToPointCloud(*laser_scan, laser_scan->range_max, FLAGS_truncate_scan_angles);
          matchClouds.push_back(std::pair<double, std::vector<Vector2f>>(scan_time, cloud));
        }
      }
    }
  }
  bag.close();
  printf("Done.\n");
  fflush(stdout);
  CorrelativeScanMatcher matcher(FLAGS_laser_range, FLAGS_trans_range, 0.3, 0.03);
  std::cout << baseClouds.size() << std::endl;
  cimg_library::CImgDisplay display1;
  for (unsigned int i = 1; i < baseClouds.size(); i+=1) {
    double baseTime = baseClouds[i].first;
    char timestamp[20];
    sprintf(timestamp, "%.5f", baseTime);
    double condition_avg = 0.0;
    double scale_avg = 0.0;
    std::vector<Vector2f> baseCloud = baseClouds[i].second;
    LookupTable high_res_lookup = matcher.GetLookupTableHighRes(baseCloud);
    display1.empty();
    display1.display(high_res_lookup.GetDebugImage().resize_doubleXY());
    string filename = out_dir + "/" + "cloud_" + timestamp + ".bmp";
    high_res_lookup.GetDebugImage().normalize(0, 255).save_bmp(filename.c_str());

    std::vector<int> comparisonIndices;
    // Find the list of "other" clouds within the base cloud's window.
    for (unsigned int j = 0; j <= matchClouds.size(); j++) {
      if (abs(matchClouds[j].first - baseTime) < window) {
        comparisonIndices.push_back(j);
      }
    }

    std::random_shuffle(comparisonIndices.begin(), comparisonIndices.end());

    comparisonIndices.resize(FLAGS_comparisons);

    std::cout << "Comparisons: " << comparisonIndices.size() << std::endl;

    for(auto idx : comparisonIndices) {
      std::vector<Vector2f> cloud = matchClouds[idx].second;
      Eigen::Matrix3f uncertainty = matcher.GetUncertaintyMatrix(baseCloud, cloud);
      Eigen::Vector3cf eigenvalues = uncertainty.eigenvalues();
      std::vector<float> eigens{eigenvalues[0].real(), eigenvalues[1].real(), eigenvalues[2].real()};
      std::sort(std::begin(eigens), std::end(eigens));
      condition_avg += eigens[2] / eigens[0];
      scale_avg += eigens[2];
    }

    condition_avg /= comparisonIndices.size();
    scale_avg /= comparisonIndices.size();

    std::cout << "Average Condition #: " << condition_avg << std::endl;
    std::cout << "Average Scale: " << scale_avg << std::endl;

    filename = out_dir + "/" + "stats_" + timestamp + ".txt";
    std::ofstream stats_write(filename.c_str());
    stats_write << condition_avg << std::endl;
    stats_write << scale_avg << std::endl;
    stats_write.close();
  }
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(*argv);
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Load and pre-process the data.
  if (FLAGS_bag_file.compare("") != 0 && FLAGS_lidar_topic.compare("") != 0 && FLAGS_window != 0.0 && FLAGS_out_dir.compare("") != 0) {
    bag_uncertainty_calc(FLAGS_bag_file, FLAGS_window, FLAGS_out_dir);
  } else {
    std::cout << "Must specify bag file, lidar topic, window, and output path!" << std::endl;
    exit(0);
  }
  return 0;
}
