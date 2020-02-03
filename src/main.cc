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
DEFINE_string(
  bag_file,
  "",
  "Bag file from which to read scans. Only works if scan_match_topic isn't provided.");
DEFINE_string(
  lidar_topic,
  "/Cobot/Laser",
  "topic within bag file which to read scans. Only works if scan_match_topic isn't provided.");
DEFINE_bool(
  truncate_scan_angles,
  true,
  "If true, truncate angles of scans so we dont get artifacts at the ends from some scanners (default: true)");
DEFINE_double(
  base_timestamp,
  0.0,
  "The timestamp in the bag file for the base scan.");
DEFINE_double(
  match_timestamp,
  0.0,
  "The timestamp in the bag file for the scan to match.");
DEFINE_double(
  window,
  0.0,
  "The window around the base scan to search for matches, to compute avg uncertainty.");
DEFINE_string(
  out_dir,
  "uncertainty_info",
  "folder in which to store images/uncertainty stats.");

#define MAX_COMPARISONS 20

void SignalHandler(int signum) {
  printf("Exiting with %d\n", signum);
  ros::shutdown();
  exit(0);
}

// Given 2 scans calculate relative transformation & uncertainty
void scan_match_bag_file(string bag_path, double base_timestamp, double match_timestamp) {
  printf("Loading bag file... ");
  std::vector<Vector2f> baseCloud;
  std::vector<Vector2f> matchCloud;

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
        if (abs(scan_time - base_timestamp) <= 1e-1) {
          printf("Found Base Scan %f\n", scan_time);
          baseCloud = pointcloud_helpers::LaserScanToPointCloud(*laser_scan, laser_scan->range_max, FLAGS_truncate_scan_angles);
        }
        if (abs(scan_time - match_timestamp) <= 1e-1) {
          printf("Found Match Scan %f\n", scan_time);
          matchCloud = pointcloud_helpers::LaserScanToPointCloud(*laser_scan, laser_scan->range_max, FLAGS_truncate_scan_angles);
        }
      }
    }
  }
  bag.close();
  printf("Done.\n");
  fflush(stdout);

  CorrelativeScanMatcher matcher(4, 0.3, 0.03);

  cimg_library::CImgDisplay display1;
  cimg_library::CImgDisplay display2;
  LookupTable high_res_lookup = matcher.GetLookupTableHighRes(baseCloud);
  LookupTable match_lookup = matcher.GetLookupTableHighRes(matchCloud);
  display1.display(match_lookup.GetDebugImage());
  display2.display(high_res_lookup.GetDebugImage().resize_doubleXY());
  
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

  Eigen::Affine2f transform = Eigen::Translation2f(trans.first) * Eigen::Rotation2Df(trans.second).toRotationMatrix();

  vector<Vector2f> baseTransformed;
  for (const Vector2f& point : baseCloud) {
    if (high_res_lookup.IsInside(point)) {
      baseTransformed.push_back(transform * point);
    }
  }

  cimg_library::CImgDisplay display3;
  cimg_library::CImg<double> match_image = match_lookup.GetDebugImage();
  cimg_library::CImg<double> base_image = matcher.GetLookupTableHighRes(baseTransformed).GetDebugImage();
  cimg_library::CImg<double> transform_image(base_image.width(), base_image.height(), 1, 3);
  for (int x = 0; x < base_image.width(); x++) {
    for (int y = 0; y < base_image.height(); y++) {
      transform_image(x, y, 0, 2) = base_image(x, y);
      transform_image(x, y, 0, 1) = match_image(x, y);
    }
  }
  display3.display(transform_image);

  // Wait for the windows to close
  while (!display1.is_closed() && !display2.is_closed()) {
    display1.wait();
  }
}

// Given a base timestamp and a window, find the nearby scans to this base timesetamp and calculate avg uncertainty
void scan_window_bag_file(string bag_path, double base_timestamp, double window) {
  printf("Loading bag file... ");
  std::vector<Vector2f> baseCloud;
  std::vector<std::vector<Vector2f>> matchClouds;
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
          baseCloud = pointcloud_helpers::LaserScanToPointCloud(*laser_scan, laser_scan->range_max, FLAGS_truncate_scan_angles);
        }else if (abs(scan_time - base_timestamp) <= window) {
          printf("Found Match Scan %f\n", scan_time);
          matchClouds.push_back(pointcloud_helpers::LaserScanToPointCloud(*laser_scan, laser_scan->range_max, FLAGS_truncate_scan_angles));
        }
      }
    }
  }
  bag.close();
  printf("Done.\n");
  fflush(stdout);
  CorrelativeScanMatcher matcher(4, 0.3, 0.03);

  cimg_library::CImgDisplay display1;
  LookupTable high_res_lookup = matcher.GetLookupTableHighRes(baseCloud);
  display1.display(high_res_lookup.GetDebugImage().resize_doubleXY());

  double condition_avg = 0.0;
  double scale_avg = 0.0;
  for (auto cloud : matchClouds) {
    Eigen::Matrix3f uncertainty = matcher.GetUncertaintyMatrix(baseCloud, cloud);
    Eigen::Vector3cf eigenvalues = uncertainty.eigenvalues();
    std::vector<float> eigens{eigenvalues[0].real(), eigenvalues[1].real(), eigenvalues[2].real()};
    std::sort(std::begin(eigens), std::end(eigens));
    std::cout << "Condition #: " << eigens[2] / eigens[0] << std::endl;
    std::cout << "Maximum scale: " << eigens[2] << std::endl;
    condition_avg += eigens[2] / eigens[0];
    scale_avg += eigens[2];
  }

  condition_avg /= matchClouds.size();
  scale_avg /= matchClouds.size();

  std::cout << "Average Condition #: " << condition_avg << std::endl;
  std::cout << "Average Scale: " << scale_avg << std::endl;

  // Wait for the windows to close
  while (!display1.is_closed()) {
    display1.wait();
  }
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
        double scan_time = (laser_scan->header.stamp - view.getBeginTime()).toSec();
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
  CorrelativeScanMatcher matcher(4, 0.3, 0.03);
  std::cout << baseClouds.size() << std::endl;
  cimg_library::CImgDisplay display1;
  for (unsigned int i = 1; i < baseClouds.size(); i+=1) {
    double baseTime = baseClouds[i].first;
    double condition_avg = 0.0;
    double scale_avg = 0.0;
    std::vector<Vector2f> baseCloud = baseClouds[i].second;
    LookupTable high_res_lookup = matcher.GetLookupTableHighRes(baseCloud);
    display1.empty();
    display1.display(high_res_lookup.GetDebugImage().resize_doubleXY());
    string filename = out_dir + "/" + "cloud_" + std::to_string(baseTime) + ".jpeg";
    high_res_lookup.GetDebugImage().normalize(0, 255).save_jpeg(filename.c_str());

    std::vector<int> comparisonIndices;
    // Find the list of "other" clouds within the base cloud's window.
    for (unsigned int j = 0; j <= matchClouds.size(); j++) {
      if (abs(matchClouds[j].first - baseTime) < window) {
        comparisonIndices.push_back(j);
      }
    }

    std::random_shuffle(comparisonIndices.begin(), comparisonIndices.end());

    comparisonIndices.resize(MAX_COMPARISONS);

    std::cout << "Comparisons: " << comparisonIndices.size() << std::endl;

    for(auto idx : comparisonIndices) {
      std::vector<Vector2f> cloud = matchClouds[idx].second;
      Eigen::Matrix3f uncertainty = matcher.GetUncertaintyMatrix(baseCloud, cloud);;
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

    filename = out_dir + "/" + "stats_" + std::to_string(baseTime) + ".txt";
    std::ofstream stats_write(filename.c_str());
    stats_write << condition_avg << std::endl;
    stats_write << scale_avg << std::endl;
    stats_write.close();
  }
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
  if (FLAGS_bag_file.compare("") != 0 && FLAGS_lidar_topic.compare("") != 0 && FLAGS_base_timestamp != 0.0) {
    if (FLAGS_match_timestamp != 0) {
      scan_match_bag_file(FLAGS_bag_file.c_str(), FLAGS_base_timestamp, FLAGS_match_timestamp);
    } else if (FLAGS_window != 0) {
      scan_window_bag_file(FLAGS_bag_file.c_str(), FLAGS_base_timestamp, FLAGS_window);
    }
  } else if (FLAGS_bag_file.compare("") != 0 && FLAGS_lidar_topic.compare("") != 0 && FLAGS_window != 0.0 && FLAGS_out_dir.compare("") != 0) {
    bag_uncertainty_calc(FLAGS_bag_file, FLAGS_window, FLAGS_out_dir);
  } else if(FLAGS_scan_match_topic.compare("") != 0) {
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
