#include <algorithm>
#include <csignal>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>

#include "CorrelativeScanMatcher.h"
#include "eigen3/Eigen/Dense"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "pointcloud_helpers.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

using Eigen::Vector2f;
using sensor_msgs::PointCloud2;
using std::string;
using std::vector;

#define DEBUG false

DEFINE_string(bag_file, "", "Bag file from which to read scans.");
DEFINE_string(lidar_topic, "/Cobot/Laser",
              "topic within bag file which to read scans.");
DEFINE_double(trans_range, 2,
              "The range of possible transformations to consider.");
DEFINE_double(laser_range, 30, "The maximum range of scans.");
DEFINE_bool(truncate_scan_angles, true,
            "If true, truncate angles of scans so we dont get artifacts at the "
            "ends from some scanners (default: true)");
DEFINE_bool(output_images, false,
            "If true, output bmp images in addition to statistics for each "
            "timing (default: false)");
DEFINE_double(window, 1.5,
              "The window (in time)  around each scan compare with, to compute "
              "avg uncertainty.");
DEFINE_uint64(
    comparisons, 10,
    "The number of nearby scans to compare with, to compute avg uncertainty.");
DEFINE_string(out_dir, "uncertainty_info",
              "folder in which to store images/uncertainty stats.");
DEFINE_uint64(base_clouds, 200,
              "The maximum number of base clouds to evaluate.");

#define MAX_WINDOW_DISTANCE 2000

void SignalHandler(int signum) {
  printf("Exiting with %d\n", signum);
  ros::shutdown();
  exit(0);
}

void bag_uncertainty_calc(const string bag_path, const unsigned int base_clouds,
                          const double window, const string out_dir) {
  printf("Loading bag file... ");
  std::vector<std::pair<double, std::vector<Vector2f>>> clouds;
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
  for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it) {
    const rosbag::MessageInstance& message = *it;
    {
      // Load all the point clouds into memory.
      sensor_msgs::LaserScanPtr laser_scan =
          message.instantiate<sensor_msgs::LaserScan>();
      if (laser_scan != nullptr) {
        double scan_time = (laser_scan->header.stamp).toSec();
        std::vector<Vector2f> cloud = pointcloud_helpers::LaserScanToPointCloud(
            *laser_scan, laser_scan->range_max, FLAGS_truncate_scan_angles);
        clouds.push_back(
            std::pair<double, std::vector<Vector2f>>(scan_time, cloud));
      }
    }
  }
  bag.close();
  printf("Done.\n");
  fflush(stdout);
  CorrelativeScanMatcher matcher(FLAGS_laser_range, FLAGS_trans_range, 0.3,
                                 0.03);
  const unsigned int skip_amt = floor(clouds.size() / base_clouds);

#if DEBUG
  std::cout << clouds.size() << std::endl;
  cimg_library::CImgDisplay display1;
#endif
  std::cout << "Processing 1 out of every " << skip_amt << " scans."
            << std::endl;
  // #endif

  std::pair<string, std::pair<double, double>> stats[base_clouds];
  unsigned int processed_count = 0;
#pragma omp parallel for shared(processed_count)
  for (unsigned int idx = 0; idx < base_clouds; idx++) {
    unsigned int i = idx * skip_amt;
    double baseTime = clouds[i].first;
    char timestamp[20];
    sprintf(timestamp, "%.5f", baseTime);
    std::vector<Vector2f> baseCloud = clouds[i].second;
    LookupTable high_res_lookup = matcher.GetLookupTableHighRes(baseCloud);
#if DEBUG
    display1.empty();
    display1.display(high_res_lookup.GetDebugImage().resize_doubleXY());
    printf("Processing Base Scan: %s\n", timestamp);
#endif
    if (FLAGS_output_images) {
      string filename = out_dir + "/" + "cloud_" + timestamp + ".bmp";
      high_res_lookup.GetDebugImage().normalize(0, 255).save_bmp(
          filename.c_str());
    }

    std::vector<int> comparisonIndices;
    // Find the list of "other" clouds within the base cloud's window.
    for (unsigned int j = 0; j < clouds.size(); j++) {
      if (baseTime - clouds[j].first < window &&
          baseTime - clouds[j].first > 0) {
        comparisonIndices.push_back(j);
      }
    }

    std::random_shuffle(comparisonIndices.begin(), comparisonIndices.end());

    comparisonIndices.resize(FLAGS_comparisons);

    std::vector<std::vector<Vector2f>> comparisonClouds;
    for (auto idx : comparisonIndices) {
      comparisonClouds.push_back(clouds[idx].second);
    }

    stats[idx] = std::make_pair(timestamp, matcher.GetLocalUncertaintyStats(
                                               comparisonClouds, baseCloud));

#if DEBUG
    std::cout << "Average Condition #: " << stats[idx].first << std::endl;
    std::cout << "Average Scale: " << stats[idx].second << std::endl;
#endif

    processed_count++;
    if (processed_count % 50 == 0) {
      std::cout << "Processed " << processed_count << " scans out of "
                << base_clouds << std::endl;
    }
  }

  string txt_filename = out_dir + "/local_uncertainty_stats.txt";
  std::ofstream stats_write(txt_filename.c_str());
  for (auto s : stats) {
    stats_write << s.first << ": " << s.second.first << ", " << s.second.second
                << std::endl;
  }
  stats_write.close();

  std::cout << "Processed " << base_clouds << " Base Scans." << std::endl;
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(*argv);
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Load and pre-process the data.
  if (FLAGS_bag_file.compare("") != 0 && FLAGS_lidar_topic.compare("") != 0 &&
      FLAGS_window != 0.0 && FLAGS_out_dir.compare("") != 0) {
    bag_uncertainty_calc(FLAGS_bag_file, FLAGS_base_clouds, FLAGS_window,
                         FLAGS_out_dir);
  } else {
    std::cout << "Must specify bag file, lidar topic, window, and output path!"
              << std::endl;
    exit(0);
  }
  return 0;
}
