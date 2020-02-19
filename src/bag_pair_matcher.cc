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
#include "Eigen/Dense"
#include "pointcloud_helpers.h"
#include "CorrelativeScanMatcher.h"

using std::string;
using std::vector;
using sensor_msgs::PointCloud2;
using namespace Eigen;

DEFINE_string(
  bag_file,
  "",
  "Bag file from which to read scans. Only works if scan_match_topic isn't provided.");
DEFINE_string(
  lidar_topic,
  "/Cobot/Laser",
  "topic within bag file which to read scans. Only works if scan_match_topic isn't provided.");
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
  base_timestamp,
  0.0,
  "The timestamp in the bag file for the base scan.");
DEFINE_double(
  match_timestamp,
  0.0,
  "The timestamp in the bag file for the scan to match.");
DEFINE_bool(
  calc_uncertainty,
  false,
  "Whether or not to calculate covariance matrix to illustrate uncertainty");

#define MAX_COMPARISONS 20

void SignalHandler(int signum) {
  printf("Exiting with %d\n", signum);
  ros::shutdown();
  exit(0);
}

// Given 2 scans calculate relative transformation & uncertainty
void scan_match_bag_file(string bag_path, string lidar_topic, double base_timestamp, double match_timestamp, bool truncate_scan_angles, bool calc_uncertainty) {
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
  topics.emplace_back(lidar_topic.c_str());
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
          baseCloud = pointcloud_helpers::LaserScanToPointCloud(*laser_scan, laser_scan->range_max, truncate_scan_angles);
        }
        if (abs(scan_time - match_timestamp) <= 1e-1) {
          printf("Found Match Scan %f\n", scan_time);
          matchCloud = pointcloud_helpers::LaserScanToPointCloud(*laser_scan, laser_scan->range_max, truncate_scan_angles);
        }
      }
    }
  }
  bag.close();
  printf("Done.\n");
  fflush(stdout);

  CorrelativeScanMatcher matcher(FLAGS_laser_range, FLAGS_trans_range, 0.3, 0.03);

  cimg_library::CImgDisplay display1;
  cimg_library::CImgDisplay display2;
  LookupTable high_res_lookup = matcher.GetLookupTableHighRes(baseCloud);
  LookupTable match_lookup = matcher.GetLookupTableHighRes(matchCloud);
  CImg<double> scanDisplay;
  scanDisplay.append(match_lookup.GetDebugImage());
  scanDisplay.append(high_res_lookup.GetDebugImage(), 'x');
  display1.display(scanDisplay);

  std::pair<double, std::pair<Vector2f, float>> matchResult = matcher.GetTransformation(baseCloud, matchCloud);
  fflush(stdout); 
  double prob = matchResult.first;
  std::pair<Vector2f, float> trans = matchResult.second;
  printf("recovered relative translation: (%f, %f), rotation: %f with score %f\n", trans.first.x(), trans.first.y(), trans.second, prob);
  std::pair<double, std::pair<Vector2f, float>> baseResult = matcher.GetTransformation(matchCloud, baseCloud);
  printf("recovered relative translation: (%f, %f), rotation: %f with score %f\n", baseResult.second.first.x(), baseResult.second.first.y(), baseResult.second.second, prob);

  printf("Visualizing results...\n");
  Affine2f transform = Translation2f(trans.first) * Rotation2Df(trans.second).toRotationMatrix();
  vector<Vector2f> baseTransformed;
  for (const Vector2f& point : baseCloud) {
    if (high_res_lookup.IsInside(point)) {
      baseTransformed.push_back(transform * point);
    }
  }

  Affine2f transform_match_affine = Translation2f(baseResult.second.first) * Rotation2Df(baseResult.second.second).toRotationMatrix();
  vector<Vector2f> matchTransformed;
  for (const Vector2f& point : matchCloud) {
    if (match_lookup.IsInside(point)) {
      matchTransformed.push_back(transform_match_affine * point);
    }
  }

  cimg_library::CImg<double> match_image = match_lookup.GetDebugImage();
  cimg_library::CImg<double> match_image_transformed = matcher.GetLookupTableHighRes(matchTransformed).GetDebugImage();
  cimg_library::CImg<double> base_image = high_res_lookup.GetDebugImage();
  cimg_library::CImg<double> base_image_transformed = matcher.GetLookupTableHighRes(baseTransformed).GetDebugImage();
  cimg_library::CImg<double> base_transformed_image(base_image.width(), base_image.height(), 1, 3, 0);
  cimg_library::CImg<double> match_transformed_image(base_image.width(), base_image.height(), 1, 3, 0);
  for (int x = 0; x < base_image.width(); x++) {
    for (int y = 0; y < base_image.height(); y++) {
      base_transformed_image(x, y, 0, 2) = base_image_transformed(x, y);
      base_transformed_image(x, y, 0, 1) = match_image(x, y);
      match_transformed_image(x, y, 0, 2) = base_image(x, y);
      match_transformed_image(x, y, 0, 1) = match_image_transformed(x, y);
    }
  }

  // Try uncertainty stuff
  if (calc_uncertainty) {
    printf("Calculating uncertainty matrix...\n");
    Matrix2f uncertainty = matcher.GetUncertaintyMatrix(baseCloud, matchCloud, trans.second);
    EigenSolver<Matrix2f> es(uncertainty);
    Matrix2f eigenvectors = es.eigenvectors().real();
    Vector2f eigenvalues = es.eigenvalues().real();
    std::cout << "Values:" << eigenvalues << std:: endl;
    std::cout << "Vectors:" << eigenvectors << std:: endl;

    CImg<unsigned char> uncertainty_img(base_image_transformed.width(),base_image_transformed.height(), 1, 3, 0);
    const unsigned char color[] = { 0, 0, 255 };
    uncertainty_img.draw_ellipse(uncertainty_img.width()/2, uncertainty_img.height()/2, eigenvalues[0] * 50, eigenvalues[1] * 50, atan(eigenvectors.col(0)[1] / eigenvectors.col(0)[0]), color, 1.0);
    std::cout << "Condition #: " << eigenvalues.maxCoeff() / eigenvalues.minCoeff() << std::endl;
    std::cout << "Maximum scale: " << eigenvalues.maxCoeff() << std::endl;
    display2.display(uncertainty_img);
    base_image_transformed.draw_image(0, 0, uncertainty_img);
  }
  CImg<double> resultImg;
  resultImg.append(match_transformed_image);
  resultImg.append(base_transformed_image, 'x');

  scanDisplay.append(resultImg, 'y');
  display1.display(scanDisplay);

  // Wait for the windows to close
  while (!display1.is_closed()) {
    display1.wait();
  }
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(*argv);
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Load and pre-process the data.
  if (FLAGS_bag_file.compare("") != 0 && FLAGS_lidar_topic.compare("") != 0 && FLAGS_base_timestamp != 0.0 && FLAGS_match_timestamp != 0.0) {
    scan_match_bag_file(FLAGS_bag_file, FLAGS_lidar_topic, FLAGS_base_timestamp, FLAGS_match_timestamp, FLAGS_truncate_scan_angles, FLAGS_calc_uncertainty);
  } else {
    std::cout << "Must specify bag file, lidar topic & timestamps" << std::endl;
    exit(0);
  }
  return 0;
}
