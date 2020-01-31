//
// Created by jack on 1/3/20.
//

#ifndef CORRELATIVESCANMATCHER_H
#define CORRELATIVESCANMATCHER_H

#include <cstdint>
#include <cmath>
#include <memory>
#include <vector>
#include <mutex>
#include <boost/dynamic_bitset.hpp>
#include <glog/logging.h>
#include "string"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "Eigen/Dense"
#include "ros/ros.h"
#include "./CImg.h"

#define DEFAULT_GAUSSIAN_SIGMA 4
#define MIN_VALUE_FOR_LOOKUP 1E-12

using std::vector;
using Eigen::Vector2f;
using sensor_msgs::Image;
using cimg_library::CImg;

struct LookupTable {
  uint64_t width;
  uint64_t height;
  double resolution;
  CImg<double> values;
  LookupTable(const uint64_t range,
              const double resolution) :
              width((range * 2.0) / resolution + 1),
              height((range * 2.0) / resolution + 1),
              resolution(resolution) {
    // Construct a width x height image, with only 1 z level.
    // And, only one double per color with default value 0.0.
    values = CImg<double>(width, height, 1, 1, 0.0);
  }

  LookupTable() : width(0), height(0), resolution(1) {}

  inline double GetPointValue(Vector2f point) const {
    uint64_t x = width / 2 + round(point.x() / resolution);
    uint64_t y = height / 2 + round(point.y() / resolution);
    if (x >= width || y >= height) {
      return -1.0;
    }
    CHECK_LE(values(x,y), 1.0);
    if (values(x,y) <= MIN_VALUE_FOR_LOOKUP) {
      return MIN_VALUE_FOR_LOOKUP;
    }
    return values(x, y);
  }

  void SetPointValue(Vector2f point, double value) {
    uint64_t x = width / 2 + point.x() / resolution;
    uint64_t y = height / 2 + point.y() / resolution;
    if (x >= width || y >= height) {
      return;
    }
    values(x, y) = value;
  }

  void GaussianBlur(const double sigma) {
    values = values.blur(sigma, sigma, 0, true, true);
  }

  void GaussianBlur() {
    GaussianBlur(DEFAULT_GAUSSIAN_SIGMA);
  }

  CImg<double> GetDebugImage() const {
    return values;
  }

  double MaxArea(double start_x, double start_y, double end_x, double end_y) const {
    double max = 0.0;
    for (double x = start_x; x < end_x; x += resolution) {
      for (double y = start_y; y < end_y; y += resolution) {
        double temp = GetPointValue(Vector2f(x, y));
        if (temp > max) {
          max = temp;
        }
      }
    }
    return max;
  }

  // Converts the x and y into an absolute 1D coordinate.
  size_t AbsCoords(double x, double y) const {
    x -= std::fmod(x, resolution);
    y -= std::fmod(y, resolution);
    size_t row = ((height / 2) + round(y / resolution)) * width;
    size_t col = (width / 2) + round (x / resolution);
    CHECK_GE(row + col, 0);
    return row + col;
  }
};

class CorrelativeScanMatcher {
 public:
    CorrelativeScanMatcher(double scanner_range, double low_res, double high_res)
    : range_(scanner_range), low_res_(low_res), high_res_(high_res) {};
    std::pair<double, std::pair<Eigen::Vector2f, float>>
    GetTransformation(const vector<Vector2f>& pointcloud_a,
                      const vector<Vector2f>& pointcloud_b);
    std::pair<double, std::pair<Eigen::Vector2f, float>>
    GetTransformation(const vector<Vector2f>& pointcloud_a,
                      const vector<Vector2f>& pointcloud_b,
                      const double rotation_a,
                      const double rotation_b);
    Eigen::Matrix3f GetUncertaintyMatrix(const vector<Vector2f>& pointcloud_a,
                                         const vector<Vector2f>& pointcloud_b);
    Eigen::Matrix3f GetUncertaintyMatrix(const vector<Vector2f>& pointcloud_a,
                                         const vector<Vector2f>& pointcloud_b,
                                         double rotation_a,
                                         double rotation_b);
    LookupTable GetLookupTableHighRes(const vector<Vector2f>& pointcloud);
    LookupTable GetLookupTableLowRes(const LookupTable& high_res_table);
 private:
    LookupTable GetLookupTable(const vector<Vector2f>& pointcloud, double resolution);
    std::pair<double, std::pair<Eigen::Vector2f, float>> 
      GetProbAndTransformation(const vector<Vector2f>& pointcloud_a,
                               const LookupTable& pointcloud_b_cost,
                               double resolution,
                               double x_min,
                               double x_max,
                               double y_min, 
                               double y_max,
                               bool excluding,
                               const boost::dynamic_bitset<>& excluded);
    double range_;
    double low_res_;
    double high_res_;
};


#endif //LIDAR_SLAM_CORRELATIVESCANMATCHER_H
