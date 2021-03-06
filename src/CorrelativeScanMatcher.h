#ifndef SRC_CORRELATIVESCANMATCHER_H_
#define SRC_CORRELATIVESCANMATCHER_H_

#include <glog/logging.h>
#include <cmath>
#include <cstdint>

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <boost/dynamic_bitset.hpp>
#include "eigen3/Eigen/Dense"

#include "./CImg.h"

#define DEFAULT_GAUSSIAN_SIGMA 4
#define MIN_VALUE_FOR_LOOKUP 1E-10

using cimg_library::CImg;
using Eigen::Vector2f;
using std::pair;
using std::vector;

struct LookupTable {
  uint64_t width;
  uint64_t height;
  double resolution;
  CImg<double> values;
  LookupTable(const double range, const double resolution)
      : width(floor((range * 2.0) / resolution)),
        height(floor((range * 2.0) / resolution)),
        resolution(resolution) {
    // Construct a width x height image, with only 1 z level.
    // And, only one double per color with default value 0.0.
    values = CImg<double>(width, height, 1, 1, 0.0);
  }

  LookupTable() : width(0), height(0), resolution(1) {}

  inline uint64_t convertX(float x) const {
    return width / 2 + floor(x / resolution);
  }

  inline uint64_t convertY(float y) const {
    return height / 2 + floor(y / resolution);
  }

  inline double GetPointValue(Vector2f point) const {
    uint64_t x = convertX(point.x());
    uint64_t y = convertY(point.y());
    if (x >= width || y >= height || values(x, y) <= MIN_VALUE_FOR_LOOKUP) {
      return MIN_VALUE_FOR_LOOKUP;
    }
    CHECK_LE(values(x, y), 1.0);
    return values(x, y);
  }

  bool IsInside(Vector2f point) const {
    uint64_t x = convertX(point.x());
    uint64_t y = convertY(point.y());
    if (x >= width || y >= height) {
      return false;
    }
    return true;
  }

  void SetPointValue(Vector2f point, double value) {
    uint64_t x = convertX(point.x());
    uint64_t y = convertY(point.y());
    if (x >= width || y >= height) {
      return;
    }
    values(x, y) = value;
  }

  void normalize() { values = values.normalize(0, 1); }

  void GaussianBlur(const double sigma) {
    values = values.blur(sigma, sigma, 0, true, true);
  }

  void clear() { values = values.clear(); }

  void GaussianBlur() { GaussianBlur(DEFAULT_GAUSSIAN_SIGMA); }

  CImg<double> GetDebugImage() const { return values; }

  double MaxArea(double start_x, double start_y, double end_x,
                 double end_y) const {
    uint64_t sx = convertX(start_x);
    uint64_t sy = convertY(start_y);
    uint64_t ex = convertX(end_x);
    uint64_t ey = convertY(end_y);

    CImg<double> cropped = values.get_crop(sx, sy, ex - 1, ey - 1);
    const double max_val = cropped.max();
    return max_val;
  }

  // Converts the x and y into an absolute 1D coordinate.
  size_t AbsCoords(double x, double y) const {
    size_t row = ((height / 2) + round(y / resolution)) * width;
    size_t col = (width / 2) + round(x / resolution);
    CHECK_GE(row + col, 0);
    return row + col;
  }
};

class CorrelativeScanMatcher {
 public:
  CorrelativeScanMatcher(double scanner_range, double trans_range,
                         double low_res, double high_res)
      : range_(scanner_range),
        trans_range_(trans_range),
        low_res_(low_res),
        high_res_(high_res) {}
  pair<double, pair<Eigen::Vector2f, float>> GetTransformation(
      const vector<Vector2f>& pointcloud_a,
      const vector<Vector2f>& pointcloud_b, const double rotation_min = 0,
      const double rotation_max = 2 * M_PI);
  pair<double, pair<Eigen::Vector2f, float>> GetTransformation(
      const vector<Vector2f>& pointcloud_a,
      const vector<Vector2f>& pointcloud_b, const double rotation_a,
      const double rotation_b, const double rotation_restriction);
  Eigen::Matrix3f GetUncertaintyMatrix(const vector<Vector2f>& pointcloud_a,
                                       const vector<Vector2f>& pointcloud_b);
  Eigen::Matrix2f GetUncertaintyMatrix(const vector<Vector2f>& pointcloud_a,
                                       const vector<Vector2f>& pointcloud_b,
                                       const double rotation);
  // Computes the local uncertainty of the *last* scan in the provided vector of
  // point clouds, relative to the previous ones
  std::pair<double, double> GetLocalUncertaintyStats(
      const vector<vector<Vector2f>>& comparisonClouds,
      const vector<Vector2f> targetCloud);
  LookupTable GetLookupTableHighRes(const vector<Vector2f>& pointcloud);
  LookupTable GetLookupTableLowRes(const LookupTable& high_res_table);

 private:
  LookupTable GetLookupTable(const vector<Vector2f>& pointcloud,
                             double resolution);
  pair<double, pair<Eigen::Vector2f, float>> GetProbAndTransformation(
      const vector<Vector2f>& pointcloud_a,
      const LookupTable& pointcloud_b_cost, double resolution, double x_min,
      double x_max, double y_min, double y_max, double rotation);
  double range_;
  double trans_range_;
  double low_res_;
  double high_res_;
};

#endif  // SRC_CORRELATIVESCANMATCHER_H_
