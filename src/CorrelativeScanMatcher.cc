//
// Created by jack on 1/3/20.
//

#include <vector>
#include <algorithm>

#include <boost/dynamic_bitset.hpp>
#include "Eigen/Dense"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "CorrelativeScanMatcher.h"
#include "./CImg.h"
#include "./pointcloud_helpers.h"


#define UNCERTAINTY_USELESS_THRESHOLD 0.01

using std::vector;
using Eigen::Vector2f;

using sensor_msgs::PointCloud2;

LookupTable
CorrelativeScanMatcher::GetLookupTable(const vector<Vector2f>& pointcloud,
                                       double resolution) {
  LookupTable table(range_, resolution);
  for (const Vector2f& point : pointcloud) {
    table.SetPointValue(point, 1);
  }
  table.GaussianBlur();
  return table;
}

LookupTable
CorrelativeScanMatcher::GetLookupTableLowRes(const LookupTable& high_res_table) {
  LookupTable low_res_table(range_, low_res_);
  // Run the max filter over the portions of this table.
  for (double x = -range_; x <= range_; x += low_res_) {
    for (double y = -range_; y <= range_; y += low_res_) {
      // Get the max value for all the cells that this low res
      // cell encompasses.
      double max_area = high_res_table.MaxArea(x - low_res_, y - low_res_, x, y);
      low_res_table.SetPointValue(Vector2f(x, y), max_area);
    }
  }
  return low_res_table;
}

LookupTable
CorrelativeScanMatcher::GetLookupTableHighRes(const vector<Vector2f>& pointcloud) {
  return GetLookupTable(pointcloud, high_res_);
}

vector<Vector2f> RotatePointcloud(const vector<Vector2f>& pointcloud,
                                  const double rotation) {
  const Eigen::Matrix2f rot_matrix =
    Eigen::Rotation2Df(rotation).toRotationMatrix();
  vector<Vector2f> rotated_pointcloud;
  for (const Vector2f& point : pointcloud) {
    rotated_pointcloud.push_back(rot_matrix * point);
  }
  return rotated_pointcloud;
}

double CalculatePointcloudCost(const vector<Vector2f>& pointcloud,
                               const double x_trans,
                               const double y_trans,
                               const LookupTable& cost_table) {
  double probability = 0.0;
  double min_cost = 1.0;
  double hits = 0;
  double dep_factor = 0.6;
  for (const Vector2f& point : pointcloud) {
    double cost = cost_table.GetPointValue(point + Vector2f(x_trans, y_trans));
    if (cost < 0) {
      continue;
    }
    hits++;
    // Only count as percentage of points that fall inside the grid.
    probability += log(cost);
    if (cost < min_cost) {
      min_cost = cost;
    }
  }
  return exp(probability / (hits * dep_factor));
}

std::pair<double, std::pair<Eigen::Vector2f, float>>
CorrelativeScanMatcher::GetProbAndTransformation(const vector<Vector2f>& pointcloud_a,
                                                 const LookupTable& pointcloud_b_cost,
                                                 double resolution,
                                                 double x_min,
                                                 double x_max,
                                                 double y_min, 
                                                 double y_max,
                                                 bool excluding,
                                                 const boost::dynamic_bitset<>& excluded) {
  
  std::pair<Eigen::Vector2f, float> current_most_likely_trans =
    std::make_pair(Vector2f(x_min + resolution, y_min + resolution), 0);
  double current_most_likely_prob = 0.0;
  // One degree accuracy seems to be enough for now.
  for (double rotation = 0; rotation <= M_2_PI; rotation += M_PI / 180) {
    // Rotate the pointcloud by this rotation.
    const vector<Vector2f> rotated_pointcloud_a =
      RotatePointcloud(pointcloud_a, rotation);
    for (double x_trans = x_min + resolution; x_trans <= x_max; x_trans += resolution) {
      for (double y_trans = y_min + resolution;
           y_trans <= y_max;
           y_trans += resolution) {
        // If we are excluding scans, and this is a banned scan. Then don't
        // consider it.
        if (excluding &&
            excluded[pointcloud_b_cost.AbsCoords(x_trans, y_trans)]) {
          continue;
        }
        // Otherwise, get the probability / cost of this scan.
        double probability =
          CalculatePointcloudCost(
            rotated_pointcloud_a,
            x_trans,
            y_trans,
            pointcloud_b_cost);
        // If it is the best so far, keep track of it!
        if (probability > current_most_likely_prob) {
          current_most_likely_trans = std::pair<Eigen::Vector2f, float>(Vector2f(x_trans, y_trans),
                                                  rotation);
          current_most_likely_prob = probability;
        }
      }
    }
  }
  return std::pair<double, std::pair<Eigen::Vector2f, float>>(current_most_likely_prob,
                                        current_most_likely_trans);
}

std::pair<double, std::pair<Eigen::Vector2f, float>>
CorrelativeScanMatcher::GetTransformation(const vector<Vector2f>& pointcloud_a,
                                          const vector<Vector2f>& pointcloud_b) {
  double current_probability = 1.0;
  double best_probability = 0.0;
  std::pair<Eigen::Vector2f, float> best_transformation;
  uint64_t low_res_width = (range_ * 2.0) / low_res_ + 1;
  boost::dynamic_bitset<> excluded_low_res(low_res_width * low_res_width);
  boost::dynamic_bitset<> excluded_high_res(0); // Dumby value, never used.
  const LookupTable pointcloud_b_cost_high_res =
    GetLookupTableHighRes(pointcloud_b);
  const LookupTable pointcloud_b_cost_low_res =
    GetLookupTableLowRes(pointcloud_b_cost_high_res);
  // What is this while loop for? What even changes over the course of the loop?
  while (current_probability >= best_probability) {
    // Evaluate over the low_res lookup table.
    auto prob_and_trans_low_res =
      GetProbAndTransformation(pointcloud_a,
                               pointcloud_b_cost_low_res,
                               low_res_,
                               -range_,
                               range_,
                               -range_,
                               range_,
                               true,
                               excluded_low_res);
    if (prob_and_trans_low_res.first < best_probability) {
      return std::make_pair(best_probability, best_transformation);
    }
    printf("Found Low Res Pose (%f, %f): %f\n", prob_and_trans_low_res.second.first.x(), prob_and_trans_low_res.second.first.y(), prob_and_trans_low_res.first);

    double x_min_high_res = std::max(prob_and_trans_low_res.second.first.x() - low_res_, -range_);
    double x_max_high_res = std::min(prob_and_trans_low_res.second.first.x() + low_res_, range_);
    double y_min_high_res = std::max(prob_and_trans_low_res.second.first.y() - low_res_, -range_);
    double y_max_high_res = std::min(prob_and_trans_low_res.second.first.y() + low_res_, range_);
    printf("Commencing High Res Search in window (%f, %f) (%f, %f) \n", x_min_high_res, y_min_high_res, x_max_high_res, y_max_high_res);
    CHECK_LT(x_min_high_res, range_);
    CHECK_LT(y_min_high_res, range_);
    CHECK_GT(x_max_high_res, -range_);
    CHECK_GT(y_max_high_res, -range_);
    std::cout << "Banning Abs Coords: " << pointcloud_b_cost_low_res.AbsCoords(prob_and_trans_low_res.second.first.x(), prob_and_trans_low_res.second.first.y()) << std::endl;
    CHECK(!excluded_low_res[pointcloud_b_cost_low_res.AbsCoords(prob_and_trans_low_res.second.first.x(), prob_and_trans_low_res.second.first.y())]);
    excluded_low_res.set(pointcloud_b_cost_low_res.AbsCoords(prob_and_trans_low_res.second.first.x(),
                                                             prob_and_trans_low_res.second.first.y()),
                         true);
    auto prob_and_trans_high_res = GetProbAndTransformation(pointcloud_a,
                                                            pointcloud_b_cost_high_res,
                                                            high_res_,
                                                            x_min_high_res,
                                                            x_max_high_res,
                                                            y_min_high_res,
                                                            y_max_high_res,
                                                            false,
                                                            excluded_high_res);
    
    printf("Found High Res Pose (%f, %f): %f\n", prob_and_trans_high_res.second.first.x(), prob_and_trans_high_res.second.first.y(), prob_and_trans_high_res.first);

    if (prob_and_trans_high_res.first > best_probability) {
      // This is the new best and we should keep searching to make
      // sure there is nothing better.
      best_probability = prob_and_trans_high_res.first;
      best_transformation = prob_and_trans_high_res.second;
    }
    current_probability = prob_and_trans_high_res.first;
  }
  return std::make_pair(best_probability, best_transformation);
}

std::pair<double, std::pair<Eigen::Vector2f, float>>
CorrelativeScanMatcher::GetTransformation(const vector<Vector2f>& pointcloud_a,
                                          const vector<Vector2f>& pointcloud_b,
                                          const double rotation_a,
                                          const double rotation_b) {
  const vector<Vector2f>& rotated_pointcloud_a =
    RotatePointcloud(pointcloud_a, rotation_a);
  const vector<Vector2f>& rotated_pointcloud_b =
    RotatePointcloud(pointcloud_b, rotation_b);
  return GetTransformation(rotated_pointcloud_a,
                           rotated_pointcloud_b);
}

Eigen::Matrix3f
CorrelativeScanMatcher::GetUncertaintyMatrix(const vector<Vector2f>& pointcloud_a,
                                             const vector<Vector2f>& pointcloud_b) {
  // Calculation Method taken from Realtime Correlative Scan Matching
  // by Edward Olsen.
  Eigen::Matrix3f K = Eigen::Matrix3f::Zero();
  Eigen::Vector3f u(0, 0, 0);
  double s = 0;
  const LookupTable pointcloud_b_cost_high_res = GetLookupTableHighRes(pointcloud_b);
  const LookupTable pointcloud_b_cost_low_res =
          GetLookupTableLowRes(pointcloud_b_cost_high_res);
  vector<double> low_res_costs(pointcloud_b_cost_low_res.AbsCoords(range_, range_) + 1, -1);
  for (double rotation = 0; rotation <= M_2_PI; rotation += M_PI / 180) {
    // Rotate the pointcloud by this rotation.
    const vector<Vector2f> rotated_pointcloud_a =
            RotatePointcloud(pointcloud_a, rotation);
    for (double x_trans = -range_ + high_res_;
         x_trans <= range_;
         x_trans += high_res_) {
      for (double y_trans = -range_ + high_res_;
           y_trans <= range_;
           y_trans += high_res_) {
        // If this is a negligible amount of the total sum then just use the
        // low res cost, don't worry about the high res cost.
        size_t low_res_cost_idx =
          pointcloud_b_cost_low_res.AbsCoords(x_trans, y_trans);
        double low_res_cost =
          low_res_costs[low_res_cost_idx];
        if (low_res_cost < 0) {
          double low_res_x = x_trans - std::fmod(x_trans, low_res_);
          double low_res_y = y_trans - std::fmod(y_trans, low_res_);
          low_res_costs[low_res_cost_idx] =
            CalculatePointcloudCost(rotated_pointcloud_a,
                                    low_res_x,
                                    low_res_y,
                                    pointcloud_b_cost_low_res);
          low_res_cost = low_res_costs[low_res_cost_idx];
        }
        CHECK_GE(low_res_cost, 0);
        double cost = 0.0;
        if (low_res_cost <= UNCERTAINTY_USELESS_THRESHOLD) {
          cost = low_res_cost;
        } else {
          cost = CalculatePointcloudCost(rotated_pointcloud_a,
                                         x_trans,
                                         y_trans,
                                         pointcloud_b_cost_high_res);
        }
        Eigen::Vector3f x(x_trans, y_trans, rotation);
        K += x * x.transpose() * cost;
        u += x * cost;
        s += cost;
      }
    }
  }
  // Calculate Uncertainty matrix.
  std::cout << "K: " << std::endl << K << std::endl;
  std::cout << "u " << std::endl << u << std::endl;
  std::cout << "s: " << std::endl << s << std::endl;
  Eigen::Matrix3f uncertainty = (1.0/s) * K - (1.0/(s*s)) * u * u.transpose();
  return uncertainty;
}

Eigen::Matrix3f
CorrelativeScanMatcher::GetUncertaintyMatrix(const vector<Vector2f>& pointcloud_a,
                                             const vector<Vector2f>& pointcloud_b,
                                             double rotation_a,
                                             double rotation_b) {
  const vector<Vector2f>& rotated_pointcloud_a =
    RotatePointcloud(pointcloud_a, rotation_a);
  const vector<Vector2f>& rotated_pointcloud_b =
    RotatePointcloud(pointcloud_b, rotation_b);
  return GetUncertaintyMatrix(rotated_pointcloud_a, rotated_pointcloud_b);
}
