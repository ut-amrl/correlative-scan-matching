#include "./CorrelativeScanMatcher.h"

#include <vector>
#include <algorithm>

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/dynamic_bitset.hpp>
#include "Eigen/Dense"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "./CImg.h"
#include "./pointcloud_helpers.h"

#define UNCERTAINTY_USELESS_THRESHOLD log(1e-4)
#define DEBUG false

using std::vector;
using std::pair;
using Eigen::Vector2f;
using boost::math::chi_squared;
using boost::math::complement;
using boost::math::quantile;

using sensor_msgs::PointCloud2;

#define EPSILON 1e-6

LookupTable
CorrelativeScanMatcher::GetLookupTable(const vector<Vector2f>& pointcloud,
                                       double resolution) {
  LookupTable table(range_, resolution);
  for (const Vector2f& point : pointcloud) {
    table.SetPointValue(point, 1);
  }
  table.GaussianBlur();
  table.normalize();
  return table;
}

LookupTable
CorrelativeScanMatcher::
GetLookupTableLowRes(const LookupTable& high_res_table) {
  LookupTable low_res_table(range_, low_res_);
  // Run the max filter over the portions of this table.

  for (double x = -range_ + EPSILON; x < range_; x += low_res_) {
    for (double y = -range_ + EPSILON; y < range_; y += low_res_) {
      // Get the max value for all the cells that this low res
      // cell encompasses.
      double max_area = high_res_table.MaxArea(x, y, x + low_res_, y + low_res_);
      low_res_table.SetPointValue(Vector2f(x, y), max_area);
    }
  }

  return low_res_table;
}

LookupTable
CorrelativeScanMatcher::
GetLookupTableHighRes(const vector<Vector2f>& pointcloud) {
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
  for (const Vector2f& point : pointcloud) {
    double cost = cost_table.GetPointValue(point + Vector2f(x_trans, y_trans));
    // Only count as percentage of points that fall inside the grid.
    probability += log(cost);
  }

  return probability / pointcloud.size();
}

bool LessThanTransProb(pair<double, pair<Vector2f, float>> a,
                       pair<double, pair<Vector2f, float>> b) {
  return a.first < b.first;
}

vector<pair<double, pair<Vector2f, float>>>
MemoizeLowRes(const vector<Vector2f>& query_scan,
              const LookupTable& low_res_table,
              const double resolution,
              const double range,
              const double restrict_rotation_min,
              const double restrict_rotation_max) {
  typedef pair<double, pair<Vector2f, float>> TransProb;
  vector<TransProb> low_res_costs(low_res_table.AbsCoords(range, range) + 1,
                                  std::make_pair(-INFINITY,
                                                 std::make_pair(Vector2f(0,0),
                                                                0)));
  for (double rotation = restrict_rotation_min;
       rotation < restrict_rotation_max;
       rotation += M_PI / 180) {
    // Rotate the pointcloud by this rotation.
    const vector<Vector2f> rotated_query =
            RotatePointcloud(query_scan, rotation);
    for (double x_trans = -range + EPSILON;
         x_trans < range;
         x_trans += resolution) {
      for (double y_trans = -range + EPSILON;
           y_trans < range;
           y_trans += resolution) {
        // If this is a negligible amount of the total sum then just use the
        // low res cost, don't worry about the high res cost.
        pair<Vector2f, float> transformation =
                std::make_pair(Vector2f(x_trans, y_trans), rotation);
        double low_res_cost =
                CalculatePointcloudCost(rotated_query,
                                        x_trans,
                                        y_trans,
                                        low_res_table);
        TransProb trans_prob = std::make_pair(low_res_cost, transformation);
        size_t abs_coord = low_res_table.AbsCoords(x_trans, y_trans);
        low_res_costs[abs_coord] =
                std::max(low_res_costs[abs_coord],
                         trans_prob,
                         LessThanTransProb);
      }
    }
  }
  return low_res_costs;
}

pair<double, pair<Eigen::Vector2f, float>>
CorrelativeScanMatcher::
GetProbAndTransformation(const vector<Vector2f>& rotated_pointcloud_a,
                         const LookupTable& pointcloud_b_cost,
                         double resolution,
                         double x_min,
                         double x_max,
                         double y_min,
                         double y_max,
                         double rotation) {
  pair<Eigen::Vector2f, float> current_most_likely_trans =
          std::make_pair(Vector2f(x_min, y_min), rotation);
  double current_most_likely_prob = -INFINITY;

  for (double x_trans = x_min + EPSILON; x_trans < x_max; x_trans += resolution) {
    for (double y_trans = y_min + EPSILON; y_trans < y_max; y_trans += resolution) {
      // Otherwise, get the probability / cost of this scan.
      double probability =
              CalculatePointcloudCost(
                      rotated_pointcloud_a,
                      x_trans,
                      y_trans,
                      pointcloud_b_cost);
      // If it is the best so far, keep track of it!
      if (probability > current_most_likely_prob) {
        current_most_likely_trans =
                pair<Eigen::Vector2f, float>(Vector2f(x_trans, y_trans),
                                             rotation);
        current_most_likely_prob = probability;
      }
    }
  }
  return pair<double, pair<Eigen::Vector2f, float>>(current_most_likely_prob,
          current_most_likely_trans);
}


pair<double, pair<Eigen::Vector2f, float>>
CorrelativeScanMatcher::
GetTransformation(const vector<Vector2f>& pointcloud_a,
                  const vector<Vector2f>& pointcloud_b,
                  const double rotation_min,
                  const double rotation_max) {
  double current_probability = 1.0;
  double best_probability = -INFINITY;
  double smaller_range = 2;
  pair<Eigen::Vector2f, float> best_transformation;
  const LookupTable pointcloud_b_cost_high_res =
          GetLookupTableHighRes(pointcloud_b);
  const LookupTable pointcloud_b_cost_low_res =
          GetLookupTableLowRes(pointcloud_b_cost_high_res);
  vector<pair<double, pair<Vector2f, float>>> low_res_costs =
                                             MemoizeLowRes(pointcloud_a,
                                                           pointcloud_b_cost_low_res,
                                                           low_res_,
                                                           smaller_range,
                                                           rotation_min,
                                                           rotation_max);
#if DEBUG
  std::cout << "Low Res Cost: " << CalculatePointcloudCost(RotatePointcloud(pointcloud_a, 3.14), 0.7, -0.2, pointcloud_b_cost_low_res) << std::endl;
  std::cout << "High Res Cost: " << CalculatePointcloudCost(RotatePointcloud(pointcloud_a, 3.14), 0.7, -0.2, pointcloud_b_cost_high_res) << std::endl;
#endif
  while (current_probability >= best_probability) {
    // Evaluate over the low_res lookup table.
    std::vector<pair<double, pair<Vector2f, float>>>::iterator max =
                                                                       std::max_element(low_res_costs.begin(),
                                                                                        low_res_costs.end(),
                                                                                        LessThanTransProb);
    // Exclude this scan so we never get it again.
    auto prob_and_trans_low_res = *max;
    max->first = -INFINITY;
    current_probability = prob_and_trans_low_res.first;
    if (current_probability < best_probability) {
      break;
    }
    //double best_probability_low_res = best_prob_and_trans_low_res.first;
    Vector2f best_translation_low_res = prob_and_trans_low_res.second.first;
    double best_rotation_low_res = prob_and_trans_low_res.second.second;
    auto rotated_pointcloud_a = RotatePointcloud(pointcloud_a, prob_and_trans_low_res.second.second);
#if DEBUG
    printf("Found Low Res Pose (%f, %f), rotation %f: %f\n",
           best_translation_low_res.x(),
           best_translation_low_res.y(),
           best_rotation_low_res,
           best_probability_low_res);
#endif
    double x_min_high_res =
            std::max(best_translation_low_res.cast<double>().x(),
                     -smaller_range);
    double x_max_high_res =
            std::min(best_translation_low_res.x() + low_res_,
                     smaller_range);
    double y_min_high_res =
            std::max(best_translation_low_res.cast<double>().y(),
                     -smaller_range);
    double y_max_high_res =
            std::min(best_translation_low_res.y() + low_res_,
                     smaller_range);
#if DEBUG
    printf("Commencing High Res Search in window (%f, %f) (%f, %f) \n",
           x_min_high_res,
           y_min_high_res,
           x_max_high_res,
           y_max_high_res);
#endif
    CHECK_LT(x_min_high_res, smaller_range);
    CHECK_LT(y_min_high_res, smaller_range);
    CHECK_GT(x_max_high_res, -smaller_range);
    CHECK_GT(y_max_high_res, -smaller_range);
    auto prob_and_trans_high_res =
            GetProbAndTransformation(rotated_pointcloud_a,
                                     pointcloud_b_cost_high_res,
                                     high_res_,
                                     x_min_high_res,
                                     x_max_high_res,
                                     y_min_high_res,
                                     y_max_high_res,
                                     best_rotation_low_res);
#if DEBUG
    if (prob_and_trans_high_res.first > best_probability_low_res) {
      for (double y = y_min_high_res; y < y_max_high_res; y += high_res_) {
        for (double x = x_min_high_res; x < x_max_high_res; x += high_res_) {
          std::cout << CalculatePointcloudCost(RotatePointcloud(pointcloud_a, best_rotation_low_res), x, y, pointcloud_b_cost_high_res) << " ";
        }
        std::cout << std::endl;
      }
    }
    printf("Found High Res Pose (%f, %f, %f): %f\n",
           prob_and_trans_high_res.second.first.x(),
           prob_and_trans_high_res.second.first.y(),
           prob_and_trans_high_res.second.second,
           prob_and_trans_high_res.first);
#endif

    if (prob_and_trans_high_res.first > best_probability) {
      // This is the new best and we should keep searching to make
      // sure there is nothing better.
      best_probability = prob_and_trans_high_res.first;
      best_transformation = prob_and_trans_high_res.second;
    }
  }
  return std::make_pair(best_probability, best_transformation);
}

pair<double, pair<Eigen::Vector2f, float>>
CorrelativeScanMatcher::GetTransformation(const vector<Vector2f>& pointcloud_a,
                                          const vector<Vector2f>& pointcloud_b,
                                          const double rotation_a,
                                          const double rotation_b,
                                          const double rotation_restriction) {
  const vector<Vector2f>& rotated_pointcloud_a =
          RotatePointcloud(pointcloud_a, rotation_a);
  const vector<Vector2f>& rotated_pointcloud_b =
          RotatePointcloud(pointcloud_b, rotation_b);
  return GetTransformation(rotated_pointcloud_a,
                           rotated_pointcloud_b,
                           -(rotation_restriction) / 2,
                           rotation_restriction / 2);
}



Eigen::Matrix3f
CorrelativeScanMatcher::
GetUncertaintyMatrix(const vector<Vector2f>& pointcloud_a,
                     const vector<Vector2f>& pointcloud_b) {
  // Calculation Method taken from Realtime Correlative Scan Matching
  // by Edward Olsen.
  // TODO: Give uncertainty calculations ability to restrict the rotation as well.
  typedef pair<double, pair<Vector2f, float>> TransProb;
  Eigen::Matrix3f K = Eigen::Matrix3f::Zero();
  Eigen::Vector3f u(0, 0, 0);
  double s = 0;
  double smaller_range = 2;
  const LookupTable pointcloud_b_cost_high_res =
          GetLookupTableHighRes(pointcloud_b);
  const LookupTable pointcloud_b_cost_low_res =
          GetLookupTableLowRes(pointcloud_b_cost_high_res);
  const vector<TransProb> low_res_costs =
          MemoizeLowRes(pointcloud_a,
                        pointcloud_b_cost_low_res,
                        low_res_,
                        smaller_range,
                        0,
                        2 * M_PI);
  printf("Calculating Uncertainty...\n");
  for (double rotation = 0; rotation < 2*M_PI; rotation += M_PI / 180) {
    // Rotate the pointcloud by this rotation.
    const vector<Vector2f> rotated_pointcloud_a =
            RotatePointcloud(pointcloud_a, rotation);
    for (double x_trans = -smaller_range + EPSILON;
         x_trans < smaller_range;
         x_trans += low_res_) {
      for (double y_trans = -smaller_range + EPSILON;
           y_trans < smaller_range;
           y_trans += low_res_) {
        // If this is a negligible amount of the total sum then just use the
        // low res cost, don't worry about the high res cost.
        size_t abs_coord =
                pointcloud_b_cost_low_res.AbsCoords(x_trans, y_trans);
        double low_res_cost = low_res_costs[abs_coord].first;
        double cost = 0.0;
        if (low_res_cost <= UNCERTAINTY_USELESS_THRESHOLD) {
          cost = low_res_cost;
        } else {
          cost = CalculatePointcloudCost(rotated_pointcloud_a,
                                         x_trans,
                                         y_trans,
                                         pointcloud_b_cost_high_res);
        }
        cost = exp(cost);
        Eigen::Vector3f x(x_trans, y_trans, rotation);
        K += x * x.transpose() * cost;
        u += x * cost;
        s += cost;
      }
    }
  }
  // Calculate Uncertainty matrix.
  // std::cout << "K: " << std::endl << K << std::endl;
  // std::cout << "u " << std::endl << u << std::endl;
  // std::cout << "s: " << std::endl << s << std::endl;
  Eigen::Matrix3f uncertainty = (1.0/s) * K - (1.0/(s*s)) * u * u.transpose();
  return uncertainty;
}

Eigen::Matrix3f
CorrelativeScanMatcher::
GetUncertaintyMatrix(const vector<Vector2f>& pointcloud_a,
                     const vector<Vector2f>& pointcloud_b,
                     double rotation_a,
                     double rotation_b) {
  const vector<Vector2f>& rotated_pointcloud_a =
          RotatePointcloud(pointcloud_a, rotation_a);
  const vector<Vector2f>& rotated_pointcloud_b =
          RotatePointcloud(pointcloud_b, rotation_b);
  return GetUncertaintyMatrix(rotated_pointcloud_a, rotated_pointcloud_b);
}

bool CorrelativeScanMatcher::SimilarScans(const vector<Vector2f>& pointcloud_a,
                                          const vector<Vector2f>& pointcloud_b,
                                          const double certainty) {
  CHECK_LE(certainty, 1.0);
  CHECK_GE(certainty, 0.0);
  const double uncertainty = 1 - certainty;
  const LookupTable b_table = GetLookupTableHighRes(pointcloud_b);
  double chi_squared_val = 0.0;
  for (const Vector2f& point : pointcloud_a) {
    // For every point the value here is 1, the value in b could be anything.
    std::cout << b_table.GetPointValue(point) << std::endl;
    double expected = b_table.GetPointValue(point) * 100;
    double observed = 100;
    chi_squared_val += pow(observed - expected, 2) / expected;
  }
  // Degrees of freedom is pointcloud size.
  chi_squared dist(pointcloud_a.size());
  std::cout << "Chi Squared Value: " << chi_squared_val << std::endl;
  std::cout << "Chi Squared Quantile: " << quantile(complement(dist, uncertainty)) << std::endl;
  if (chi_squared_val >= quantile(complement(dist, uncertainty))) {
    return false;
  }
  return true;
}

