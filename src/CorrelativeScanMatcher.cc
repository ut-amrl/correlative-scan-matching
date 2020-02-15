#include "./CorrelativeScanMatcher.h"

#include <vector>
#include <algorithm>

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
              const double rotation_min,
              const double rotation_max) {
  typedef pair<double, pair<Vector2f, float>> TransProb;
  vector<TransProb> low_res_costs(low_res_table.AbsCoords(range, range) + 1,
                                  std::make_pair(-INFINITY,
                                                 std::make_pair(Vector2f(0,0),
                                                                0)));
  for (double rotation = rotation_min; rotation < rotation_max; rotation += M_PI / 180) {
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
                         double rotation,
                         bool excluding,
                         const boost::dynamic_bitset<>& excluded) {
  pair<Eigen::Vector2f, float> current_most_likely_trans =
    std::make_pair(Vector2f(x_min, y_min), rotation);
  double current_most_likely_prob = -INFINITY;

  for (double x_trans = x_min + EPSILON; x_trans < x_max; x_trans += resolution) {
    for (double y_trans = y_min + EPSILON; y_trans < y_max; y_trans += resolution) {
      // If we are excluding scans, and this is a banned scan. Then don't
      // consider it.
      if (excluding && excluded[pointcloud_b_cost.AbsCoords(x_trans, y_trans)]) {
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
                  const vector<Vector2f>& pointcloud_b) {
  double current_probability = 1.0;
  double best_probability = -INFINITY;
  pair<Eigen::Vector2f, float> best_transformation;
  uint64_t low_res_width = (range_ * 2.0) / low_res_ + 1;
  boost::dynamic_bitset<> excluded_low_res(low_res_width * low_res_width);
  // Dumby value, never used.
  boost::dynamic_bitset<> excluded_high_res(0);
  const LookupTable pointcloud_b_cost_high_res =
    GetLookupTableHighRes(pointcloud_b);
  const LookupTable pointcloud_b_cost_low_res =
    GetLookupTableLowRes(pointcloud_b_cost_high_res);
  vector<pair<double, pair<Vector2f, float>>> low_res_costs = MemoizeLowRes(pointcloud_a, pointcloud_b_cost_low_res, low_res_, trans_range_, 0, 2 * M_PI);
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
               -trans_range_);
    double x_max_high_res =
      std::min(best_translation_low_res.x() + low_res_,
               trans_range_);
    double y_min_high_res =
      std::max(best_translation_low_res.cast<double>().y(),
               -trans_range_);
    double y_max_high_res =
      std::min(best_translation_low_res.y() + low_res_,
               trans_range_);
    
    #if DEBUG
    printf("Commencing High Res Search in window (%f, %f) (%f, %f) \n",
           x_min_high_res,
           y_min_high_res,
           x_max_high_res,
           y_max_high_res);
    #endif
    
    CHECK_LT(x_min_high_res, trans_range_);
    CHECK_LT(y_min_high_res, trans_range_);
    CHECK_GT(x_max_high_res, -trans_range_);
    CHECK_GT(y_max_high_res, -trans_range_);
    double trans_x = best_translation_low_res.x();
    double trans_y = best_translation_low_res.y();
    if (excluded_low_res[pointcloud_b_cost_low_res.AbsCoords(trans_x,
                                                             trans_y)]) {
      return std::make_pair(best_probability, best_transformation);
    }
    excluded_low_res.set(pointcloud_b_cost_low_res.AbsCoords(trans_x,
                                                             trans_y), true);
    auto prob_and_trans_high_res =
      GetProbAndTransformation(rotated_pointcloud_a,
                               pointcloud_b_cost_high_res,
                               high_res_,
                               x_min_high_res,
                               x_max_high_res,
                               y_min_high_res,
                               y_max_high_res,
                               best_rotation_low_res,
                               false,
                               excluded_high_res);
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
                                          const double rotation_b) {
  const vector<Vector2f>& rotated_pointcloud_a =
    RotatePointcloud(pointcloud_a, rotation_a);
  const vector<Vector2f>& rotated_pointcloud_b =
    RotatePointcloud(pointcloud_b, rotation_b);
  return GetTransformation(rotated_pointcloud_a,
                           rotated_pointcloud_b);
}

// Calculates full uncertainty (including rotation)
Eigen::Matrix3f
CorrelativeScanMatcher::
GetUncertaintyMatrix(const vector<Vector2f>& pointcloud_a,
                     const vector<Vector2f>& pointcloud_b) {
  // Calculation Method taken from Realtime Correlative Scan Matching
  // by Edward Olsen.
  typedef pair<double, pair<Vector2f, float>> TransProb;
  Eigen::Matrix3f K = Eigen::Matrix3f::Zero();
  Eigen::Vector3f u(0, 0, 0);
  double s = 0;
  const LookupTable pointcloud_b_cost_high_res =
    GetLookupTableHighRes(pointcloud_b);
  const LookupTable pointcloud_b_cost_low_res =
    GetLookupTableLowRes(pointcloud_b_cost_high_res);
  const vector<TransProb> low_res_costs =
    MemoizeLowRes(pointcloud_a,
                  pointcloud_b_cost_low_res,
                  low_res_,
                  trans_range_,
                  0,
                  2 * M_PI);
  printf("Calculating Uncertainty...\n");
  for (double rotation = 0; rotation < 2*M_PI; rotation += M_PI / 180) {
    // Rotate the pointcloud by this rotation.
    const vector<Vector2f> rotated_pointcloud_a =
            RotatePointcloud(pointcloud_a, rotation);
    for (double x_trans = -trans_range_ + EPSILON;
         x_trans < trans_range_;
         x_trans += low_res_) {
      for (double y_trans = -trans_range_ + EPSILON;
          y_trans < trans_range_;
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

// Calculates x-y uncertainty
Eigen::Matrix2f
CorrelativeScanMatcher::
GetUncertaintyMatrix(const vector<Vector2f>& pointcloud_a,
                     const vector<Vector2f>& pointcloud_b,
                     const double rotation) {
  // Calculation Method taken from Realtime Correlative Scan Matching
  // by Edward Olsen.
  Eigen::Matrix2f K = Eigen::Matrix2f::Zero();
  Eigen::Vector2f u(0, 0);
  double s = 0;
  const LookupTable pointcloud_b_cost_high_res =
    GetLookupTableHighRes(pointcloud_b);
  const LookupTable pointcloud_b_cost_low_res =
    GetLookupTableLowRes(pointcloud_b_cost_high_res);
  printf("Calculating Uncertainty given rotation...\n");
  // Rotate the pointcloud by this rotation.
  const vector<Vector2f> rotated_pointcloud_a = RotatePointcloud(pointcloud_a, rotation);
  auto low_res_costs = MemoizeLowRes(pointcloud_a, pointcloud_b_cost_low_res, low_res_, trans_range_, rotation, rotation + EPSILON);

  for (double x_trans = -trans_range_ + EPSILON;
        x_trans < trans_range_;
        x_trans += low_res_) {
    for (double y_trans = -trans_range_ + EPSILON;
        y_trans < trans_range_;
        y_trans += low_res_) {
      // If this is a negligible amount of the total sum then just use the
      // low res cost, don't worry about the high res cost.
      size_t abs_coord = pointcloud_b_cost_low_res.AbsCoords(x_trans, y_trans);
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
      Eigen::Vector2f x(x_trans, y_trans);
      K += x * x.transpose() * cost;
      u += x * cost;
      s += cost;
    }
  }
  // Calculate Uncertainty matrix.
  // std::cout << "K: " << std::endl << K << std::endl;
  // std::cout << "u " << std::endl << u << std::endl;
  // std::cout << "s: " << std::endl << s << std::endl;
  Eigen::Matrix2f uncertainty = (1.0/s) * K - (1.0/(s*s)) * u * u.transpose();
  return uncertainty;
}