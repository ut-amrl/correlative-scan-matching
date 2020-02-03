//
// Created by jack on 9/15/19.
//

#include <glog/logging.h>
#include "./pointcloud_helpers.h"

#include "ros/package.h"
#include "eigen3/Eigen/Dense"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

using Eigen::Vector2f;
using Eigen::Matrix2f;
using Eigen::Rotation2D;
using std::pair;
using std::vector;

#define GLANCING_THRESHOLD 0.10
#define GLANCING_ANGLE_THRESHOLD 45

vector<Vector2f>
FilterGlancing(const float angle_min,
               const float angle_step,
               const vector<pair<size_t, Vector2f>> indexed_pointcloud) {
  vector<Vector2f> pointcloud;
  CHECK_GE(indexed_pointcloud.size(), 1);
  Vector2f last_point = indexed_pointcloud[0].second;
  // Throw out points that have a big distance between them and the last point,
  // helps to filter out points at a glancing distance.
  // TODO: We can rewrite this to use the normal once we have a way to find a
  //  points normal
  for (const pair<size_t, Vector2f>& indexed_point : indexed_pointcloud) {
    if ((last_point - indexed_point.second).norm() <= GLANCING_THRESHOLD) {
      pointcloud.push_back(indexed_point.second);
    }
    last_point = indexed_point.second;
  }
  return pointcloud;
}

vector<Vector2f>
pointcloud_helpers::LaserScanToPointCloud(sensor_msgs::LaserScan &laser_scan,
                                          double max_range,
                                          bool truncate_ends = false) {
  vector<pair<size_t, Vector2f>> pointcloud;
  float angle_offset = laser_scan.angle_min;
  float angle_truncation = truncate_ends ? M_PI / 12.0 : 0;
  for (size_t index = 0; index < laser_scan.ranges.size(); index++) {
    float range = laser_scan.ranges[index];
    if (range >= laser_scan.range_min && range <= max_range && angle_offset > laser_scan.angle_min + angle_truncation && angle_offset < laser_scan.angle_max - angle_truncation) {
      // Only accept valid ranges.
      // Then we must rotate the point by the specified angle at that distance.
      Vector2f point(range, 0.0);
      Matrix2f rot_matrix =
        Rotation2D<float>(angle_offset)
          .toRotationMatrix();
      point = rot_matrix * point;
      pointcloud.emplace_back(index, point);
    }
    angle_offset += laser_scan.angle_increment;
  }
  return FilterGlancing(laser_scan.angle_min,
                        laser_scan.angle_increment,
                        pointcloud);
}

vector<Vector2f>
pointcloud_helpers::RosCloudToPointCloud(sensor_msgs::PointCloud2 &cloud) {
  vector<Vector2f> point_cloud;
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  
  for(; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
    point_cloud.push_back(Vector2f(*iter_x, *iter_y));
  }

  return point_cloud;
}
