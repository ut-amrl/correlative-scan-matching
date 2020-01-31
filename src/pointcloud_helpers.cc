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

vector<Vector2f>
pointcloud_helpers::PointCloudToVector(sensor_msgs::PointCloud2 &cloud) {
  vector<Vector2f> point_cloud;
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  
  for(; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
    point_cloud.push_back(Vector2f(*iter_x, *iter_y));
  }

  return point_cloud;
}
