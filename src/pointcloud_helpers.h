//
// Created by jack on 9/15/19.
//

#ifndef SRC_POINTCLOUD_HELPERS_H_
#define SRC_POINTCLOUD_HELPERS_H_

#include <sensor_msgs/LaserScan.h>
#include "ros/package.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "eigen3/Eigen/Dense"

using Eigen::Vector2f;
using sensor_msgs::PointCloud2;

namespace pointcloud_helpers {
  std::vector<Vector2f> PointCloudToVector(sensor_msgs::PointCloud2 &cloud);
};
#endif // SRC_POINTCLOUD_HELPERS_H_
