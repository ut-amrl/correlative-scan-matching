//
// Created by jack on 9/15/19.
//

#ifndef SRC_POINTCLOUD_HELPERS_H_
#define SRC_POINTCLOUD_HELPERS_H_

#include <sensor_msgs/LaserScan.h>
#include "eigen3/Eigen/Dense"
#include "ros/package.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

using Eigen::Vector2f;
using sensor_msgs::PointCloud2;

namespace pointcloud_helpers {
std::vector<Vector2f> RosCloudToPointCloud(sensor_msgs::PointCloud2 &cloud);
std::vector<Vector2f> LaserScanToPointCloud(sensor_msgs::LaserScan &laser_scan,
                                            double max_range,
                                            bool truncate_ends);
};      // namespace pointcloud_helpers
#endif  // SRC_POINTCLOUD_HELPERS_H_
