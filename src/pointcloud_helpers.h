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
using ros::Publisher;

namespace pointcloud_helpers {
  void InitPointcloud(PointCloud2* point);
  void PushBackBytes(float val, sensor_msgs::PointCloud2& ptr);
  void PublishPointcloud(const std::vector<Vector2f>& points,
                         PointCloud2& point_cloud,
                         Publisher& pub);
  std::vector<Vector2f>
  LaserScanToPointCloud(sensor_msgs::LaserScan &laser_scan, double max_range);

};
#endif // SRC_POINTCLOUD_HELPERS_H_
