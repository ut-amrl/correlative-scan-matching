//
// Created by jack on 9/15/19.
//

#include <glog/logging.h>
#include "./pointcloud_helpers.h"

#include "ros/package.h"
#include "eigen3/Eigen/Dense"
#include "sensor_msgs/PointCloud2.h"

#include "./math_util.h"

using Eigen::Vector2f;
using Eigen::Matrix2f;
using Eigen::Rotation2D;
using std::pair;
using std::vector;
using math_util::NormalsSimilar;

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
                                          double max_range) {
  vector<pair<size_t, Vector2f>> pointcloud;
  float angle_offset = 0.0f;
  for (size_t index = 0; index < laser_scan.ranges.size(); index++) {
    float range = laser_scan.ranges[index];
    if (range >= laser_scan.range_min && range <= max_range) {
      // Only accept valid ranges.
      // Then we must rotate the point by the specified angle at that distance.
      Vector2f point(range, 0.0);
      Matrix2f rot_matrix =
        Rotation2D<float>(laser_scan.angle_min +
                         (laser_scan.angle_increment * index))
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

void pointcloud_helpers::InitPointcloud(PointCloud2* point) {
  std::string arr[3] = {"x", "y", "z"};
  point->header.seq = 1;
  point->header.stamp = ros::Time::now();
  point->header.frame_id = "map";
  sensor_msgs::PointField field;
  int offset = 0;
  field.datatype = 7;
  field.count = 1;
  for (std::string type : arr) {
    field.offset = offset;
    field.name = type;
    point->fields.push_back(field);
    offset += 4;
  }
  point->height = 1;
  point->width = 0;
  point->is_bigendian = false;
  point->point_step = 12;
  point->row_step = 0;
  point->is_dense = true;
}

void pointcloud_helpers::PushBackBytes(float val,
                                       sensor_msgs::PointCloud2& ptr) {
  uint8_t *data_ptr = reinterpret_cast<uint8_t*>(&val);
  for (int i = 0; i < 4; i++) {
    ptr.data.push_back(data_ptr[i]);
  }
}

void
pointcloud_helpers::PublishPointcloud(const vector<Vector2f>& points,
                                      PointCloud2& point_cloud,
                                      Publisher& pub) {
  for (uint64_t i = 0; i < points.size(); i++) {
    Vector2f vec = points[i];
    PushBackBytes(vec[0], point_cloud);
    PushBackBytes(vec[1], point_cloud);
    PushBackBytes(0.0f, point_cloud);
  }
  point_cloud.width = points.size();
  pub.publish(point_cloud);
  point_cloud.width = 0;
  point_cloud.data.clear();
}
