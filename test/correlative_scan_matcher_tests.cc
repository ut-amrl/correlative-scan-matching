//
// Created by jack on 11/24/19.
//


#include "gtest/gtest.h"
#include "eigen3/Eigen/Dense"
#include "../src/CorrelativeScanMatcher.h"

using Eigen::Vector2f;

TEST(CorrelativeScanMatcherTests, SetSinglePointTest) {
  LookupTable table(4, 1);
  table.SetPointValue(Vector2f(0, 0), 0.5);
  ASSERT_DOUBLE_EQ(table.GetPointValue(Vector2f(0, 0)), 0.5);
  ASSERT_DOUBLE_EQ(table.GetPointValue(Vector2f(.5, .5)), 0.5);
  ASSERT_LT(table.GetPointValue(Vector2f(-.5, -.5)), 0.5);
}

TEST(CorrelativeScanMatcherTests, OverwritePointTest) {
  LookupTable table(4, 1);
  table.SetPointValue(Vector2f(3.9, 3.9), 0.01);
  ASSERT_DOUBLE_EQ(table.GetPointValue(Vector2f(3.9, 3.9)), 0.01);
  table.SetPointValue(Vector2f(-3.9, -3.9), 0.02);
  ASSERT_DOUBLE_EQ(table.GetPointValue(Vector2f(-3.9, -3.9)), 0.02);
}

TEST(CorrelativeScanMatcherTests, NearbyPointsTest) {
  LookupTable table(4, 1);
  table.SetPointValue(Vector2f(-3.9, -3.9), 0.01);
  table.SetPointValue(Vector2f(-3.0, -3.0), 0.275);
  ASSERT_DOUBLE_EQ(table.GetPointValue(Vector2f(-3.51, -3.51)), 0.01);
  ASSERT_DOUBLE_EQ(table.GetPointValue(Vector2f(-3.49, -3.49)), 0.01);
  ASSERT_DOUBLE_EQ(table.GetPointValue(Vector2f(-2.99, -2.99)), 0.275);
}

TEST(CorrelativeScanMatcherTest, BoundaryEdgeTest) {
  LookupTable table(4, 1);
  table.SetPointValue(Vector2f(-4, -4), 0.23);
  ASSERT_DOUBLE_EQ(table.GetPointValue(Vector2f(-4, -4)), 0.23);
  ASSERT_DOUBLE_EQ(table.GetPointValue(Vector2f(-4, -3.99)), 0.23);
  table.SetPointValue(Vector2f(4, -4), 0.24);
  ASSERT_LT(table.GetPointValue(Vector2f(3.99, -4)), 0.24);
  table.SetPointValue(Vector2f(3.99, -4), 0.24);
  ASSERT_DOUBLE_EQ(table.GetPointValue(Vector2f(3.99, -3.99)), 0.24);
}

TEST(CorrelativeScanMatcherTest, MaxAreaTest) {
  LookupTable table(4, 1);
  table.SetPointValue(Vector2f(-3.5, -3), 0.2);
  table.SetPointValue(Vector2f(-3, 3), 0.1);
  table.SetPointValue(Vector2f(2, -4), 0.4);
  table.SetPointValue(Vector2f(2, 2), 0.5);
  table.SetPointValue(Vector2f(4, 0), 0.6);
  ASSERT_DOUBLE_EQ(table.MaxArea(-4, -3, 3, 1), 0.2);
}

TEST(CorrelativeScanMatcherTest, OutOfBoundsTest) {
  LookupTable table(4, 1);
  table.SetPointValue(Vector2f(-4, -5), 0.5);
  ASSERT_LT(table.MaxArea(-4, -4, 4, 4), 0.5);
}

