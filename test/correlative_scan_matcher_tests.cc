//
// Created by jack on 11/24/19.
//


#include "gtest/gtest.h"
#include "eigen3/Eigen/Dense"
#include "../src/CorrelativeScanMatcher.h"

using Eigen::Vector2f;

TEST(CorrelativeSCanMatcherTests, GetAndSetPointValueTest) {
  LookupTable table(4, 1);
  table.SetPointValue(Vector2f(0, 0), 0.5);
  ASSERT_DOUBLE_EQ(table.GetPointValue(Vector2f(0, 0)), 0.5);
  table.SetPointValue(Vector2f(3.9, 3.9), 0.01);
  ASSERT_DOUBLE_EQ(table.GetPointValue(Vector2f(3.9, 3.9)), 0.01);
  table.SetPointValue(Vector2f(-3.9, -3.9), 0.02);
  ASSERT_DOUBLE_EQ(table.GetPointValue(Vector2f(-3.9, -3.9)), 0.02);
  ASSERT_DOUBLE_EQ(table.GetPointValue(Vector2f(-3.5, -3.5)), 0.02);
  //table.SetPointValue(Vector2f(4, 4), 0.05);
  //ASSERT_DOUBLE_EQ(table.GetPointValue(Vector2f(4, 4)), 0.05);
  table.SetPointValue(Vector2f(-4, -4), 0.23);
  ASSERT_DOUBLE_EQ(table.GetPointValue(Vector2f(-4, -4)), 0.23);
  table.SetPointValue(Vector2f(-3, 2.5), 0.25);
  ASSERT_DOUBLE_EQ(table.GetPointValue(Vector2f(-3, 2.5)), 0.25);
  table.SetPointValue(Vector2f(3.6, -2.7), 0.27);
  ASSERT_DOUBLE_EQ(table.GetPointValue(Vector2f(3.6, -2.7)), 0.27);
}

