// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for tf2_polygon_msgs.hpp.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <cmath>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include <cras_geo_msgs/msg/polygon64.hpp>
#include <cras_geo_msgs/msg/polygon64_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_instance.hpp>
#include <cras_geo_msgs/msg/polygon64_instance_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_instance_stamped.hpp>
#include <cras_geo_msgs/msg/polygon64_instance_stamped_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_stamped.hpp>
#include <cras_geo_msgs/msg/polygon64_stamped_collection.hpp>
#include <cras_geo_msgs/msg/polygon_collection.hpp>
#include <cras_geo_msgs/msg/polygon_instance_collection.hpp>
#include <cras_geo_msgs/msg/polygon_instance_stamped_collection.hpp>
#include <cras_geo_msgs/msg/polygon_stamped_collection.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_instance.hpp>
#include <geometry_msgs/msg/polygon_instance_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <polygon_msgs/msg/complex_polygon2_d.hpp>
#include <polygon_msgs/msg/complex_polygon2_d_collection.hpp>
#include <polygon_msgs/msg/complex_polygon2_d_stamped.hpp>
#include <polygon_msgs/msg/point2_d.hpp>
#include <polygon_msgs/msg/polygon2_d.hpp>
#include <polygon_msgs/msg/polygon2_d_collection.hpp>
#include <polygon_msgs/msg/polygon2_d_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>

#include <cras_geo_tools/tf2_polygon_msgs.hpp>

namespace std_msgs::msg {
/**
 * \brief Print the given color so that a failing comparison shows the individual channels instead of raw bytes.
 * \param[in] color The color to print.
 * \param[out] os The stream to print to.
 */
void PrintTo(const ColorRGBA& color, std::ostream* os) {
  *os << "rgba(" << color.r << ", " << color.g << ", " << color.b << ", " << color.a << ')';
}
}  // namespace std_msgs::msg

namespace {

// Tolerances of the transformations. The 32-bit polygons can only store the results with limited precision.
constexpr double kTol = 1e-9;
constexpr double kTol32 = 1e-5;

// The frames of the test transform.
const char* kParentFrame = "base_link";
const char* kChildFrame = "odom";

// The vertices of the test polygons.
const std::vector<std::pair<double, double>> kPoints = {{1.0, 2.0}, {-3.0, 0.5}, {0.0, -4.0}};  // NOLINT

/**
 * \brief Create the transform used in the tests.
 *
 * The transform translates by (1, -2, -3) and rotates by 90 degrees around the Z-axis, which means that it maps
 * a point in the \f$z=0\f$ plane from coordinates \f$(x, y)\f$ to coordinates \f$(1 - y, x - 2)\f$.
 *
 * \return The transform.
 */
geometry_msgs::msg::TransformStamped createTestTf() {
  auto tf = geometry_msgs::msg::TransformStamped();
  tf.header.stamp.sec = 2;
  tf.header.frame_id = kParentFrame;
  tf.child_frame_id = kChildFrame;
  tf.transform.translation.x = 1.0;
  tf.transform.translation.y = -2.0;
  tf.transform.translation.z = -3.0;
  tf.transform.rotation.z = M_SQRT1_2;
  tf.transform.rotation.w = M_SQRT1_2;
  return tf;
}

/**
 * \brief Create a 2D polygon with the given vertices.
 * \param[in] points The vertices of the polygon.
 * \return The created polygon.
 */
polygon_msgs::msg::Polygon2D createPolygon2D(const std::vector<std::pair<double, double>>& points) {
  auto polygon = polygon_msgs::msg::Polygon2D();
  for (const auto& point : points) {
    auto& new_point = polygon.points.emplace_back();
    new_point.x = point.first;
    new_point.y = point.second;
  }
  return polygon;
}

/**
 * \brief Create a 3D polygon with the given vertices (all lying in the z=0 plane).
 * \param[in] points The vertices of the polygon.
 * \return The created polygon.
 */
geometry_msgs::msg::Polygon createPolygon(const std::vector<std::pair<double, double>>& points) {
  auto polygon = geometry_msgs::msg::Polygon();
  for (const auto& point : points) {
    auto& new_point = polygon.points.emplace_back();
    new_point.x = point.first;
    new_point.y = point.second;
  }
  return polygon;
}

/**
 * \brief Create a double-precision 3D polygon with the given vertices (all lying in the z=0 plane).
 * \param[in] points The vertices of the polygon.
 * \return The created polygon.
 */
cras_geo_msgs::msg::Polygon64 createPolygon64(const std::vector<std::pair<double, double>>& points) {
  auto polygon = cras_geo_msgs::msg::Polygon64();
  for (const auto& point : points) {
    auto& new_point = polygon.points.emplace_back();
    new_point.x = point.first;
    new_point.y = point.second;
  }
  return polygon;
}

/**
 * \brief Create per-polygon colors with the given number of items.
 * \param[in] num The number of colors.
 * \return The created colors.
 */
std::vector<std_msgs::msg::ColorRGBA> createColors(const size_t num) {
  auto colors = std::vector<std_msgs::msg::ColorRGBA>();
  for (size_t i = 0; i < num; ++i) {
    auto& color = colors.emplace_back();
    color.r = 1.0;
    color.g = static_cast<float>(i) / static_cast<float>(num);
    color.a = 1.0;
  }
  return colors;
}

}  // namespace

TEST(Tf2PolygonMsgs, Point2D) {  // NOLINT
  auto point = polygon_msgs::msg::Point2D();
  point.x = 3.0;
  point.y = -5.0;

  auto transformed = polygon_msgs::msg::Point2D();
  tf2::doTransform(point, transformed, createTestTf());

  EXPECT_NEAR(1.0 - (-5.0), transformed.x, kTol);
  EXPECT_NEAR(3.0 - 2.0, transformed.y, kTol);

  // the input must not be modified
  EXPECT_DOUBLE_EQ(3.0, point.x);
  EXPECT_DOUBLE_EQ(-5.0, point.y);
}

TEST(Tf2PolygonMsgs, IdentityTransform) {  // NOLINT
  auto point = polygon_msgs::msg::Point2D();
  point.x = 1.5;
  point.y = -2.5;

  auto transformed = polygon_msgs::msg::Point2D();
  tf2::doTransform(point, transformed, geometry_msgs::msg::TransformStamped());

  EXPECT_DOUBLE_EQ(point.x, transformed.x);
  EXPECT_DOUBLE_EQ(point.y, transformed.y);
}

TEST(Tf2PolygonMsgs, Polygon2D) {  // NOLINT
  const auto polygon = createPolygon2D(kPoints);
  ASSERT_EQ(3u, polygon.points.size());

  auto transformed = polygon_msgs::msg::Polygon2D();
  tf2::doTransform(polygon, transformed, createTestTf());

  ASSERT_EQ(3u, transformed.points.size());
  EXPECT_NEAR(1.0 - polygon.points[0].y, transformed.points[0].x, kTol);
  EXPECT_NEAR(polygon.points[0].x - 2.0, transformed.points[0].y, kTol);
  EXPECT_NEAR(1.0 - polygon.points[1].y, transformed.points[1].x, kTol);
  EXPECT_NEAR(polygon.points[1].x - 2.0, transformed.points[1].y, kTol);
  EXPECT_NEAR(1.0 - polygon.points[2].y, transformed.points[2].x, kTol);
  EXPECT_NEAR(polygon.points[2].x - 2.0, transformed.points[2].y, kTol);
}

TEST(Tf2PolygonMsgs, Polygon2DStamped) {  // NOLINT
  const auto tf = createTestTf();

  auto polygon = polygon_msgs::msg::Polygon2DStamped();
  polygon.header.stamp.sec = 100;
  polygon.header.frame_id = "some_frame";
  polygon.polygon = createPolygon2D(kPoints);

  auto transformed = polygon_msgs::msg::Polygon2DStamped();
  tf2::doTransform(polygon, transformed, tf);

  // the header is taken from the transform, not from the input message
  EXPECT_EQ(tf.header.stamp, transformed.header.stamp);
  EXPECT_EQ(kChildFrame, transformed.header.frame_id);

  ASSERT_EQ(3u, transformed.polygon.points.size());
  EXPECT_NEAR(1.0 - polygon.polygon.points[0].y, transformed.polygon.points[0].x, kTol);
  EXPECT_NEAR(polygon.polygon.points[0].x - 2.0, transformed.polygon.points[0].y, kTol);
  EXPECT_NEAR(1.0 - polygon.polygon.points[1].y, transformed.polygon.points[1].x, kTol);
  EXPECT_NEAR(polygon.polygon.points[1].x - 2.0, transformed.polygon.points[1].y, kTol);
  EXPECT_NEAR(1.0 - polygon.polygon.points[2].y, transformed.polygon.points[2].x, kTol);
  EXPECT_NEAR(polygon.polygon.points[2].x - 2.0, transformed.polygon.points[2].y, kTol);
}

TEST(Tf2PolygonMsgs, Polygon2DCollection) {  // NOLINT
  const auto tf = createTestTf();

  auto collection = polygon_msgs::msg::Polygon2DCollection();
  collection.header.stamp.sec = 100;
  collection.header.frame_id = "some_frame";
  collection.polygons = {createPolygon2D(kPoints), createPolygon2D({{0.0, 0.0}, {1.0, 1.0}})};
  collection.colors = createColors(2);

  auto transformed = polygon_msgs::msg::Polygon2DCollection();
  tf2::doTransform(collection, transformed, tf);

  EXPECT_EQ(tf.header.stamp, transformed.header.stamp);
  EXPECT_EQ(kChildFrame, transformed.header.frame_id);
  // colors are not transformed, they are only copied
  EXPECT_EQ(collection.colors, transformed.colors);
  ASSERT_EQ(2u, transformed.polygons.size());

  ASSERT_EQ(3u, transformed.polygons[0].points.size());
  EXPECT_NEAR(1.0 - collection.polygons[0].points[0].y, transformed.polygons[0].points[0].x, kTol);
  EXPECT_NEAR(collection.polygons[0].points[0].x - 2.0, transformed.polygons[0].points[0].y, kTol);
  EXPECT_NEAR(1.0 - collection.polygons[0].points[1].y, transformed.polygons[0].points[1].x, kTol);
  EXPECT_NEAR(collection.polygons[0].points[1].x - 2.0, transformed.polygons[0].points[1].y, kTol);
  EXPECT_NEAR(1.0 - collection.polygons[0].points[2].y, transformed.polygons[0].points[2].x, kTol);
  EXPECT_NEAR(collection.polygons[0].points[2].x - 2.0, transformed.polygons[0].points[2].y, kTol);

  ASSERT_EQ(2u, transformed.polygons[1].points.size());
  EXPECT_NEAR(1.0 - collection.polygons[1].points[0].y, transformed.polygons[1].points[0].x, kTol);
  EXPECT_NEAR(collection.polygons[1].points[0].x - 2.0, transformed.polygons[1].points[0].y, kTol);
  EXPECT_NEAR(1.0 - collection.polygons[1].points[1].y, transformed.polygons[1].points[1].x, kTol);
  EXPECT_NEAR(collection.polygons[1].points[1].x - 2.0, transformed.polygons[1].points[1].y, kTol);
}

TEST(Tf2PolygonMsgs, ComplexPolygon2D) {  // NOLINT
  auto polygon = polygon_msgs::msg::ComplexPolygon2D();
  polygon.outer = createPolygon2D(kPoints);
  polygon.inner = {createPolygon2D({{0.0, 0.0}, {1.0, 1.0}}), createPolygon2D({{0.5, 0.25}})};

  auto transformed = polygon_msgs::msg::ComplexPolygon2D();
  tf2::doTransform(polygon, transformed, createTestTf());

  ASSERT_EQ(3u, transformed.outer.points.size());
  EXPECT_NEAR(1.0 - polygon.outer.points[0].y, transformed.outer.points[0].x, kTol);
  EXPECT_NEAR(polygon.outer.points[0].x - 2.0, transformed.outer.points[0].y, kTol);
  EXPECT_NEAR(1.0 - polygon.outer.points[1].y, transformed.outer.points[1].x, kTol);
  EXPECT_NEAR(polygon.outer.points[1].x - 2.0, transformed.outer.points[1].y, kTol);
  EXPECT_NEAR(1.0 - polygon.outer.points[2].y, transformed.outer.points[2].x, kTol);
  EXPECT_NEAR(polygon.outer.points[2].x - 2.0, transformed.outer.points[2].y, kTol);

  ASSERT_EQ(2u, transformed.inner.size());

  ASSERT_EQ(2u, transformed.inner[0].points.size());
  EXPECT_NEAR(1.0 - polygon.inner[0].points[0].y, transformed.inner[0].points[0].x, kTol);
  EXPECT_NEAR(polygon.inner[0].points[0].x - 2.0, transformed.inner[0].points[0].y, kTol);
  EXPECT_NEAR(1.0 - polygon.inner[0].points[1].y, transformed.inner[0].points[1].x, kTol);
  EXPECT_NEAR(polygon.inner[0].points[1].x - 2.0, transformed.inner[0].points[1].y, kTol);

  ASSERT_EQ(1u, transformed.inner[1].points.size());
  EXPECT_NEAR(1.0 - polygon.inner[1].points[0].y, transformed.inner[1].points[0].x, kTol);
  EXPECT_NEAR(polygon.inner[1].points[0].x - 2.0, transformed.inner[1].points[0].y, kTol);
}

TEST(Tf2PolygonMsgs, ComplexPolygon2DStamped) {  // NOLINT
  const auto tf = createTestTf();

  auto polygon = polygon_msgs::msg::ComplexPolygon2DStamped();
  polygon.header.stamp.sec = 100;
  polygon.header.frame_id = "some_frame";
  polygon.polygon.outer = createPolygon2D(kPoints);
  polygon.polygon.inner = {createPolygon2D({{0.0, 0.0}, {1.0, 1.0}})};

  auto transformed = polygon_msgs::msg::ComplexPolygon2DStamped();
  tf2::doTransform(polygon, transformed, tf);

  EXPECT_EQ(tf.header.stamp, transformed.header.stamp);
  EXPECT_EQ(kChildFrame, transformed.header.frame_id);

  ASSERT_EQ(3u, transformed.polygon.outer.points.size());
  EXPECT_NEAR(1.0 - polygon.polygon.outer.points[0].y, transformed.polygon.outer.points[0].x, kTol);
  EXPECT_NEAR(polygon.polygon.outer.points[0].x - 2.0, transformed.polygon.outer.points[0].y, kTol);
  EXPECT_NEAR(1.0 - polygon.polygon.outer.points[1].y, transformed.polygon.outer.points[1].x, kTol);
  EXPECT_NEAR(polygon.polygon.outer.points[1].x - 2.0, transformed.polygon.outer.points[1].y, kTol);
  EXPECT_NEAR(1.0 - polygon.polygon.outer.points[2].y, transformed.polygon.outer.points[2].x, kTol);
  EXPECT_NEAR(polygon.polygon.outer.points[2].x - 2.0, transformed.polygon.outer.points[2].y, kTol);

  ASSERT_EQ(1u, transformed.polygon.inner.size());
  ASSERT_EQ(2u, transformed.polygon.inner[0].points.size());
  EXPECT_NEAR(1.0 - polygon.polygon.inner[0].points[0].y, transformed.polygon.inner[0].points[0].x, kTol);
  EXPECT_NEAR(polygon.polygon.inner[0].points[0].x - 2.0, transformed.polygon.inner[0].points[0].y, kTol);
  EXPECT_NEAR(1.0 - polygon.polygon.inner[0].points[1].y, transformed.polygon.inner[0].points[1].x, kTol);
  EXPECT_NEAR(polygon.polygon.inner[0].points[1].x - 2.0, transformed.polygon.inner[0].points[1].y, kTol);
}

TEST(Tf2PolygonMsgs, ComplexPolygon2DCollection) {  // NOLINT
  auto collection = polygon_msgs::msg::ComplexPolygon2DCollection();
  collection.header.stamp.sec = 100;
  collection.header.frame_id = "some_frame";
  collection.polygons = {polygon_msgs::msg::ComplexPolygon2D{}, polygon_msgs::msg::ComplexPolygon2D{}};
  collection.polygons[0].outer = createPolygon2D(kPoints);
  collection.polygons[1].outer = createPolygon2D({{0.0, 0.0}, {1.0, 1.0}});
  collection.polygons[1].inner = {createPolygon2D({{0.5, 0.25}})};
  collection.colors = createColors(2);

  const auto tf = createTestTf();

  auto transformed = polygon_msgs::msg::ComplexPolygon2DCollection();
  tf2::doTransform(collection, transformed, tf);

  EXPECT_EQ(tf.header.stamp, transformed.header.stamp);
  EXPECT_EQ(kChildFrame, transformed.header.frame_id);
  // colors are not transformed, they are only copied
  EXPECT_EQ(collection.colors, transformed.colors);
  ASSERT_EQ(2u, transformed.polygons.size());

  ASSERT_EQ(3u, transformed.polygons[0].outer.points.size());
  EXPECT_NEAR(1.0 - collection.polygons[0].outer.points[0].y, transformed.polygons[0].outer.points[0].x, kTol);
  EXPECT_NEAR(collection.polygons[0].outer.points[0].x - 2.0, transformed.polygons[0].outer.points[0].y, kTol);
  EXPECT_NEAR(1.0 - collection.polygons[0].outer.points[1].y, transformed.polygons[0].outer.points[1].x, kTol);
  EXPECT_NEAR(collection.polygons[0].outer.points[1].x - 2.0, transformed.polygons[0].outer.points[1].y, kTol);
  EXPECT_NEAR(1.0 - collection.polygons[0].outer.points[2].y, transformed.polygons[0].outer.points[2].x, kTol);
  EXPECT_NEAR(collection.polygons[0].outer.points[2].x - 2.0, transformed.polygons[0].outer.points[2].y, kTol);
  EXPECT_TRUE(transformed.polygons[0].inner.empty());

  ASSERT_EQ(2u, transformed.polygons[1].outer.points.size());
  EXPECT_NEAR(1.0 - collection.polygons[1].outer.points[0].y, transformed.polygons[1].outer.points[0].x, kTol);
  EXPECT_NEAR(collection.polygons[1].outer.points[0].x - 2.0, transformed.polygons[1].outer.points[0].y, kTol);
  EXPECT_NEAR(1.0 - collection.polygons[1].outer.points[1].y, transformed.polygons[1].outer.points[1].x, kTol);
  EXPECT_NEAR(collection.polygons[1].outer.points[1].x - 2.0, transformed.polygons[1].outer.points[1].y, kTol);

  ASSERT_EQ(1u, transformed.polygons[1].inner.size());
  ASSERT_EQ(1u, transformed.polygons[1].inner[0].points.size());
  EXPECT_NEAR(1.0 - collection.polygons[1].inner[0].points[0].y, transformed.polygons[1].inner[0].points[0].x, kTol);
  EXPECT_NEAR(collection.polygons[1].inner[0].points[0].x - 2.0, transformed.polygons[1].inner[0].points[0].y, kTol);
}

TEST(Tf2PolygonMsgs, PolygonInstance) {  // NOLINT
  auto polygon = geometry_msgs::msg::PolygonInstance();
  polygon.id = 42;
  polygon.polygon = createPolygon(kPoints);

  auto transformed = geometry_msgs::msg::PolygonInstance();
  tf2::doTransform(polygon, transformed, createTestTf());

  // the instance ID is preserved
  EXPECT_EQ(polygon.id, transformed.id);

  ASSERT_EQ(3u, transformed.polygon.points.size());
  EXPECT_NEAR(1.0 - polygon.polygon.points[0].y, transformed.polygon.points[0].x, kTol32);
  EXPECT_NEAR(polygon.polygon.points[0].x - 2.0, transformed.polygon.points[0].y, kTol32);
  EXPECT_NEAR(1.0 - polygon.polygon.points[1].y, transformed.polygon.points[1].x, kTol32);
  EXPECT_NEAR(polygon.polygon.points[1].x - 2.0, transformed.polygon.points[1].y, kTol32);
  EXPECT_NEAR(1.0 - polygon.polygon.points[2].y, transformed.polygon.points[2].x, kTol32);
  EXPECT_NEAR(polygon.polygon.points[2].x - 2.0, transformed.polygon.points[2].y, kTol32);
}

TEST(Tf2PolygonMsgs, PolygonInstanceStamped) {  // NOLINT
  auto polygon = geometry_msgs::msg::PolygonInstanceStamped();
  polygon.header.stamp.sec = 100;
  polygon.header.frame_id = "some_frame";
  polygon.polygon.id = 42;
  polygon.polygon.polygon = createPolygon(kPoints);

  const auto tf = createTestTf();

  auto transformed = geometry_msgs::msg::PolygonInstanceStamped();
  tf2::doTransform(polygon, transformed, tf);

  EXPECT_EQ(tf.header.stamp, transformed.header.stamp);
  EXPECT_EQ(kChildFrame, transformed.header.frame_id);
  EXPECT_EQ(polygon.polygon.id, transformed.polygon.id);

  ASSERT_EQ(3u, transformed.polygon.polygon.points.size());
  EXPECT_NEAR(1.0 - polygon.polygon.polygon.points[0].y, transformed.polygon.polygon.points[0].x, kTol32);
  EXPECT_NEAR(polygon.polygon.polygon.points[0].x - 2.0, transformed.polygon.polygon.points[0].y, kTol32);
  EXPECT_NEAR(1.0 - polygon.polygon.polygon.points[1].y, transformed.polygon.polygon.points[1].x, kTol32);
  EXPECT_NEAR(polygon.polygon.polygon.points[1].x - 2.0, transformed.polygon.polygon.points[1].y, kTol32);
  EXPECT_NEAR(1.0 - polygon.polygon.polygon.points[2].y, transformed.polygon.polygon.points[2].x, kTol32);
  EXPECT_NEAR(polygon.polygon.polygon.points[2].x - 2.0, transformed.polygon.polygon.points[2].y, kTol32);
}

TEST(Tf2PolygonMsgs, PolygonCollection) {  // NOLINT
  auto collection = cras_geo_msgs::msg::PolygonCollection();
  collection.header.stamp.sec = 100;
  collection.header.frame_id = "some_frame";
  collection.polygons = {createPolygon(kPoints), createPolygon({{0.0, 0.0}, {1.0, 1.0}})};
  collection.colors = createColors(2);

  const auto tf = createTestTf();

  auto transformed = cras_geo_msgs::msg::PolygonCollection();
  tf2::doTransform(collection, transformed, tf);

  EXPECT_EQ(tf.header.stamp, transformed.header.stamp);
  EXPECT_EQ(kChildFrame, transformed.header.frame_id);
  // colors are not transformed, they are only copied
  EXPECT_EQ(collection.colors, transformed.colors);
  ASSERT_EQ(2u, transformed.polygons.size());

  ASSERT_EQ(3u, transformed.polygons[0].points.size());
  EXPECT_NEAR(1.0 - collection.polygons[0].points[0].y, transformed.polygons[0].points[0].x, kTol32);
  EXPECT_NEAR(collection.polygons[0].points[0].x - 2.0, transformed.polygons[0].points[0].y, kTol32);
  EXPECT_NEAR(1.0 - collection.polygons[0].points[1].y, transformed.polygons[0].points[1].x, kTol32);
  EXPECT_NEAR(collection.polygons[0].points[1].x - 2.0, transformed.polygons[0].points[1].y, kTol32);
  EXPECT_NEAR(1.0 - collection.polygons[0].points[2].y, transformed.polygons[0].points[2].x, kTol32);
  EXPECT_NEAR(collection.polygons[0].points[2].x - 2.0, transformed.polygons[0].points[2].y, kTol32);

  ASSERT_EQ(2u, transformed.polygons[1].points.size());
  EXPECT_NEAR(1.0 - collection.polygons[1].points[0].y, transformed.polygons[1].points[0].x, kTol32);
  EXPECT_NEAR(collection.polygons[1].points[0].x - 2.0, transformed.polygons[1].points[0].y, kTol32);
  EXPECT_NEAR(1.0 - collection.polygons[1].points[1].y, transformed.polygons[1].points[1].x, kTol32);
  EXPECT_NEAR(collection.polygons[1].points[1].x - 2.0, transformed.polygons[1].points[1].y, kTol32);
}

TEST(Tf2PolygonMsgs, PolygonInstanceCollection) {  // NOLINT
  auto collection = cras_geo_msgs::msg::PolygonInstanceCollection();
  collection.header.stamp.sec = 100;
  collection.header.frame_id = "some_frame";
  collection.polygons = {geometry_msgs::msg::PolygonInstance{}, geometry_msgs::msg::PolygonInstance{}};
  collection.polygons[0].id = 8;
  collection.polygons[0].polygon = createPolygon(kPoints);
  collection.polygons[1].id = 9;
  collection.polygons[1].polygon = createPolygon({{0.0, 0.0}, {1.0, 1.0}});
  collection.colors = createColors(2);

  const auto tf = createTestTf();

  auto transformed = cras_geo_msgs::msg::PolygonInstanceCollection();
  tf2::doTransform(collection, transformed, tf);

  EXPECT_EQ(tf.header.stamp, transformed.header.stamp);
  EXPECT_EQ(kChildFrame, transformed.header.frame_id);
  EXPECT_EQ(collection.colors, transformed.colors);
  ASSERT_EQ(2u, transformed.polygons.size());
  EXPECT_EQ(8u, transformed.polygons[0].id);
  EXPECT_EQ(9u, transformed.polygons[1].id);

  ASSERT_EQ(3u, transformed.polygons[0].polygon.points.size());
  EXPECT_NEAR(1.0 - collection.polygons[0].polygon.points[0].y, transformed.polygons[0].polygon.points[0].x, kTol32);
  EXPECT_NEAR(collection.polygons[0].polygon.points[0].x - 2.0, transformed.polygons[0].polygon.points[0].y, kTol32);
  EXPECT_NEAR(1.0 - collection.polygons[0].polygon.points[1].y, transformed.polygons[0].polygon.points[1].x, kTol32);
  EXPECT_NEAR(collection.polygons[0].polygon.points[1].x - 2.0, transformed.polygons[0].polygon.points[1].y, kTol32);
  EXPECT_NEAR(1.0 - collection.polygons[0].polygon.points[2].y, transformed.polygons[0].polygon.points[2].x, kTol32);
  EXPECT_NEAR(collection.polygons[0].polygon.points[2].x - 2.0, transformed.polygons[0].polygon.points[2].y, kTol32);

  ASSERT_EQ(2u, transformed.polygons[1].polygon.points.size());
  EXPECT_NEAR(1.0 - collection.polygons[1].polygon.points[0].y, transformed.polygons[1].polygon.points[0].x, kTol32);
  EXPECT_NEAR(collection.polygons[1].polygon.points[0].x - 2.0, transformed.polygons[1].polygon.points[0].y, kTol32);
  EXPECT_NEAR(1.0 - collection.polygons[1].polygon.points[1].y, transformed.polygons[1].polygon.points[1].x, kTol32);
  EXPECT_NEAR(collection.polygons[1].polygon.points[1].x - 2.0, transformed.polygons[1].polygon.points[1].y, kTol32);
}

TEST(Tf2PolygonMsgs, PolygonStampedCollection) {  // NOLINT
  auto collection = cras_geo_msgs::msg::PolygonStampedCollection();
  collection.header.stamp.sec = 100;
  collection.header.frame_id = "some_frame";
  collection.polygons = {geometry_msgs::msg::PolygonStamped{}, geometry_msgs::msg::PolygonStamped{}};
  collection.polygons[0].polygon = createPolygon(kPoints);
  collection.polygons[1].polygon = createPolygon({{0.0, 0.0}, {1.0, 1.0}});
  collection.colors = createColors(2);

  const auto tf = createTestTf();

  auto transformed = cras_geo_msgs::msg::PolygonStampedCollection();
  tf2::doTransform(collection, transformed, tf);

  EXPECT_EQ(tf.header.stamp, transformed.header.stamp);
  EXPECT_EQ(kChildFrame, transformed.header.frame_id);
  EXPECT_EQ(collection.colors, transformed.colors);
  ASSERT_EQ(2u, transformed.polygons.size());

  ASSERT_EQ(3u, transformed.polygons[0].polygon.points.size());
  EXPECT_NEAR(1.0 - collection.polygons[0].polygon.points[0].y, transformed.polygons[0].polygon.points[0].x, kTol32);
  EXPECT_NEAR(collection.polygons[0].polygon.points[0].x - 2.0, transformed.polygons[0].polygon.points[0].y, kTol32);
  EXPECT_NEAR(1.0 - collection.polygons[0].polygon.points[1].y, transformed.polygons[0].polygon.points[1].x, kTol32);
  EXPECT_NEAR(collection.polygons[0].polygon.points[1].x - 2.0, transformed.polygons[0].polygon.points[1].y, kTol32);
  EXPECT_NEAR(1.0 - collection.polygons[0].polygon.points[2].y, transformed.polygons[0].polygon.points[2].x, kTol32);
  EXPECT_NEAR(collection.polygons[0].polygon.points[2].x - 2.0, transformed.polygons[0].polygon.points[2].y, kTol32);

  ASSERT_EQ(2u, transformed.polygons[1].polygon.points.size());
  EXPECT_NEAR(1.0 - collection.polygons[1].polygon.points[0].y, transformed.polygons[1].polygon.points[0].x, kTol32);
  EXPECT_NEAR(collection.polygons[1].polygon.points[0].x - 2.0, transformed.polygons[1].polygon.points[0].y, kTol32);
  EXPECT_NEAR(1.0 - collection.polygons[1].polygon.points[1].y, transformed.polygons[1].polygon.points[1].x, kTol32);
  EXPECT_NEAR(collection.polygons[1].polygon.points[1].x - 2.0, transformed.polygons[1].polygon.points[1].y, kTol32);
}

TEST(Tf2PolygonMsgs, PolygonInstanceStampedCollection) {  // NOLINT
  auto collection = cras_geo_msgs::msg::PolygonInstanceStampedCollection();
  collection.header.stamp.sec = 100;
  collection.header.frame_id = "some_frame";
  collection.polygons = {
    geometry_msgs::msg::PolygonInstanceStamped{}, geometry_msgs::msg::PolygonInstanceStamped{}};
  collection.polygons[0].polygon.id = 8;
  collection.polygons[0].polygon.polygon = createPolygon(kPoints);
  collection.polygons[1].polygon.id = 9;
  collection.polygons[1].polygon.polygon = createPolygon({{0.0, 0.0}, {1.0, 1.0}});
  collection.colors = createColors(2);

  const auto tf = createTestTf();

  auto transformed = cras_geo_msgs::msg::PolygonInstanceStampedCollection();
  tf2::doTransform(collection, transformed, tf);

  EXPECT_EQ(tf.header.stamp, transformed.header.stamp);
  EXPECT_EQ(kChildFrame, transformed.header.frame_id);
  EXPECT_EQ(collection.colors, transformed.colors);
  ASSERT_EQ(2u, transformed.polygons.size());
  EXPECT_EQ(8u, transformed.polygons[0].polygon.id);
  EXPECT_EQ(9u, transformed.polygons[1].polygon.id);

  ASSERT_EQ(3u, transformed.polygons[0].polygon.polygon.points.size());
  EXPECT_NEAR(
    1.0 - collection.polygons[0].polygon.polygon.points[0].y,
    transformed.polygons[0].polygon.polygon.points[0].x, kTol32);
  EXPECT_NEAR(
    collection.polygons[0].polygon.polygon.points[0].x - 2.0,
    transformed.polygons[0].polygon.polygon.points[0].y, kTol32);
  EXPECT_NEAR(
    1.0 - collection.polygons[0].polygon.polygon.points[1].y,
    transformed.polygons[0].polygon.polygon.points[1].x, kTol32);
  EXPECT_NEAR(
    collection.polygons[0].polygon.polygon.points[1].x - 2.0,
    transformed.polygons[0].polygon.polygon.points[1].y, kTol32);
  EXPECT_NEAR(
    1.0 - collection.polygons[0].polygon.polygon.points[2].y,
    transformed.polygons[0].polygon.polygon.points[2].x, kTol32);
  EXPECT_NEAR(
    collection.polygons[0].polygon.polygon.points[2].x - 2.0,
    transformed.polygons[0].polygon.polygon.points[2].y, kTol32);

  ASSERT_EQ(2u, transformed.polygons[1].polygon.polygon.points.size());
  EXPECT_NEAR(
    1.0 - collection.polygons[1].polygon.polygon.points[0].y,
    transformed.polygons[1].polygon.polygon.points[0].x, kTol32);
  EXPECT_NEAR(
    collection.polygons[1].polygon.polygon.points[0].x - 2.0,
    transformed.polygons[1].polygon.polygon.points[0].y, kTol32);
  EXPECT_NEAR(
    1.0 - collection.polygons[1].polygon.polygon.points[1].y,
    transformed.polygons[1].polygon.polygon.points[1].x, kTol32);
  EXPECT_NEAR(
    collection.polygons[1].polygon.polygon.points[1].x - 2.0,
    transformed.polygons[1].polygon.polygon.points[1].y, kTol32);
}

TEST(Tf2PolygonMsgs, Polygon64) {  // NOLINT
  const auto polygon = createPolygon64(kPoints);
  ASSERT_EQ(3u, polygon.points.size());

  auto transformed = cras_geo_msgs::msg::Polygon64();
  tf2::doTransform(polygon, transformed, createTestTf());

  ASSERT_EQ(3u, transformed.points.size());
  EXPECT_NEAR(1.0 - polygon.points[0].y, transformed.points[0].x, kTol);
  EXPECT_NEAR(polygon.points[0].x - 2.0, transformed.points[0].y, kTol);
  // the points are transformed in 3D, so their Z-coordinate is translated as well
  EXPECT_NEAR(-3.0, transformed.points[0].z, kTol);
  EXPECT_NEAR(1.0 - polygon.points[1].y, transformed.points[1].x, kTol);
  EXPECT_NEAR(polygon.points[1].x - 2.0, transformed.points[1].y, kTol);
  EXPECT_NEAR(-3.0, transformed.points[1].z, kTol);
  EXPECT_NEAR(1.0 - polygon.points[2].y, transformed.points[2].x, kTol);
  EXPECT_NEAR(polygon.points[2].x - 2.0, transformed.points[2].y, kTol);
  EXPECT_NEAR(-3.0, transformed.points[2].z, kTol);

  // the input must not be modified
  EXPECT_DOUBLE_EQ(1.0, polygon.points[0].x);
  EXPECT_DOUBLE_EQ(2.0, polygon.points[0].y);
}

TEST(Tf2PolygonMsgs, Polygon64Stamped) {  // NOLINT
  const auto tf = createTestTf();

  auto polygon = cras_geo_msgs::msg::Polygon64Stamped();
  polygon.header.stamp.sec = 100;
  polygon.header.frame_id = "some_frame";
  polygon.polygon = createPolygon64(kPoints);

  auto transformed = cras_geo_msgs::msg::Polygon64Stamped();
  tf2::doTransform(polygon, transformed, tf);

  // unlike the other stamped messages, this specialization keeps the header of the input message
  EXPECT_EQ(polygon.header, transformed.header);

  ASSERT_EQ(3u, transformed.polygon.points.size());
  EXPECT_NEAR(1.0 - polygon.polygon.points[0].y, transformed.polygon.points[0].x, kTol);
  EXPECT_NEAR(polygon.polygon.points[0].x - 2.0, transformed.polygon.points[0].y, kTol);
  EXPECT_NEAR(1.0 - polygon.polygon.points[1].y, transformed.polygon.points[1].x, kTol);
  EXPECT_NEAR(polygon.polygon.points[1].x - 2.0, transformed.polygon.points[1].y, kTol);
  EXPECT_NEAR(1.0 - polygon.polygon.points[2].y, transformed.polygon.points[2].x, kTol);
  EXPECT_NEAR(polygon.polygon.points[2].x - 2.0, transformed.polygon.points[2].y, kTol);
}

TEST(Tf2PolygonMsgs, Polygon64Instance) {  // NOLINT
  auto polygon = cras_geo_msgs::msg::Polygon64Instance();
  polygon.id = 42;
  polygon.polygon = createPolygon64(kPoints);

  auto transformed = cras_geo_msgs::msg::Polygon64Instance();
  tf2::doTransform(polygon, transformed, createTestTf());

  // the instance ID is preserved
  EXPECT_EQ(polygon.id, transformed.id);

  ASSERT_EQ(3u, transformed.polygon.points.size());
  EXPECT_NEAR(1.0 - polygon.polygon.points[0].y, transformed.polygon.points[0].x, kTol);
  EXPECT_NEAR(polygon.polygon.points[0].x - 2.0, transformed.polygon.points[0].y, kTol);
  EXPECT_NEAR(1.0 - polygon.polygon.points[1].y, transformed.polygon.points[1].x, kTol);
  EXPECT_NEAR(polygon.polygon.points[1].x - 2.0, transformed.polygon.points[1].y, kTol);
  EXPECT_NEAR(1.0 - polygon.polygon.points[2].y, transformed.polygon.points[2].x, kTol);
  EXPECT_NEAR(polygon.polygon.points[2].x - 2.0, transformed.polygon.points[2].y, kTol);
}

TEST(Tf2PolygonMsgs, Polygon64InstanceStamped) {  // NOLINT
  const auto tf = createTestTf();

  auto polygon = cras_geo_msgs::msg::Polygon64InstanceStamped();
  polygon.header.stamp.sec = 100;
  polygon.header.frame_id = "some_frame";
  polygon.polygon.id = 42;
  polygon.polygon.polygon = createPolygon64(kPoints);

  auto transformed = cras_geo_msgs::msg::Polygon64InstanceStamped();
  tf2::doTransform(polygon, transformed, tf);

  // unlike the other stamped messages, this specialization keeps the header of the input message
  EXPECT_EQ(polygon.header, transformed.header);
  EXPECT_EQ(polygon.polygon.id, transformed.polygon.id);

  ASSERT_EQ(3u, transformed.polygon.polygon.points.size());
  EXPECT_NEAR(1.0 - polygon.polygon.polygon.points[0].y, transformed.polygon.polygon.points[0].x, kTol);
  EXPECT_NEAR(polygon.polygon.polygon.points[0].x - 2.0, transformed.polygon.polygon.points[0].y, kTol);
  EXPECT_NEAR(1.0 - polygon.polygon.polygon.points[1].y, transformed.polygon.polygon.points[1].x, kTol);
  EXPECT_NEAR(polygon.polygon.polygon.points[1].x - 2.0, transformed.polygon.polygon.points[1].y, kTol);
  EXPECT_NEAR(1.0 - polygon.polygon.polygon.points[2].y, transformed.polygon.polygon.points[2].x, kTol);
  EXPECT_NEAR(polygon.polygon.polygon.points[2].x - 2.0, transformed.polygon.polygon.points[2].y, kTol);
}

TEST(Tf2PolygonMsgs, Polygon64Collection) {  // NOLINT
  auto collection = cras_geo_msgs::msg::Polygon64Collection();
  collection.header.stamp.sec = 100;
  collection.header.frame_id = "some_frame";
  collection.polygons = {createPolygon64(kPoints), createPolygon64({{0.0, 0.0}, {1.0, 1.0}})};
  collection.colors = createColors(2);

  const auto tf = createTestTf();

  auto transformed = cras_geo_msgs::msg::Polygon64Collection();
  tf2::doTransform(collection, transformed, tf);

  EXPECT_EQ(tf.header.stamp, transformed.header.stamp);
  EXPECT_EQ(kChildFrame, transformed.header.frame_id);
  // colors are not transformed, they are only copied
  EXPECT_EQ(collection.colors, transformed.colors);
  ASSERT_EQ(2u, transformed.polygons.size());

  ASSERT_EQ(3u, transformed.polygons[0].points.size());
  EXPECT_NEAR(1.0 - collection.polygons[0].points[0].y, transformed.polygons[0].points[0].x, kTol);
  EXPECT_NEAR(collection.polygons[0].points[0].x - 2.0, transformed.polygons[0].points[0].y, kTol);
  EXPECT_NEAR(1.0 - collection.polygons[0].points[1].y, transformed.polygons[0].points[1].x, kTol);
  EXPECT_NEAR(collection.polygons[0].points[1].x - 2.0, transformed.polygons[0].points[1].y, kTol);
  EXPECT_NEAR(1.0 - collection.polygons[0].points[2].y, transformed.polygons[0].points[2].x, kTol);
  EXPECT_NEAR(collection.polygons[0].points[2].x - 2.0, transformed.polygons[0].points[2].y, kTol);

  ASSERT_EQ(2u, transformed.polygons[1].points.size());
  EXPECT_NEAR(1.0 - collection.polygons[1].points[0].y, transformed.polygons[1].points[0].x, kTol);
  EXPECT_NEAR(collection.polygons[1].points[0].x - 2.0, transformed.polygons[1].points[0].y, kTol);
  EXPECT_NEAR(1.0 - collection.polygons[1].points[1].y, transformed.polygons[1].points[1].x, kTol);
  EXPECT_NEAR(collection.polygons[1].points[1].x - 2.0, transformed.polygons[1].points[1].y, kTol);
}

TEST(Tf2PolygonMsgs, Polygon64InstanceCollection) {  // NOLINT
  auto collection = cras_geo_msgs::msg::Polygon64InstanceCollection();
  collection.header.stamp.sec = 100;
  collection.header.frame_id = "some_frame";
  collection.polygons = {cras_geo_msgs::msg::Polygon64Instance{}, cras_geo_msgs::msg::Polygon64Instance{}};
  collection.polygons[0].id = 8;
  collection.polygons[0].polygon = createPolygon64(kPoints);
  collection.polygons[1].id = 9;
  collection.polygons[1].polygon = createPolygon64({{0.0, 0.0}, {1.0, 1.0}});
  collection.colors = createColors(2);

  const auto tf = createTestTf();

  auto transformed = cras_geo_msgs::msg::Polygon64InstanceCollection();
  tf2::doTransform(collection, transformed, tf);

  EXPECT_EQ(tf.header.stamp, transformed.header.stamp);
  EXPECT_EQ(kChildFrame, transformed.header.frame_id);
  EXPECT_EQ(collection.colors, transformed.colors);
  ASSERT_EQ(2u, transformed.polygons.size());
  EXPECT_EQ(8, transformed.polygons[0].id);
  EXPECT_EQ(9, transformed.polygons[1].id);

  ASSERT_EQ(3u, transformed.polygons[0].polygon.points.size());
  EXPECT_NEAR(1.0 - collection.polygons[0].polygon.points[0].y, transformed.polygons[0].polygon.points[0].x, kTol);
  EXPECT_NEAR(collection.polygons[0].polygon.points[0].x - 2.0, transformed.polygons[0].polygon.points[0].y, kTol);
  EXPECT_NEAR(1.0 - collection.polygons[0].polygon.points[1].y, transformed.polygons[0].polygon.points[1].x, kTol);
  EXPECT_NEAR(collection.polygons[0].polygon.points[1].x - 2.0, transformed.polygons[0].polygon.points[1].y, kTol);
  EXPECT_NEAR(1.0 - collection.polygons[0].polygon.points[2].y, transformed.polygons[0].polygon.points[2].x, kTol);
  EXPECT_NEAR(collection.polygons[0].polygon.points[2].x - 2.0, transformed.polygons[0].polygon.points[2].y, kTol);

  ASSERT_EQ(2u, transformed.polygons[1].polygon.points.size());
  EXPECT_NEAR(1.0 - collection.polygons[1].polygon.points[0].y, transformed.polygons[1].polygon.points[0].x, kTol);
  EXPECT_NEAR(collection.polygons[1].polygon.points[0].x - 2.0, transformed.polygons[1].polygon.points[0].y, kTol);
  EXPECT_NEAR(1.0 - collection.polygons[1].polygon.points[1].y, transformed.polygons[1].polygon.points[1].x, kTol);
  EXPECT_NEAR(collection.polygons[1].polygon.points[1].x - 2.0, transformed.polygons[1].polygon.points[1].y, kTol);
}

TEST(Tf2PolygonMsgs, Polygon64StampedCollection) {  // NOLINT
  auto collection = cras_geo_msgs::msg::Polygon64StampedCollection();
  collection.header.stamp.sec = 100;
  collection.header.frame_id = "some_frame";
  collection.polygons = {cras_geo_msgs::msg::Polygon64Stamped{}, cras_geo_msgs::msg::Polygon64Stamped{}};
  collection.polygons[0].polygon = createPolygon64(kPoints);
  collection.polygons[1].polygon = createPolygon64({{0.0, 0.0}, {1.0, 1.0}});
  collection.colors = createColors(2);

  const auto tf = createTestTf();

  auto transformed = cras_geo_msgs::msg::Polygon64StampedCollection();
  tf2::doTransform(collection, transformed, tf);

  EXPECT_EQ(tf.header.stamp, transformed.header.stamp);
  EXPECT_EQ(kChildFrame, transformed.header.frame_id);
  EXPECT_EQ(collection.colors, transformed.colors);
  ASSERT_EQ(2u, transformed.polygons.size());

  ASSERT_EQ(3u, transformed.polygons[0].polygon.points.size());
  EXPECT_NEAR(1.0 - collection.polygons[0].polygon.points[0].y, transformed.polygons[0].polygon.points[0].x, kTol);
  EXPECT_NEAR(collection.polygons[0].polygon.points[0].x - 2.0, transformed.polygons[0].polygon.points[0].y, kTol);
  EXPECT_NEAR(1.0 - collection.polygons[0].polygon.points[1].y, transformed.polygons[0].polygon.points[1].x, kTol);
  EXPECT_NEAR(collection.polygons[0].polygon.points[1].x - 2.0, transformed.polygons[0].polygon.points[1].y, kTol);
  EXPECT_NEAR(1.0 - collection.polygons[0].polygon.points[2].y, transformed.polygons[0].polygon.points[2].x, kTol);
  EXPECT_NEAR(collection.polygons[0].polygon.points[2].x - 2.0, transformed.polygons[0].polygon.points[2].y, kTol);

  ASSERT_EQ(2u, transformed.polygons[1].polygon.points.size());
  EXPECT_NEAR(1.0 - collection.polygons[1].polygon.points[0].y, transformed.polygons[1].polygon.points[0].x, kTol);
  EXPECT_NEAR(collection.polygons[1].polygon.points[0].x - 2.0, transformed.polygons[1].polygon.points[0].y, kTol);
  EXPECT_NEAR(1.0 - collection.polygons[1].polygon.points[1].y, transformed.polygons[1].polygon.points[1].x, kTol);
  EXPECT_NEAR(collection.polygons[1].polygon.points[1].x - 2.0, transformed.polygons[1].polygon.points[1].y, kTol);
}

TEST(Tf2PolygonMsgs, Polygon64InstanceStampedCollection) {  // NOLINT
  auto collection = cras_geo_msgs::msg::Polygon64InstanceStampedCollection();
  collection.header.stamp.sec = 100;
  collection.header.frame_id = "some_frame";
  collection.polygons = {
    cras_geo_msgs::msg::Polygon64InstanceStamped{}, cras_geo_msgs::msg::Polygon64InstanceStamped{}};
  collection.polygons[0].polygon.id = 8;
  collection.polygons[0].polygon.polygon = createPolygon64(kPoints);
  collection.polygons[1].polygon.id = 9;
  collection.polygons[1].polygon.polygon = createPolygon64({{0.0, 0.0}, {1.0, 1.0}});
  collection.colors = createColors(2);

  const auto tf = createTestTf();

  auto transformed = cras_geo_msgs::msg::Polygon64InstanceStampedCollection();
  tf2::doTransform(collection, transformed, tf);

  EXPECT_EQ(tf.header.stamp, transformed.header.stamp);
  EXPECT_EQ(kChildFrame, transformed.header.frame_id);
  EXPECT_EQ(collection.colors, transformed.colors);
  ASSERT_EQ(2u, transformed.polygons.size());
  EXPECT_EQ(8, transformed.polygons[0].polygon.id);
  EXPECT_EQ(9, transformed.polygons[1].polygon.id);

  ASSERT_EQ(3u, transformed.polygons[0].polygon.polygon.points.size());
  EXPECT_NEAR(
    1.0 - collection.polygons[0].polygon.polygon.points[0].y,
    transformed.polygons[0].polygon.polygon.points[0].x, kTol);
  EXPECT_NEAR(
    collection.polygons[0].polygon.polygon.points[0].x - 2.0,
    transformed.polygons[0].polygon.polygon.points[0].y, kTol);
  EXPECT_NEAR(
    1.0 - collection.polygons[0].polygon.polygon.points[1].y,
    transformed.polygons[0].polygon.polygon.points[1].x, kTol);
  EXPECT_NEAR(
    collection.polygons[0].polygon.polygon.points[1].x - 2.0,
    transformed.polygons[0].polygon.polygon.points[1].y, kTol);
  EXPECT_NEAR(
    1.0 - collection.polygons[0].polygon.polygon.points[2].y,
    transformed.polygons[0].polygon.polygon.points[2].x, kTol);
  EXPECT_NEAR(
    collection.polygons[0].polygon.polygon.points[2].x - 2.0,
    transformed.polygons[0].polygon.polygon.points[2].y, kTol);

  ASSERT_EQ(2u, transformed.polygons[1].polygon.polygon.points.size());
  EXPECT_NEAR(
    1.0 - collection.polygons[1].polygon.polygon.points[0].y,
    transformed.polygons[1].polygon.polygon.points[0].x, kTol);
  EXPECT_NEAR(
    collection.polygons[1].polygon.polygon.points[0].x - 2.0,
    transformed.polygons[1].polygon.polygon.points[0].y, kTol);
  EXPECT_NEAR(
    1.0 - collection.polygons[1].polygon.polygon.points[1].y,
    transformed.polygons[1].polygon.polygon.points[1].x, kTol);
  EXPECT_NEAR(
    collection.polygons[1].polygon.polygon.points[1].x - 2.0,
    transformed.polygons[1].polygon.polygon.points[1].y, kTol);
}

TEST(Tf2PolygonMsgs, EmptyPolygons) {  // NOLINT
  auto collection = cras_geo_msgs::msg::PolygonCollection();
  collection.polygons.emplace_back();

  auto transformed = cras_geo_msgs::msg::PolygonCollection();
  tf2::doTransform(collection, transformed, createTestTf());

  ASSERT_EQ(1u, transformed.polygons.size());
  EXPECT_TRUE(transformed.polygons[0].points.empty());
}

TEST(Tf2PolygonMsgs, EmptyPolygonsPolygon64) {  // NOLINT
  auto collection = cras_geo_msgs::msg::Polygon64Collection();
  collection.polygons.emplace_back();

  auto transformed = cras_geo_msgs::msg::Polygon64Collection();
  tf2::doTransform(collection, transformed, createTestTf());

  ASSERT_EQ(1u, transformed.polygons.size());
  EXPECT_TRUE(transformed.polygons[0].points.empty());

  auto polygon = cras_geo_msgs::msg::Polygon64Stamped();
  polygon.header.stamp.sec = 100;
  polygon.header.frame_id = "some_frame";

  auto transformedPolygon = cras_geo_msgs::msg::Polygon64Stamped();
  tf2::doTransform(polygon, transformedPolygon, createTestTf());

  EXPECT_TRUE(transformedPolygon.polygon.points.empty());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
