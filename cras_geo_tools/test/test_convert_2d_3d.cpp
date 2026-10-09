// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for convert_2d_3d.hpp.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <cmath>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <cras_cpp_common/expected.hpp>
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
#include <cras_geo_tools/convert_2d_3d.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_instance.hpp>
#include <geometry_msgs/msg/polygon_instance_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <polygon_msgs/msg/polygon2_d.hpp>
#include <polygon_msgs/msg/polygon2_d_collection.hpp>
#include <polygon_msgs/msg/polygon2_d_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_ros/buffer.hpp>

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

// Tolerance of the coordinate comparisons. The 3D polygons store the coordinates in single precision.
constexpr double kTol = 1e-6;

// The frames used in the tests. The polygons are in the polygon frame, the collections in the collection frame.
const char* kPolygonFrame = "polygon_frame";
const char* kCollectionFrame = "collection_frame";

/**
 * \brief A vertex of the test polygons.
 */
struct Vertex {
  double x;  //!< The X-coordinate.
  double y;  //!< The Y-coordinate.
  double z;  //!< The Z-coordinate.
};

// A flat polygon lying in the plane z=1.5.
const std::vector<Vertex> kFlatVertices = {{1.0, 2.0, 1.5}, {-3.0, 0.5, 1.5}, {0.0, -4.0, 1.5}};  // NOLINT

// A flat polygon lying in a different plane than the previous one.
const std::vector<Vertex> kOtherFlatVertices = {{1.0, 2.0, 2.5}, {-3.0, 0.5, 2.5}};  // NOLINT

// A polygon which is not flat (its vertices differ in the Z-coordinate by 0.1).
const std::vector<Vertex> kNonFlatVertices = {{1.0, 2.0, 0.0}, {-3.0, 0.5, 0.1}, {0.0, -4.0, 0.0}};  // NOLINT

// The vertices used by the tests which transform the polygons to a different frame.
const std::vector<Vertex> kTransformedVertices = {{1.0, 0.0, 3.0}, {0.0, 1.0, 3.0}, {2.0, 2.0, 3.0}};  // NOLINT

/**
 * \brief Create a timestamp with the given values.
 * \param[in] sec Seconds.
 * \param[in] nanosec Nanoseconds.
 * \return The timestamp.
 */
builtin_interfaces::msg::Time createTime(const int32_t sec, const uint32_t nanosec = 0) {
  auto time = builtin_interfaces::msg::Time();
  time.sec = sec;
  time.nanosec = nanosec;
  return time;
}

/**
 * \brief Create a 2D polygon with the given vertices.
 * \param[in] vertices The vertices of the polygon.
 * \return The created polygon.
 */
polygon_msgs::msg::Polygon2D createPolygon2D(const std::vector<Vertex>& vertices) {
  auto polygon = polygon_msgs::msg::Polygon2D();
  for (const auto& vertex : vertices) {
    auto& point = polygon.points.emplace_back();
    point.x = vertex.x;
    point.y = vertex.y;
  }
  return polygon;
}

/**
 * \brief Create a 3D polygon with the given vertices.
 * \param[in] vertices The vertices of the polygon.
 * \return The created polygon.
 */
geometry_msgs::msg::Polygon createPolygon3D(const std::vector<Vertex>& vertices) {
  auto polygon = geometry_msgs::msg::Polygon();
  for (const auto& vertex : vertices) {
    auto& point = polygon.points.emplace_back();
    point.x = vertex.x;
    point.y = vertex.y;
    point.z = vertex.z;
  }
  return polygon;
}

/**
 * \brief Create a 3D polygon with double-precision vertices.
 * \param[in] vertices The vertices of the polygon.
 * \return The created polygon.
 */
cras_geo_msgs::msg::Polygon64 createPolygon64(const std::vector<Vertex>& vertices) {
  auto polygon = cras_geo_msgs::msg::Polygon64();
  for (const auto& vertex : vertices) {
    auto& point = polygon.points.emplace_back();
    point.x = vertex.x;
    point.y = vertex.y;
    point.z = vertex.z;
  }
  return polygon;
}

/**
 * \brief Create a stamped 2D polygon.
 * \param[in] frame_id The frame of the polygon.
 * \param[in] stamp The timestamp of the polygon.
 * \param[in] vertices The vertices of the polygon.
 * \return The created polygon.
 */
polygon_msgs::msg::Polygon2DStamped createPolygon2DStamped(
    const std::string& frame_id, const builtin_interfaces::msg::Time& stamp, const std::vector<Vertex>& vertices) {
  auto polygon = polygon_msgs::msg::Polygon2DStamped();
  polygon.header.frame_id = frame_id;
  polygon.header.stamp = stamp;
  polygon.polygon = createPolygon2D(vertices);
  return polygon;
}

/**
 * \brief Create a stamped 3D polygon.
 * \param[in] frame_id The frame of the polygon.
 * \param[in] stamp The timestamp of the polygon.
 * \param[in] vertices The vertices of the polygon.
 * \return The created polygon.
 */
geometry_msgs::msg::PolygonStamped createPolygonStamped(
    const std::string& frame_id, const builtin_interfaces::msg::Time& stamp, const std::vector<Vertex>& vertices) {
  auto polygon = geometry_msgs::msg::PolygonStamped();
  polygon.header.frame_id = frame_id;
  polygon.header.stamp = stamp;
  polygon.polygon = createPolygon3D(vertices);
  return polygon;
}

/**
 * \brief Create a stamped 3D polygon instance.
 * \param[in] frame_id The frame of the polygon.
 * \param[in] stamp The timestamp of the polygon.
 * \param[in] vertices The vertices of the polygon.
 * \param[in] id The ID of the polygon instance.
 * \return The created polygon.
 */
geometry_msgs::msg::PolygonInstanceStamped createPolygonInstanceStamped(
    const std::string& frame_id, const builtin_interfaces::msg::Time& stamp, const std::vector<Vertex>& vertices,
    const uint32_t id = 42) {
  auto polygon = geometry_msgs::msg::PolygonInstanceStamped();
  polygon.header.frame_id = frame_id;
  polygon.header.stamp = stamp;
  polygon.polygon.id = id;
  polygon.polygon.polygon = createPolygon3D(vertices);
  return polygon;
}

/**
 * \brief Create a stamped 3D polygon with double-precision vertices.
 * \param[in] frame_id The frame of the polygon.
 * \param[in] stamp The timestamp of the polygon.
 * \param[in] vertices The vertices of the polygon.
 * \return The created polygon.
 */
cras_geo_msgs::msg::Polygon64Stamped createPolygon64Stamped(
    const std::string& frame_id, const builtin_interfaces::msg::Time& stamp, const std::vector<Vertex>& vertices) {
  auto polygon = cras_geo_msgs::msg::Polygon64Stamped();
  polygon.header.frame_id = frame_id;
  polygon.header.stamp = stamp;
  polygon.polygon = createPolygon64(vertices);
  return polygon;
}

/**
 * \brief Create a stamped 3D polygon instance with double-precision vertices.
 * \param[in] frame_id The frame of the polygon.
 * \param[in] stamp The timestamp of the polygon.
 * \param[in] vertices The vertices of the polygon.
 * \param[in] id The ID of the polygon instance.
 * \return The created polygon.
 */
cras_geo_msgs::msg::Polygon64InstanceStamped createPolygon64InstanceStamped(
    const std::string& frame_id, const builtin_interfaces::msg::Time& stamp, const std::vector<Vertex>& vertices,
    const int64_t id = 42) {
  auto polygon = cras_geo_msgs::msg::Polygon64InstanceStamped();
  polygon.header.frame_id = frame_id;
  polygon.header.stamp = stamp;
  polygon.polygon.id = id;
  polygon.polygon.polygon = createPolygon64(vertices);
  return polygon;
}

/**
 * \brief Create a per-polygon color with the given channels.
 * \param[in] r The red channel.
 * \param[in] g The green channel.
 * \param[in] b The blue channel.
 * \param[in] a The alpha channel.
 * \return The created color.
 */
std_msgs::msg::ColorRGBA createColor(const float r, const float g, const float b, const float a = 1.0) {
  auto color = std_msgs::msg::ColorRGBA();
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

// The per-polygon colors of the test collections. The two colors differ so that a possible mix-up of the
// per-polygon colors is detected.
const std_msgs::msg::ColorRGBA kFirstColor = createColor(1.0, 0.0, 0.0);  // NOLINT
const std_msgs::msg::ColorRGBA kSecondColor = createColor(0.0, 1.0, 0.2);  // NOLINT

/**
 * \brief Create a TF buffer with a static transform between the collection frame and the polygon frame.
 *
 * The transform rotates by 90 degrees around the Z-axis and translates by (1, 2, 3). It is published in the
 * direction from the collection frame to the polygon frame, which is also the direction the conversion looks it
 * up in, so the polygons are rotated and translated by exactly this transform.
 *
 * \return The buffer.
 */
std::unique_ptr<tf2_ros::Buffer> createTfBuffer() {
  auto buffer = std::make_unique<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));

  auto tf = geometry_msgs::msg::TransformStamped();
  tf.header.stamp = createTime(0);
  tf.header.frame_id = kCollectionFrame;
  tf.child_frame_id = kPolygonFrame;
  tf.transform.translation.x = 1.0;
  tf.transform.translation.y = 2.0;
  tf.transform.translation.z = 3.0;
  tf.transform.rotation.z = M_SQRT1_2;
  tf.transform.rotation.w = M_SQRT1_2;
  buffer->setTransform(tf, "test", true);

  return buffer;
}

}  // namespace

TEST(Convert2D3D, Convert2DTo3DPolygonStamped) {  // NOLINT
  const auto msg_2d = createPolygon2DStamped(kPolygonFrame, createTime(1, 2), kFlatVertices);

  const auto msg_3d = cras_geo_tools::convert2DTo3D<polygon_msgs::msg::Polygon2DStamped,
      geometry_msgs::msg::PolygonStamped>(msg_2d);

  EXPECT_EQ(msg_2d.header, msg_3d.header);
  ASSERT_EQ(3u, msg_3d.polygon.points.size());
  EXPECT_NEAR(msg_2d.polygon.points[0].x, msg_3d.polygon.points[0].x, kTol);
  EXPECT_NEAR(msg_2d.polygon.points[0].y, msg_3d.polygon.points[0].y, kTol);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygon.points[0].z);
  EXPECT_NEAR(msg_2d.polygon.points[1].x, msg_3d.polygon.points[1].x, kTol);
  EXPECT_NEAR(msg_2d.polygon.points[1].y, msg_3d.polygon.points[1].y, kTol);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygon.points[1].z);
  EXPECT_NEAR(msg_2d.polygon.points[2].x, msg_3d.polygon.points[2].x, kTol);
  EXPECT_NEAR(msg_2d.polygon.points[2].y, msg_3d.polygon.points[2].y, kTol);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygon.points[2].z);
}

TEST(Convert2D3D, Convert2DTo3DPolygon64Stamped) {  // NOLINT
  const auto msg_2d = createPolygon2DStamped(kPolygonFrame, createTime(1, 2), kFlatVertices);

  const auto msg_3d = cras_geo_tools::convert2DTo3D<polygon_msgs::msg::Polygon2DStamped,
      cras_geo_msgs::msg::Polygon64Stamped>(msg_2d);

  EXPECT_EQ(msg_2d.header, msg_3d.header);
  ASSERT_EQ(3u, msg_3d.polygon.points.size());
  // the target type stores the coordinates in double precision, so no precision is lost
  EXPECT_EQ(msg_2d.polygon.points[0].x, msg_3d.polygon.points[0].x);
  EXPECT_EQ(msg_2d.polygon.points[0].y, msg_3d.polygon.points[0].y);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygon.points[0].z);
  EXPECT_EQ(msg_2d.polygon.points[1].x, msg_3d.polygon.points[1].x);
  EXPECT_EQ(msg_2d.polygon.points[1].y, msg_3d.polygon.points[1].y);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygon.points[1].z);
  EXPECT_EQ(msg_2d.polygon.points[2].x, msg_3d.polygon.points[2].x);
  EXPECT_EQ(msg_2d.polygon.points[2].y, msg_3d.polygon.points[2].y);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygon.points[2].z);
}

TEST(Convert2D3D, Convert2DTo3DPolygonCollection) {  // NOLINT
  auto msg_2d = polygon_msgs::msg::Polygon2DCollection();
  msg_2d.header.frame_id = kCollectionFrame;
  msg_2d.header.stamp = createTime(3, 4);
  msg_2d.polygons = {createPolygon2D(kFlatVertices), createPolygon2D({{0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}})};
  msg_2d.colors = {kFirstColor, kSecondColor};

  const auto msg_3d = cras_geo_tools::convert2DTo3D<polygon_msgs::msg::Polygon2DCollection,
      cras_geo_msgs::msg::PolygonCollection>(msg_2d);

  EXPECT_EQ(msg_2d.header, msg_3d.header);
  ASSERT_EQ(2u, msg_3d.polygons.size());
  // the per-polygon colors are copied to the output
  ASSERT_EQ(2u, msg_3d.colors.size());
  EXPECT_EQ(kFirstColor, msg_3d.colors[0]);
  EXPECT_EQ(kSecondColor, msg_3d.colors[1]);

  ASSERT_EQ(3u, msg_3d.polygons[0].points.size());
  EXPECT_NEAR(msg_2d.polygons[0].points[0].x, msg_3d.polygons[0].points[0].x, kTol);
  EXPECT_NEAR(msg_2d.polygons[0].points[0].y, msg_3d.polygons[0].points[0].y, kTol);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[0].points[0].z);
  EXPECT_NEAR(msg_2d.polygons[0].points[1].x, msg_3d.polygons[0].points[1].x, kTol);
  EXPECT_NEAR(msg_2d.polygons[0].points[1].y, msg_3d.polygons[0].points[1].y, kTol);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[0].points[1].z);
  EXPECT_NEAR(msg_2d.polygons[0].points[2].x, msg_3d.polygons[0].points[2].x, kTol);
  EXPECT_NEAR(msg_2d.polygons[0].points[2].y, msg_3d.polygons[0].points[2].y, kTol);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[0].points[2].z);

  ASSERT_EQ(2u, msg_3d.polygons[1].points.size());
  EXPECT_NEAR(msg_2d.polygons[1].points[0].x, msg_3d.polygons[1].points[0].x, kTol);
  EXPECT_NEAR(msg_2d.polygons[1].points[0].y, msg_3d.polygons[1].points[0].y, kTol);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[1].points[0].z);
  EXPECT_NEAR(msg_2d.polygons[1].points[1].x, msg_3d.polygons[1].points[1].x, kTol);
  EXPECT_NEAR(msg_2d.polygons[1].points[1].y, msg_3d.polygons[1].points[1].y, kTol);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[1].points[1].z);
}

TEST(Convert2D3D, Convert2DTo3DPolygon64Collection) {  // NOLINT
  auto msg_2d = polygon_msgs::msg::Polygon2DCollection();
  msg_2d.header.frame_id = kCollectionFrame;
  msg_2d.header.stamp = createTime(3, 4);
  msg_2d.polygons = {createPolygon2D(kFlatVertices), createPolygon2D({{0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}})};
  msg_2d.colors = {kFirstColor, kSecondColor};

  const auto msg_3d = cras_geo_tools::convert2DTo3D<polygon_msgs::msg::Polygon2DCollection,
      cras_geo_msgs::msg::Polygon64Collection>(msg_2d);

  EXPECT_EQ(msg_2d.header, msg_3d.header);
  ASSERT_EQ(2u, msg_3d.polygons.size());
  // the per-polygon colors are copied to the output
  ASSERT_EQ(2u, msg_3d.colors.size());
  EXPECT_EQ(kFirstColor, msg_3d.colors[0]);
  EXPECT_EQ(kSecondColor, msg_3d.colors[1]);

  ASSERT_EQ(3u, msg_3d.polygons[0].points.size());
  // the target type stores the coordinates in double precision, so no precision is lost
  EXPECT_EQ(msg_2d.polygons[0].points[0].x, msg_3d.polygons[0].points[0].x);
  EXPECT_EQ(msg_2d.polygons[0].points[0].y, msg_3d.polygons[0].points[0].y);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[0].points[0].z);
  EXPECT_EQ(msg_2d.polygons[0].points[1].x, msg_3d.polygons[0].points[1].x);
  EXPECT_EQ(msg_2d.polygons[0].points[1].y, msg_3d.polygons[0].points[1].y);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[0].points[1].z);
  EXPECT_EQ(msg_2d.polygons[0].points[2].x, msg_3d.polygons[0].points[2].x);
  EXPECT_EQ(msg_2d.polygons[0].points[2].y, msg_3d.polygons[0].points[2].y);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[0].points[2].z);

  ASSERT_EQ(2u, msg_3d.polygons[1].points.size());
  EXPECT_EQ(msg_2d.polygons[1].points[0].x, msg_3d.polygons[1].points[0].x);
  EXPECT_EQ(msg_2d.polygons[1].points[0].y, msg_3d.polygons[1].points[0].y);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[1].points[0].z);
  EXPECT_EQ(msg_2d.polygons[1].points[1].x, msg_3d.polygons[1].points[1].x);
  EXPECT_EQ(msg_2d.polygons[1].points[1].y, msg_3d.polygons[1].points[1].y);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[1].points[1].z);
}

TEST(Convert2D3D, Convert2DTo3DPolygonStampedCollection) {  // NOLINT
  auto msg_2d = polygon_msgs::msg::Polygon2DCollection();
  msg_2d.header.frame_id = kCollectionFrame;
  msg_2d.header.stamp = createTime(3, 4);
  msg_2d.polygons = {createPolygon2D(kFlatVertices), createPolygon2D({{0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}})};
  msg_2d.colors = {kFirstColor, kSecondColor};

  const auto msg_3d = cras_geo_tools::convert2DTo3D<polygon_msgs::msg::Polygon2DCollection,
      cras_geo_msgs::msg::PolygonStampedCollection>(msg_2d);

  EXPECT_EQ(msg_2d.header, msg_3d.header);
  ASSERT_EQ(2u, msg_3d.polygons.size());
  // the per-polygon colors are copied to the output
  ASSERT_EQ(2u, msg_3d.colors.size());
  EXPECT_EQ(kFirstColor, msg_3d.colors[0]);
  EXPECT_EQ(kSecondColor, msg_3d.colors[1]);
  // each polygon gets the header of the collection
  EXPECT_EQ(msg_2d.header, msg_3d.polygons[0].header);
  EXPECT_EQ(msg_2d.header, msg_3d.polygons[1].header);

  ASSERT_EQ(3u, msg_3d.polygons[0].polygon.points.size());
  EXPECT_NEAR(msg_2d.polygons[0].points[0].x, msg_3d.polygons[0].polygon.points[0].x, kTol);
  EXPECT_NEAR(msg_2d.polygons[0].points[0].y, msg_3d.polygons[0].polygon.points[0].y, kTol);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[0].polygon.points[0].z);
  EXPECT_NEAR(msg_2d.polygons[0].points[1].x, msg_3d.polygons[0].polygon.points[1].x, kTol);
  EXPECT_NEAR(msg_2d.polygons[0].points[1].y, msg_3d.polygons[0].polygon.points[1].y, kTol);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[0].polygon.points[1].z);
  EXPECT_NEAR(msg_2d.polygons[0].points[2].x, msg_3d.polygons[0].polygon.points[2].x, kTol);
  EXPECT_NEAR(msg_2d.polygons[0].points[2].y, msg_3d.polygons[0].polygon.points[2].y, kTol);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[0].polygon.points[2].z);

  ASSERT_EQ(2u, msg_3d.polygons[1].polygon.points.size());
  EXPECT_NEAR(msg_2d.polygons[1].points[0].x, msg_3d.polygons[1].polygon.points[0].x, kTol);
  EXPECT_NEAR(msg_2d.polygons[1].points[0].y, msg_3d.polygons[1].polygon.points[0].y, kTol);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[1].polygon.points[0].z);
  EXPECT_NEAR(msg_2d.polygons[1].points[1].x, msg_3d.polygons[1].polygon.points[1].x, kTol);
  EXPECT_NEAR(msg_2d.polygons[1].points[1].y, msg_3d.polygons[1].polygon.points[1].y, kTol);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[1].polygon.points[1].z);
}

TEST(Convert2D3D, Convert2DTo3DPolygon64StampedCollection) {  // NOLINT
  auto msg_2d = polygon_msgs::msg::Polygon2DCollection();
  msg_2d.header.frame_id = kCollectionFrame;
  msg_2d.header.stamp = createTime(3, 4);
  msg_2d.polygons = {createPolygon2D(kFlatVertices), createPolygon2D({{0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}})};
  msg_2d.colors = {kFirstColor, kSecondColor};

  const auto msg_3d = cras_geo_tools::convert2DTo3D<polygon_msgs::msg::Polygon2DCollection,
      cras_geo_msgs::msg::Polygon64StampedCollection>(msg_2d);

  EXPECT_EQ(msg_2d.header, msg_3d.header);
  ASSERT_EQ(2u, msg_3d.polygons.size());
  // the per-polygon colors are copied to the output
  ASSERT_EQ(2u, msg_3d.colors.size());
  EXPECT_EQ(kFirstColor, msg_3d.colors[0]);
  EXPECT_EQ(kSecondColor, msg_3d.colors[1]);
  // each polygon gets the header of the collection
  EXPECT_EQ(msg_2d.header, msg_3d.polygons[0].header);
  EXPECT_EQ(msg_2d.header, msg_3d.polygons[1].header);

  ASSERT_EQ(3u, msg_3d.polygons[0].polygon.points.size());
  // the target type stores the coordinates in double precision, so no precision is lost
  EXPECT_EQ(msg_2d.polygons[0].points[0].x, msg_3d.polygons[0].polygon.points[0].x);
  EXPECT_EQ(msg_2d.polygons[0].points[0].y, msg_3d.polygons[0].polygon.points[0].y);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[0].polygon.points[0].z);
  EXPECT_EQ(msg_2d.polygons[0].points[1].x, msg_3d.polygons[0].polygon.points[1].x);
  EXPECT_EQ(msg_2d.polygons[0].points[1].y, msg_3d.polygons[0].polygon.points[1].y);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[0].polygon.points[1].z);
  EXPECT_EQ(msg_2d.polygons[0].points[2].x, msg_3d.polygons[0].polygon.points[2].x);
  EXPECT_EQ(msg_2d.polygons[0].points[2].y, msg_3d.polygons[0].polygon.points[2].y);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[0].polygon.points[2].z);

  ASSERT_EQ(2u, msg_3d.polygons[1].polygon.points.size());
  EXPECT_EQ(msg_2d.polygons[1].points[0].x, msg_3d.polygons[1].polygon.points[0].x);
  EXPECT_EQ(msg_2d.polygons[1].points[0].y, msg_3d.polygons[1].polygon.points[0].y);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[1].polygon.points[0].z);
  EXPECT_EQ(msg_2d.polygons[1].points[1].x, msg_3d.polygons[1].polygon.points[1].x);
  EXPECT_EQ(msg_2d.polygons[1].points[1].y, msg_3d.polygons[1].polygon.points[1].y);
  EXPECT_DOUBLE_EQ(0.0, msg_3d.polygons[1].polygon.points[1].z);
}

TEST(Convert2D3D, Convert3DTo2DPolygonStamped) {  // NOLINT
  const auto msg_3d = createPolygonStamped(kPolygonFrame, createTime(1, 2), kFlatVertices);

  // by default, the flatness of the polygon is not checked
  const auto defaultOptions = cras_geo_tools::convert3DTo2D<geometry_msgs::msg::PolygonStamped,
      polygon_msgs::msg::Polygon2DStamped>(msg_3d, cras_geo_tools::ConversionOptions());
  ASSERT_TRUE(defaultOptions.has_value());
  EXPECT_EQ(msg_3d.header, defaultOptions->header);
  ASSERT_EQ(3u, defaultOptions->polygon.points.size());
  EXPECT_NEAR(msg_3d.polygon.points[0].x, defaultOptions->polygon.points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[0].y, defaultOptions->polygon.points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[1].x, defaultOptions->polygon.points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[1].y, defaultOptions->polygon.points[1].y, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[2].x, defaultOptions->polygon.points[2].x, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[2].y, defaultOptions->polygon.points[2].y, kTol);

  // a flat polygon passes even if the flatness is required
  auto options = cras_geo_tools::ConversionOptions();
  options.z_diff_threshold = 0.0;
  const auto flat = cras_geo_tools::convert3DTo2D<geometry_msgs::msg::PolygonStamped,
      polygon_msgs::msg::Polygon2DStamped>(msg_3d, options);
  ASSERT_TRUE(flat.has_value());
  EXPECT_EQ(msg_3d.header, flat->header);
  ASSERT_EQ(3u, flat->polygon.points.size());
  EXPECT_NEAR(msg_3d.polygon.points[0].x, flat->polygon.points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[0].y, flat->polygon.points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[1].x, flat->polygon.points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[1].y, flat->polygon.points[1].y, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[2].x, flat->polygon.points[2].x, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[2].y, flat->polygon.points[2].y, kTol);

  // a non-flat polygon passes with a large enough threshold (the Z-coordinates are simply dropped)
  const auto msgNonFlat = createPolygonStamped(kPolygonFrame, createTime(1, 2), kNonFlatVertices);
  options.z_diff_threshold = 0.5;
  const auto converted = cras_geo_tools::convert3DTo2D<geometry_msgs::msg::PolygonStamped,
      polygon_msgs::msg::Polygon2DStamped>(msgNonFlat, options);
  ASSERT_TRUE(converted.has_value());
  ASSERT_EQ(3u, converted->polygon.points.size());
  EXPECT_NEAR(msgNonFlat.polygon.points[0].x, converted->polygon.points[0].x, kTol);
  EXPECT_NEAR(msgNonFlat.polygon.points[0].y, converted->polygon.points[0].y, kTol);
  EXPECT_NEAR(msgNonFlat.polygon.points[1].x, converted->polygon.points[1].x, kTol);
  EXPECT_NEAR(msgNonFlat.polygon.points[1].y, converted->polygon.points[1].y, kTol);
  EXPECT_NEAR(msgNonFlat.polygon.points[2].x, converted->polygon.points[2].x, kTol);
  EXPECT_NEAR(msgNonFlat.polygon.points[2].y, converted->polygon.points[2].y, kTol);
}

TEST(Convert2D3D, Convert3DTo2DPolygonStampedNotFlat) {  // NOLINT
  const auto msg_3d = createPolygonStamped(kPolygonFrame, createTime(1, 2), kNonFlatVertices);

  auto options = cras_geo_tools::ConversionOptions();
  options.z_diff_threshold = 0.01;
  const auto msg_2d = cras_geo_tools::convert3DTo2D<geometry_msgs::msg::PolygonStamped,
      polygon_msgs::msg::Polygon2DStamped>(msg_3d, options);

  ASSERT_FALSE(msg_2d.has_value());
  EXPECT_TRUE(msg_2d.error().z_diff_too_large);
  EXPECT_FALSE(msg_2d.error().frame_mismatch);
  EXPECT_FALSE(msg_2d.error().stamp_mismatch);
  EXPECT_FALSE(msg_2d.error().tf_error);
  EXPECT_EQ("Polygon is not flat", msg_2d.error().error_message);

  // skip_errors is not supported by the single-polygon specializations, the error is returned in any case
  options.skip_errors = true;
  const auto skipped = cras_geo_tools::convert3DTo2D<geometry_msgs::msg::PolygonStamped,
      polygon_msgs::msg::Polygon2DStamped>(msg_3d, options);
  EXPECT_FALSE(skipped.has_value());
}

TEST(Convert2D3D, Convert3DTo2DPolygonInstanceStamped) {  // NOLINT
  const auto msg_3d = createPolygonInstanceStamped(kPolygonFrame, createTime(1, 2), kFlatVertices);

  auto options = cras_geo_tools::ConversionOptions();
  options.z_diff_threshold = 0.0;
  const auto flat = cras_geo_tools::convert3DTo2D<geometry_msgs::msg::PolygonInstanceStamped,
      polygon_msgs::msg::Polygon2DStamped>(msg_3d, options);
  ASSERT_TRUE(flat.has_value());
  // the instance ID is dropped by the conversion (the target type has no such field)
  EXPECT_EQ(msg_3d.header, flat->header);
  ASSERT_EQ(3u, flat->polygon.points.size());
  EXPECT_NEAR(msg_3d.polygon.polygon.points[0].x, flat->polygon.points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygon.polygon.points[0].y, flat->polygon.points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygon.polygon.points[1].x, flat->polygon.points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygon.polygon.points[1].y, flat->polygon.points[1].y, kTol);
  EXPECT_NEAR(msg_3d.polygon.polygon.points[2].x, flat->polygon.points[2].x, kTol);
  EXPECT_NEAR(msg_3d.polygon.polygon.points[2].y, flat->polygon.points[2].y, kTol);

  const auto msgNonFlat = createPolygonInstanceStamped(kPolygonFrame, createTime(1, 2), kNonFlatVertices);
  options.z_diff_threshold = 0.01;
  const auto nonFlat = cras_geo_tools::convert3DTo2D<geometry_msgs::msg::PolygonInstanceStamped,
      polygon_msgs::msg::Polygon2DStamped>(msgNonFlat, options);
  ASSERT_FALSE(nonFlat.has_value());
  EXPECT_TRUE(nonFlat.error().z_diff_too_large);
  EXPECT_EQ("Polygon is not flat", nonFlat.error().error_message);
}

TEST(Convert2D3D, Convert3DTo2DPolygon64Stamped) {  // NOLINT
  const auto msg_3d = createPolygon64Stamped(kPolygonFrame, createTime(1, 2), kFlatVertices);

  // by default, the flatness of the polygon is not checked
  const auto defaultOptions = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64Stamped,
      polygon_msgs::msg::Polygon2DStamped>(msg_3d, cras_geo_tools::ConversionOptions());
  ASSERT_TRUE(defaultOptions.has_value());
  EXPECT_EQ(msg_3d.header, defaultOptions->header);
  ASSERT_EQ(3u, defaultOptions->polygon.points.size());
  EXPECT_NEAR(msg_3d.polygon.points[0].x, defaultOptions->polygon.points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[0].y, defaultOptions->polygon.points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[1].x, defaultOptions->polygon.points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[1].y, defaultOptions->polygon.points[1].y, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[2].x, defaultOptions->polygon.points[2].x, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[2].y, defaultOptions->polygon.points[2].y, kTol);

  // a flat polygon passes even if the flatness is required
  auto options = cras_geo_tools::ConversionOptions();
  options.z_diff_threshold = 0.0;
  const auto flat = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64Stamped,
      polygon_msgs::msg::Polygon2DStamped>(msg_3d, options);
  ASSERT_TRUE(flat.has_value());
  EXPECT_EQ(msg_3d.header, flat->header);
  ASSERT_EQ(3u, flat->polygon.points.size());
  EXPECT_NEAR(msg_3d.polygon.points[0].x, flat->polygon.points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[0].y, flat->polygon.points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[1].x, flat->polygon.points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[1].y, flat->polygon.points[1].y, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[2].x, flat->polygon.points[2].x, kTol);
  EXPECT_NEAR(msg_3d.polygon.points[2].y, flat->polygon.points[2].y, kTol);

  // a non-flat polygon passes with a large enough threshold (the Z-coordinates are simply dropped)
  const auto msgNonFlat = createPolygon64Stamped(kPolygonFrame, createTime(1, 2), kNonFlatVertices);
  options.z_diff_threshold = 0.5;
  const auto converted = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64Stamped,
      polygon_msgs::msg::Polygon2DStamped>(msgNonFlat, options);
  ASSERT_TRUE(converted.has_value());
  ASSERT_EQ(3u, converted->polygon.points.size());
  EXPECT_NEAR(msgNonFlat.polygon.points[0].x, converted->polygon.points[0].x, kTol);
  EXPECT_NEAR(msgNonFlat.polygon.points[0].y, converted->polygon.points[0].y, kTol);
  EXPECT_NEAR(msgNonFlat.polygon.points[1].x, converted->polygon.points[1].x, kTol);
  EXPECT_NEAR(msgNonFlat.polygon.points[1].y, converted->polygon.points[1].y, kTol);
  EXPECT_NEAR(msgNonFlat.polygon.points[2].x, converted->polygon.points[2].x, kTol);
  EXPECT_NEAR(msgNonFlat.polygon.points[2].y, converted->polygon.points[2].y, kTol);
}

TEST(Convert2D3D, Convert3DTo2DPolygon64StampedNotFlat) {  // NOLINT
  const auto msg_3d = createPolygon64Stamped(kPolygonFrame, createTime(1, 2), kNonFlatVertices);

  auto options = cras_geo_tools::ConversionOptions();
  options.z_diff_threshold = 0.01;
  const auto msg_2d = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64Stamped,
      polygon_msgs::msg::Polygon2DStamped>(msg_3d, options);

  ASSERT_FALSE(msg_2d.has_value());
  EXPECT_TRUE(msg_2d.error().z_diff_too_large);
  EXPECT_FALSE(msg_2d.error().frame_mismatch);
  EXPECT_FALSE(msg_2d.error().stamp_mismatch);
  EXPECT_FALSE(msg_2d.error().tf_error);
  EXPECT_EQ("Polygon is not flat", msg_2d.error().error_message);

  // skip_errors is not supported by the single-polygon specializations, the error is returned in any case
  options.skip_errors = true;
  const auto skipped = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64Stamped,
      polygon_msgs::msg::Polygon2DStamped>(msg_3d, options);
  EXPECT_FALSE(skipped.has_value());
}

TEST(Convert2D3D, Convert3DTo2DPolygon64InstanceStamped) {  // NOLINT
  const auto msg_3d = createPolygon64InstanceStamped(kPolygonFrame, createTime(1, 2), kFlatVertices);

  auto options = cras_geo_tools::ConversionOptions();
  options.z_diff_threshold = 0.0;
  const auto flat = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64InstanceStamped,
      polygon_msgs::msg::Polygon2DStamped>(msg_3d, options);
  ASSERT_TRUE(flat.has_value());
  // the instance ID is dropped by the conversion (the target type has no such field)
  EXPECT_EQ(msg_3d.header, flat->header);
  ASSERT_EQ(3u, flat->polygon.points.size());
  EXPECT_NEAR(msg_3d.polygon.polygon.points[0].x, flat->polygon.points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygon.polygon.points[0].y, flat->polygon.points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygon.polygon.points[1].x, flat->polygon.points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygon.polygon.points[1].y, flat->polygon.points[1].y, kTol);
  EXPECT_NEAR(msg_3d.polygon.polygon.points[2].x, flat->polygon.points[2].x, kTol);
  EXPECT_NEAR(msg_3d.polygon.polygon.points[2].y, flat->polygon.points[2].y, kTol);

  const auto msgNonFlat = createPolygon64InstanceStamped(kPolygonFrame, createTime(1, 2), kNonFlatVertices);
  options.z_diff_threshold = 0.01;
  const auto nonFlat = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64InstanceStamped,
      polygon_msgs::msg::Polygon2DStamped>(msgNonFlat, options);
  ASSERT_FALSE(nonFlat.has_value());
  EXPECT_TRUE(nonFlat.error().z_diff_too_large);
  EXPECT_EQ("Polygon is not flat", nonFlat.error().error_message);
}

TEST(Convert2D3D, ThrowingConvert3DTo2D) {  // NOLINT
  const auto msg_3d = createPolygonStamped(kPolygonFrame, createTime(1, 2), kFlatVertices);

  // the conversion cannot fail with the default options, so the throwing variant must return the same result
  const auto msg_2d = cras_geo_tools::convert3DTo2D<geometry_msgs::msg::PolygonStamped,
      polygon_msgs::msg::Polygon2DStamped>(msg_3d);
  const auto maybeMsg2d = cras_geo_tools::convert3DTo2D<geometry_msgs::msg::PolygonStamped,
      polygon_msgs::msg::Polygon2DStamped>(msg_3d, cras_geo_tools::ConversionOptions());
  ASSERT_TRUE(maybeMsg2d.has_value());
  EXPECT_EQ(maybeMsg2d->header, msg_2d.header);
  EXPECT_EQ(maybeMsg2d->polygon, msg_2d.polygon);

  const auto emptyMsg2d = cras_geo_tools::convert3DTo2D<geometry_msgs::msg::PolygonStamped,
      polygon_msgs::msg::Polygon2DStamped>(geometry_msgs::msg::PolygonStamped());
  EXPECT_TRUE(emptyMsg2d.polygon.points.empty());
}

TEST(Convert2D3D, Convert3DTo2DPolygonCollection) {  // NOLINT
  auto msg_3d = cras_geo_msgs::msg::PolygonCollection();
  msg_3d.header.frame_id = kCollectionFrame;
  msg_3d.header.stamp = createTime(5, 6);
  msg_3d.polygons = {createPolygon3D(kFlatVertices), createPolygon3D(kOtherFlatVertices)};
  msg_3d.colors = {kFirstColor, kSecondColor};

  auto options = cras_geo_tools::ConversionOptions();
  options.z_diff_threshold = 0.0;
  options.collection_z_diff_threshold = 1.5;
  const auto msg_2d = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(msg_2d.has_value());
  EXPECT_EQ(msg_3d.header, msg_2d->header);
  ASSERT_EQ(2u, msg_2d->polygons.size());
  // the per-polygon colors are copied to the output
  ASSERT_EQ(2u, msg_2d->colors.size());
  EXPECT_EQ(kFirstColor, msg_2d->colors[0]);
  EXPECT_EQ(kSecondColor, msg_2d->colors[1]);

  ASSERT_EQ(3u, msg_2d->polygons[0].points.size());
  EXPECT_NEAR(msg_3d.polygons[0].points[0].x, msg_2d->polygons[0].points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].points[0].y, msg_2d->polygons[0].points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].points[1].x, msg_2d->polygons[0].points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].points[1].y, msg_2d->polygons[0].points[1].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].points[2].x, msg_2d->polygons[0].points[2].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].points[2].y, msg_2d->polygons[0].points[2].y, kTol);

  ASSERT_EQ(2u, msg_2d->polygons[1].points.size());
  EXPECT_NEAR(msg_3d.polygons[1].points[0].x, msg_2d->polygons[1].points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].points[0].y, msg_2d->polygons[1].points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].points[1].x, msg_2d->polygons[1].points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].points[1].y, msg_2d->polygons[1].points[1].y, kTol);

  // the polygons lie in different planes, which is reported by the collection threshold
  options.collection_z_diff_threshold = 0.5;
  const auto differentPlanes = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(differentPlanes.has_value());
  EXPECT_TRUE(differentPlanes.error().z_diff_too_large);
  EXPECT_EQ("Polygons in collection do not have the same z.", differentPlanes.error().error_message);
  options.collection_z_diff_threshold = 1.5;

  // a polygon which is not flat makes the whole conversion fail
  msg_3d.polygons[1] = createPolygon3D(kNonFlatVertices);
  const auto notFlat = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(notFlat.has_value());
  EXPECT_TRUE(notFlat.error().z_diff_too_large);
  EXPECT_EQ("Polygon is not flat", notFlat.error().error_message);

  // skip_errors suppresses the errors of the individual polygons, but the polygons are still converted
  options.skip_errors = true;
  const auto skipped = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(skipped.has_value());
  ASSERT_EQ(2u, skipped->polygons.size());
  // all polygons are kept, so all their colors are kept as well
  ASSERT_EQ(2u, skipped->colors.size());
  EXPECT_EQ(kFirstColor, skipped->colors[0]);
  EXPECT_EQ(kSecondColor, skipped->colors[1]);
  ASSERT_EQ(3u, skipped->polygons[1].points.size());
  EXPECT_NEAR(msg_3d.polygons[1].points[0].x, skipped->polygons[1].points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].points[0].y, skipped->polygons[1].points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].points[1].x, skipped->polygons[1].points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].points[1].y, skipped->polygons[1].points[1].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].points[2].x, skipped->polygons[1].points[2].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].points[2].y, skipped->polygons[1].points[2].y, kTol);
}

TEST(Convert2D3D, Convert3DTo2DPolygon64Collection) {  // NOLINT
  auto msg_3d = cras_geo_msgs::msg::Polygon64Collection();
  msg_3d.header.frame_id = kCollectionFrame;
  msg_3d.header.stamp = createTime(5, 6);
  msg_3d.polygons = {createPolygon64(kFlatVertices), createPolygon64(kOtherFlatVertices)};
  msg_3d.colors = {kFirstColor, kSecondColor};

  auto options = cras_geo_tools::ConversionOptions();
  options.z_diff_threshold = 0.0;
  options.collection_z_diff_threshold = 1.5;
  const auto msg_2d = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64Collection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(msg_2d.has_value());
  EXPECT_EQ(msg_3d.header, msg_2d->header);
  ASSERT_EQ(2u, msg_2d->polygons.size());
  // the per-polygon colors are copied to the output
  ASSERT_EQ(2u, msg_2d->colors.size());
  EXPECT_EQ(kFirstColor, msg_2d->colors[0]);
  EXPECT_EQ(kSecondColor, msg_2d->colors[1]);

  ASSERT_EQ(3u, msg_2d->polygons[0].points.size());
  EXPECT_NEAR(msg_3d.polygons[0].points[0].x, msg_2d->polygons[0].points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].points[0].y, msg_2d->polygons[0].points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].points[1].x, msg_2d->polygons[0].points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].points[1].y, msg_2d->polygons[0].points[1].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].points[2].x, msg_2d->polygons[0].points[2].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].points[2].y, msg_2d->polygons[0].points[2].y, kTol);

  ASSERT_EQ(2u, msg_2d->polygons[1].points.size());
  EXPECT_NEAR(msg_3d.polygons[1].points[0].x, msg_2d->polygons[1].points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].points[0].y, msg_2d->polygons[1].points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].points[1].x, msg_2d->polygons[1].points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].points[1].y, msg_2d->polygons[1].points[1].y, kTol);

  // the polygons lie in different planes, which is reported by the collection threshold
  options.collection_z_diff_threshold = 0.5;
  const auto differentPlanes = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64Collection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(differentPlanes.has_value());
  EXPECT_TRUE(differentPlanes.error().z_diff_too_large);
  EXPECT_EQ("Polygons in collection do not have the same z.", differentPlanes.error().error_message);
  options.collection_z_diff_threshold = 1.5;

  // a polygon which is not flat makes the whole conversion fail
  msg_3d.polygons[1] = createPolygon64(kNonFlatVertices);
  const auto notFlat = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64Collection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(notFlat.has_value());
  EXPECT_TRUE(notFlat.error().z_diff_too_large);
  EXPECT_EQ("Polygon is not flat", notFlat.error().error_message);

  // skip_errors suppresses the errors of the individual polygons, but the polygons are still converted
  options.skip_errors = true;
  const auto skipped = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64Collection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(skipped.has_value());
  ASSERT_EQ(2u, skipped->polygons.size());
  // all polygons are kept, so all their colors are kept as well
  ASSERT_EQ(2u, skipped->colors.size());
  EXPECT_EQ(kFirstColor, skipped->colors[0]);
  EXPECT_EQ(kSecondColor, skipped->colors[1]);
  ASSERT_EQ(3u, skipped->polygons[1].points.size());
  EXPECT_NEAR(msg_3d.polygons[1].points[0].x, skipped->polygons[1].points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].points[0].y, skipped->polygons[1].points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].points[1].x, skipped->polygons[1].points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].points[1].y, skipped->polygons[1].points[1].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].points[2].x, skipped->polygons[1].points[2].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].points[2].y, skipped->polygons[1].points[2].y, kTol);
}

TEST(Convert2D3D, Convert3DTo2DPolygonInstanceCollection) {  // NOLINT
  auto msg_3d = cras_geo_msgs::msg::PolygonInstanceCollection();
  msg_3d.header.frame_id = kCollectionFrame;
  msg_3d.header.stamp = createTime(5, 6);
  msg_3d.polygons.emplace_back().polygon = createPolygon3D(kFlatVertices);
  msg_3d.polygons.emplace_back().polygon = createPolygon3D(kNonFlatVertices);
  msg_3d.colors = {kFirstColor, kSecondColor};

  auto options = cras_geo_tools::ConversionOptions();
  options.z_diff_threshold = 0.0;
  const auto notFlat = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonInstanceCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(notFlat.has_value());
  EXPECT_EQ("Polygon is not flat", notFlat.error().error_message);

  // the polygons lie in different planes, which is reported by the collection threshold
  options.z_diff_threshold = 0.5;
  options.collection_z_diff_threshold = 0.5;
  const auto differentPlanes = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonInstanceCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(differentPlanes.has_value());
  EXPECT_EQ("Polygons in collection do not have the same z.", differentPlanes.error().error_message);

  options.collection_z_diff_threshold = 1.5;
  const auto msg_2d = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonInstanceCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(msg_2d.has_value());
  EXPECT_EQ(msg_3d.header, msg_2d->header);
  ASSERT_EQ(2u, msg_2d->polygons.size());
  // the per-polygon colors are copied to the output
  ASSERT_EQ(2u, msg_2d->colors.size());
  EXPECT_EQ(kFirstColor, msg_2d->colors[0]);
  EXPECT_EQ(kSecondColor, msg_2d->colors[1]);
  ASSERT_EQ(3u, msg_2d->polygons[1].points.size());
  EXPECT_NEAR(msg_3d.polygons[1].polygon.points[0].x, msg_2d->polygons[1].points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].polygon.points[0].y, msg_2d->polygons[1].points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].polygon.points[1].x, msg_2d->polygons[1].points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].polygon.points[1].y, msg_2d->polygons[1].points[1].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].polygon.points[2].x, msg_2d->polygons[1].points[2].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].polygon.points[2].y, msg_2d->polygons[1].points[2].y, kTol);
}

TEST(Convert2D3D, Convert3DTo2DPolygon64InstanceCollection) {  // NOLINT
  auto msg_3d = cras_geo_msgs::msg::Polygon64InstanceCollection();
  msg_3d.header.frame_id = kCollectionFrame;
  msg_3d.header.stamp = createTime(5, 6);
  msg_3d.polygons.emplace_back().polygon = createPolygon64(kFlatVertices);
  msg_3d.polygons.emplace_back().polygon = createPolygon64(kNonFlatVertices);
  msg_3d.colors = {kFirstColor, kSecondColor};

  auto options = cras_geo_tools::ConversionOptions();
  options.z_diff_threshold = 0.0;
  const auto notFlat = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64InstanceCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(notFlat.has_value());
  EXPECT_EQ("Polygon is not flat", notFlat.error().error_message);

  // the polygons lie in different planes, which is reported by the collection threshold
  options.z_diff_threshold = 0.5;
  options.collection_z_diff_threshold = 0.5;
  const auto differentPlanes = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64InstanceCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(differentPlanes.has_value());
  EXPECT_EQ("Polygons in collection do not have the same z.", differentPlanes.error().error_message);

  options.collection_z_diff_threshold = 1.5;
  const auto msg_2d = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64InstanceCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(msg_2d.has_value());
  EXPECT_EQ(msg_3d.header, msg_2d->header);
  ASSERT_EQ(2u, msg_2d->polygons.size());
  // the per-polygon colors are copied to the output
  ASSERT_EQ(2u, msg_2d->colors.size());
  EXPECT_EQ(kFirstColor, msg_2d->colors[0]);
  EXPECT_EQ(kSecondColor, msg_2d->colors[1]);
  ASSERT_EQ(3u, msg_2d->polygons[1].points.size());
  EXPECT_NEAR(msg_3d.polygons[1].polygon.points[0].x, msg_2d->polygons[1].points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].polygon.points[0].y, msg_2d->polygons[1].points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].polygon.points[1].x, msg_2d->polygons[1].points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].polygon.points[1].y, msg_2d->polygons[1].points[1].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].polygon.points[2].x, msg_2d->polygons[1].points[2].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[1].polygon.points[2].y, msg_2d->polygons[1].points[2].y, kTol);
}

TEST(Convert2D3D, Convert3DTo2DPolygonStampedCollectionFrames) {  // NOLINT
  auto msg_3d = cras_geo_msgs::msg::PolygonStampedCollection();
  msg_3d.header.frame_id = kCollectionFrame;
  msg_3d.header.stamp = createTime(5);
  msg_3d.polygons.emplace_back() = createPolygonStamped(kCollectionFrame, createTime(5), kFlatVertices);
  msg_3d.polygons.emplace_back() = createPolygonStamped(kPolygonFrame, createTime(5), kFlatVertices);
  msg_3d.colors = {kFirstColor, kSecondColor};

  auto options = cras_geo_tools::ConversionOptions();
  options.z_diff_threshold = 0.0;

  // the frames are not compared unless it is requested
  const auto converted = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonStampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(converted.has_value());
  ASSERT_EQ(2u, converted->polygons.size());
  // the per-polygon colors are copied to the output
  ASSERT_EQ(2u, converted->colors.size());
  EXPECT_EQ(kFirstColor, converted->colors[0]);
  EXPECT_EQ(kSecondColor, converted->colors[1]);

  // with the frame check enabled, the mismatching polygon is reported
  options.require_same_frame = true;
  const auto mismatch = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonStampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(mismatch.has_value());
  EXPECT_TRUE(mismatch.error().frame_mismatch);
  EXPECT_FALSE(mismatch.error().z_diff_too_large);
  EXPECT_EQ("Polygon frame does not match collection frame", mismatch.error().error_message);

  // with skip_errors, the mismatching polygon is skipped instead of failing the whole conversion
  options.skip_errors = true;
  const auto skipped = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonStampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(skipped.has_value());
  EXPECT_EQ(msg_3d.header, skipped->header);
  ASSERT_EQ(1u, skipped->polygons.size());
  // the color of the skipped polygon must be skipped as well so that the colors match the polygons
  ASSERT_EQ(1u, skipped->colors.size());
  EXPECT_EQ(kFirstColor, skipped->colors[0]);
  ASSERT_EQ(3u, skipped->polygons[0].points.size());
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[0].x, skipped->polygons[0].points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[0].y, skipped->polygons[0].points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[1].x, skipped->polygons[0].points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[1].y, skipped->polygons[0].points[1].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[2].x, skipped->polygons[0].points[2].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[2].y, skipped->polygons[0].points[2].y, kTol);
}

TEST(Convert2D3D, Convert3DTo2DPolygonStampedCollectionStamps) {  // NOLINT
  auto msg_3d = cras_geo_msgs::msg::PolygonStampedCollection();
  msg_3d.header.frame_id = kCollectionFrame;
  msg_3d.header.stamp = createTime(5);
  msg_3d.polygons.emplace_back() = createPolygonStamped(kCollectionFrame, createTime(5), kFlatVertices);
  msg_3d.polygons.emplace_back() = createPolygonStamped(kCollectionFrame, createTime(6), kFlatVertices);
  msg_3d.colors = {kFirstColor, kSecondColor};

  auto options = cras_geo_tools::ConversionOptions();
  options.z_diff_threshold = 0.0;

  // the timestamps are not compared unless it is requested
  const auto converted = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonStampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(converted.has_value());
  ASSERT_EQ(2u, converted->polygons.size());

  // with the timestamp check enabled, the mismatching polygon is reported
  options.require_same_stamp = true;
  const auto mismatch = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonStampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(mismatch.has_value());
  EXPECT_FALSE(mismatch.error().z_diff_too_large);
  EXPECT_EQ("Polygon stamp does not match collection stamp", mismatch.error().error_message);

  // with skip_errors, the mismatching polygon is skipped instead of failing the whole conversion
  options.skip_errors = true;
  const auto skipped = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonStampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(skipped.has_value());
  ASSERT_EQ(1u, skipped->polygons.size());
  // the color of the skipped polygon must be skipped as well so that the colors match the polygons
  ASSERT_EQ(1u, skipped->colors.size());
  EXPECT_EQ(kFirstColor, skipped->colors[0]);
  ASSERT_EQ(3u, skipped->polygons[0].points.size());
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[0].x, skipped->polygons[0].points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[0].y, skipped->polygons[0].points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[1].x, skipped->polygons[0].points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[1].y, skipped->polygons[0].points[1].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[2].x, skipped->polygons[0].points[2].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[2].y, skipped->polygons[0].points[2].y, kTol);
}

TEST(Convert2D3D, Convert3DTo2DPolygonStampedCollectionTf) {  // NOLINT
  auto msg_3d = cras_geo_msgs::msg::PolygonStampedCollection();
  msg_3d.header.frame_id = kCollectionFrame;
  msg_3d.header.stamp = createTime(5);
  msg_3d.polygons.emplace_back() = createPolygonStamped(kPolygonFrame, createTime(6), kTransformedVertices);
  msg_3d.colors = {kFirstColor};

  auto options = cras_geo_tools::ConversionOptions();
  options.transform_to_header_stamp = true;
  options.z_diff_threshold = 0.0;

  // no TF buffer is available, so the polygon cannot be transformed
  const auto noTf = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonStampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(noTf.has_value());
  EXPECT_TRUE(noTf.error().tf_error);
  EXPECT_EQ("No transform buffer provided", noTf.error().error_message);

  // with skip_errors, the untransformable polygon is skipped
  options.skip_errors = true;
  const auto skipped = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonStampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(skipped.has_value());
  EXPECT_TRUE(skipped->polygons.empty());
  // the color of the skipped polygon is skipped as well
  EXPECT_TRUE(skipped->colors.empty());
  options.skip_errors = false;

  // the transform is not known in the TF buffer
  const auto emptyBuffer = std::make_unique<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  options.tf = emptyBuffer.get();
  const auto unknownTf = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonStampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(unknownTf.has_value());
  EXPECT_TRUE(unknownTf.error().tf_error);
  EXPECT_FALSE(unknownTf.error().error_message.empty());

  // with a known transform, the polygon is transformed to the collection frame
  const auto buffer = createTfBuffer();
  options.tf = buffer.get();
  const auto converted = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonStampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(converted.has_value());
  EXPECT_EQ(msg_3d.header, converted->header);
  ASSERT_EQ(1u, converted->polygons.size());
  // the color of the transformed polygon is preserved
  ASSERT_EQ(1u, converted->colors.size());
  EXPECT_EQ(kFirstColor, converted->colors[0]);
  ASSERT_EQ(3u, converted->polygons[0].points.size());
  EXPECT_NEAR(1.0 - kTransformedVertices[0].y, converted->polygons[0].points[0].x, kTol);
  EXPECT_NEAR(2.0 + kTransformedVertices[0].x, converted->polygons[0].points[0].y, kTol);
  EXPECT_NEAR(1.0 - kTransformedVertices[1].y, converted->polygons[0].points[1].x, kTol);
  EXPECT_NEAR(2.0 + kTransformedVertices[1].x, converted->polygons[0].points[1].y, kTol);
  EXPECT_NEAR(1.0 - kTransformedVertices[2].y, converted->polygons[0].points[2].x, kTol);
  EXPECT_NEAR(2.0 + kTransformedVertices[2].x, converted->polygons[0].points[2].y, kTol);
}

TEST(Convert2D3D, Convert3DTo2DPolygonStampedCollectionTfToHeaderFrame) {  // NOLINT
  auto msg_3d = cras_geo_msgs::msg::PolygonStampedCollection();
  msg_3d.header.frame_id = kCollectionFrame;
  msg_3d.header.stamp = createTime(5);
  msg_3d.polygons.emplace_back() = createPolygonStamped(kPolygonFrame, createTime(5), kFlatVertices);

  // the transform is looked up from the polygon frame to the polygon frame, which yields an identity transform
  auto options = cras_geo_tools::ConversionOptions();
  options.transform_to_header_frame = true;
  options.z_diff_threshold = 0.0;
  const auto buffer = createTfBuffer();
  options.tf = buffer.get();

  const auto converted = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonStampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(converted.has_value());
  ASSERT_EQ(1u, converted->polygons.size());
  ASSERT_EQ(3u, converted->polygons[0].points.size());
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[0].x, converted->polygons[0].points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[0].y, converted->polygons[0].points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[1].x, converted->polygons[0].points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[1].y, converted->polygons[0].points[1].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[2].x, converted->polygons[0].points[2].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[2].y, converted->polygons[0].points[2].y, kTol);
}

TEST(Convert2D3D, Convert3DTo2DPolygon64StampedCollectionFrames) {  // NOLINT
  auto msg_3d = cras_geo_msgs::msg::Polygon64StampedCollection();
  msg_3d.header.frame_id = kCollectionFrame;
  msg_3d.header.stamp = createTime(5);
  msg_3d.polygons.emplace_back() = createPolygon64Stamped(kCollectionFrame, createTime(5), kFlatVertices);
  msg_3d.polygons.emplace_back() = createPolygon64Stamped(kPolygonFrame, createTime(5), kFlatVertices);
  msg_3d.colors = {kFirstColor, kSecondColor};

  auto options = cras_geo_tools::ConversionOptions();
  options.z_diff_threshold = 0.0;

  // the frames are not compared unless it is requested
  const auto converted = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64StampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(converted.has_value());
  ASSERT_EQ(2u, converted->polygons.size());
  // the per-polygon colors are copied to the output
  ASSERT_EQ(2u, converted->colors.size());
  EXPECT_EQ(kFirstColor, converted->colors[0]);
  EXPECT_EQ(kSecondColor, converted->colors[1]);

  // with the frame check enabled, the mismatching polygon is reported
  options.require_same_frame = true;
  const auto mismatch = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64StampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(mismatch.has_value());
  EXPECT_TRUE(mismatch.error().frame_mismatch);
  EXPECT_FALSE(mismatch.error().z_diff_too_large);
  EXPECT_EQ("Polygon frame does not match collection frame", mismatch.error().error_message);

  // with skip_errors, the mismatching polygon is skipped instead of failing the whole conversion
  options.skip_errors = true;
  const auto skipped = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64StampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(skipped.has_value());
  EXPECT_EQ(msg_3d.header, skipped->header);
  ASSERT_EQ(1u, skipped->polygons.size());
  // the color of the skipped polygon must be skipped as well so that the colors match the polygons
  ASSERT_EQ(1u, skipped->colors.size());
  EXPECT_EQ(kFirstColor, skipped->colors[0]);
  ASSERT_EQ(3u, skipped->polygons[0].points.size());
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[0].x, skipped->polygons[0].points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[0].y, skipped->polygons[0].points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[1].x, skipped->polygons[0].points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[1].y, skipped->polygons[0].points[1].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[2].x, skipped->polygons[0].points[2].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[2].y, skipped->polygons[0].points[2].y, kTol);
}

TEST(Convert2D3D, Convert3DTo2DPolygon64StampedCollectionStamps) {  // NOLINT
  auto msg_3d = cras_geo_msgs::msg::Polygon64StampedCollection();
  msg_3d.header.frame_id = kCollectionFrame;
  msg_3d.header.stamp = createTime(5);
  msg_3d.polygons.emplace_back() = createPolygon64Stamped(kCollectionFrame, createTime(5), kFlatVertices);
  msg_3d.polygons.emplace_back() = createPolygon64Stamped(kCollectionFrame, createTime(6), kFlatVertices);
  msg_3d.colors = {kFirstColor, kSecondColor};

  auto options = cras_geo_tools::ConversionOptions();
  options.z_diff_threshold = 0.0;

  // the timestamps are not compared unless it is requested
  const auto converted = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64StampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(converted.has_value());
  ASSERT_EQ(2u, converted->polygons.size());

  // with the timestamp check enabled, the mismatching polygon is reported
  options.require_same_stamp = true;
  const auto mismatch = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64StampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(mismatch.has_value());
  EXPECT_FALSE(mismatch.error().z_diff_too_large);
  EXPECT_EQ("Polygon stamp does not match collection stamp", mismatch.error().error_message);

  // with skip_errors, the mismatching polygon is skipped instead of failing the whole conversion
  options.skip_errors = true;
  const auto skipped = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64StampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(skipped.has_value());
  ASSERT_EQ(1u, skipped->polygons.size());
  // the color of the skipped polygon must be skipped as well so that the colors match the polygons
  ASSERT_EQ(1u, skipped->colors.size());
  EXPECT_EQ(kFirstColor, skipped->colors[0]);
  ASSERT_EQ(3u, skipped->polygons[0].points.size());
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[0].x, skipped->polygons[0].points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[0].y, skipped->polygons[0].points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[1].x, skipped->polygons[0].points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[1].y, skipped->polygons[0].points[1].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[2].x, skipped->polygons[0].points[2].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[2].y, skipped->polygons[0].points[2].y, kTol);
}

TEST(Convert2D3D, Convert3DTo2DPolygon64StampedCollectionTf) {  // NOLINT
  auto msg_3d = cras_geo_msgs::msg::Polygon64StampedCollection();
  msg_3d.header.frame_id = kCollectionFrame;
  msg_3d.header.stamp = createTime(5);
  msg_3d.polygons.emplace_back() = createPolygon64Stamped(kPolygonFrame, createTime(6), kTransformedVertices);
  msg_3d.colors = {kFirstColor};

  auto options = cras_geo_tools::ConversionOptions();
  options.transform_to_header_stamp = true;
  options.z_diff_threshold = 0.0;

  // no TF buffer is available, so the polygon cannot be transformed
  const auto noTf = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64StampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(noTf.has_value());
  EXPECT_TRUE(noTf.error().tf_error);
  EXPECT_EQ("No transform buffer provided", noTf.error().error_message);

  // with skip_errors, the untransformable polygon is skipped
  options.skip_errors = true;
  const auto skipped = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64StampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(skipped.has_value());
  EXPECT_TRUE(skipped->polygons.empty());
  // the color of the skipped polygon is skipped as well
  EXPECT_TRUE(skipped->colors.empty());
  options.skip_errors = false;

  // the transform is not known in the TF buffer
  const auto emptyBuffer = std::make_unique<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  options.tf = emptyBuffer.get();
  const auto unknownTf = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64StampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(unknownTf.has_value());
  EXPECT_TRUE(unknownTf.error().tf_error);
  EXPECT_FALSE(unknownTf.error().error_message.empty());

  // with a known transform, the polygon is transformed to the collection frame
  const auto buffer = createTfBuffer();
  options.tf = buffer.get();
  const auto converted = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64StampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(converted.has_value());
  EXPECT_EQ(msg_3d.header, converted->header);
  ASSERT_EQ(1u, converted->polygons.size());
  // the color of the transformed polygon is preserved
  ASSERT_EQ(1u, converted->colors.size());
  EXPECT_EQ(kFirstColor, converted->colors[0]);
  ASSERT_EQ(3u, converted->polygons[0].points.size());
  EXPECT_NEAR(1.0 - kTransformedVertices[0].y, converted->polygons[0].points[0].x, kTol);
  EXPECT_NEAR(2.0 + kTransformedVertices[0].x, converted->polygons[0].points[0].y, kTol);
  EXPECT_NEAR(1.0 - kTransformedVertices[1].y, converted->polygons[0].points[1].x, kTol);
  EXPECT_NEAR(2.0 + kTransformedVertices[1].x, converted->polygons[0].points[1].y, kTol);
  EXPECT_NEAR(1.0 - kTransformedVertices[2].y, converted->polygons[0].points[2].x, kTol);
  EXPECT_NEAR(2.0 + kTransformedVertices[2].x, converted->polygons[0].points[2].y, kTol);
}

TEST(Convert2D3D, Convert3DTo2DPolygon64StampedCollectionTfToHeaderFrame) {  // NOLINT
  auto msg_3d = cras_geo_msgs::msg::Polygon64StampedCollection();
  msg_3d.header.frame_id = kCollectionFrame;
  msg_3d.header.stamp = createTime(5);
  msg_3d.polygons.emplace_back() = createPolygon64Stamped(kPolygonFrame, createTime(5), kFlatVertices);

  // the transform is looked up from the polygon frame to the polygon frame, which yields an identity transform
  auto options = cras_geo_tools::ConversionOptions();
  options.transform_to_header_frame = true;
  options.z_diff_threshold = 0.0;
  const auto buffer = createTfBuffer();
  options.tf = buffer.get();

  const auto converted = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64StampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(converted.has_value());
  ASSERT_EQ(1u, converted->polygons.size());
  ASSERT_EQ(3u, converted->polygons[0].points.size());
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[0].x, converted->polygons[0].points[0].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[0].y, converted->polygons[0].points[0].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[1].x, converted->polygons[0].points[1].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[1].y, converted->polygons[0].points[1].y, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[2].x, converted->polygons[0].points[2].x, kTol);
  EXPECT_NEAR(msg_3d.polygons[0].polygon.points[2].y, converted->polygons[0].points[2].y, kTol);
}

TEST(Convert2D3D, Convert3DTo2DPolygonInstanceStampedCollectionTf) {  // NOLINT
  auto msg_3d = cras_geo_msgs::msg::PolygonInstanceStampedCollection();
  msg_3d.header.frame_id = kCollectionFrame;
  msg_3d.header.stamp = createTime(5);
  msg_3d.polygons.emplace_back() = createPolygonInstanceStamped(kPolygonFrame, createTime(6), kTransformedVertices);
  msg_3d.colors = {kFirstColor};

  auto options = cras_geo_tools::ConversionOptions();
  options.transform_to_header_stamp = true;
  options.z_diff_threshold = 0.0;

  // with a known transform, the polygon is transformed to the collection frame
  const auto buffer = createTfBuffer();
  options.tf = buffer.get();
  const auto converted = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonInstanceStampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(converted.has_value());
  EXPECT_EQ(msg_3d.header, converted->header);
  ASSERT_EQ(1u, converted->polygons.size());
  // the color of the transformed polygon is preserved
  ASSERT_EQ(1u, converted->colors.size());
  EXPECT_EQ(kFirstColor, converted->colors[0]);
  ASSERT_EQ(3u, converted->polygons[0].points.size());
  EXPECT_NEAR(1.0 - kTransformedVertices[0].y, converted->polygons[0].points[0].x, kTol);
  EXPECT_NEAR(2.0 + kTransformedVertices[0].x, converted->polygons[0].points[0].y, kTol);
  EXPECT_NEAR(1.0 - kTransformedVertices[1].y, converted->polygons[0].points[1].x, kTol);
  EXPECT_NEAR(2.0 + kTransformedVertices[1].x, converted->polygons[0].points[1].y, kTol);
  EXPECT_NEAR(1.0 - kTransformedVertices[2].y, converted->polygons[0].points[2].x, kTol);
  EXPECT_NEAR(2.0 + kTransformedVertices[2].x, converted->polygons[0].points[2].y, kTol);

  // without a TF buffer, the conversion must fail
  options.tf = nullptr;
  const auto noTf = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonInstanceStampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(noTf.has_value());
  EXPECT_TRUE(noTf.error().tf_error);
  EXPECT_EQ("No transform buffer provided", noTf.error().error_message);

  // the frame check is performed before the transform is looked up
  options.require_same_frame = true;
  options.tf = buffer.get();
  const auto mismatch = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonInstanceStampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(mismatch.has_value());
  EXPECT_TRUE(mismatch.error().frame_mismatch);
  EXPECT_EQ("Polygon frame does not match collection frame", mismatch.error().error_message);
}

TEST(Convert2D3D, Convert3DTo2DPolygon64InstanceStampedCollectionTf) {  // NOLINT
  auto msg_3d = cras_geo_msgs::msg::Polygon64InstanceStampedCollection();
  msg_3d.header.frame_id = kCollectionFrame;
  msg_3d.header.stamp = createTime(5);
  msg_3d.polygons.emplace_back() = createPolygon64InstanceStamped(kPolygonFrame, createTime(6), kTransformedVertices);
  msg_3d.colors = {kFirstColor};

  auto options = cras_geo_tools::ConversionOptions();
  options.transform_to_header_stamp = true;
  options.z_diff_threshold = 0.0;

  // with a known transform, the polygon is transformed to the collection frame
  const auto buffer = createTfBuffer();
  options.tf = buffer.get();
  const auto converted = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64InstanceStampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_TRUE(converted.has_value());
  EXPECT_EQ(msg_3d.header, converted->header);
  ASSERT_EQ(1u, converted->polygons.size());
  // the color of the transformed polygon is preserved
  ASSERT_EQ(1u, converted->colors.size());
  EXPECT_EQ(kFirstColor, converted->colors[0]);
  ASSERT_EQ(3u, converted->polygons[0].points.size());
  EXPECT_NEAR(1.0 - kTransformedVertices[0].y, converted->polygons[0].points[0].x, kTol);
  EXPECT_NEAR(2.0 + kTransformedVertices[0].x, converted->polygons[0].points[0].y, kTol);
  EXPECT_NEAR(1.0 - kTransformedVertices[1].y, converted->polygons[0].points[1].x, kTol);
  EXPECT_NEAR(2.0 + kTransformedVertices[1].x, converted->polygons[0].points[1].y, kTol);
  EXPECT_NEAR(1.0 - kTransformedVertices[2].y, converted->polygons[0].points[2].x, kTol);
  EXPECT_NEAR(2.0 + kTransformedVertices[2].x, converted->polygons[0].points[2].y, kTol);

  // without a TF buffer, the conversion must fail
  options.tf = nullptr;
  const auto noTf = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64InstanceStampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(noTf.has_value());
  EXPECT_TRUE(noTf.error().tf_error);
  EXPECT_EQ("No transform buffer provided", noTf.error().error_message);

  // the frame check is performed before the transform is looked up
  options.require_same_frame = true;
  options.tf = buffer.get();
  const auto mismatch = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64InstanceStampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, options);
  ASSERT_FALSE(mismatch.has_value());
  EXPECT_TRUE(mismatch.error().frame_mismatch);
  EXPECT_EQ("Polygon frame does not match collection frame", mismatch.error().error_message);
}

TEST(Convert2D3D, RoundTrip) {  // NOLINT
  auto msg_2d = polygon_msgs::msg::Polygon2DCollection();
  msg_2d.header.frame_id = kCollectionFrame;
  msg_2d.header.stamp = createTime(7, 8);
  msg_2d.polygons = {createPolygon2D(kFlatVertices), createPolygon2D({{0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}})};
  msg_2d.colors = {kFirstColor, kSecondColor};

  const auto msg_3d = cras_geo_tools::convert2DTo3D<polygon_msgs::msg::Polygon2DCollection,
      cras_geo_msgs::msg::PolygonCollection>(msg_2d);
  const auto msg_2d_back = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonCollection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, cras_geo_tools::ConversionOptions());
  ASSERT_TRUE(msg_2d_back.has_value());

  EXPECT_EQ(msg_2d.header, msg_2d_back->header);
  ASSERT_EQ(2u, msg_2d_back->polygons.size());
  // the per-polygon colors are not affected by the round trip
  ASSERT_EQ(2u, msg_3d.colors.size());
  EXPECT_EQ(kFirstColor, msg_3d.colors[0]);
  EXPECT_EQ(kSecondColor, msg_3d.colors[1]);
  ASSERT_EQ(2u, msg_2d_back->colors.size());
  EXPECT_EQ(kFirstColor, msg_2d_back->colors[0]);
  EXPECT_EQ(kSecondColor, msg_2d_back->colors[1]);

  // the coordinates pass through a 32-bit floating point type, which limits the achievable accuracy
  ASSERT_EQ(3u, msg_2d_back->polygons[0].points.size());
  EXPECT_NEAR(msg_2d.polygons[0].points[0].x, msg_2d_back->polygons[0].points[0].x, kTol);
  EXPECT_NEAR(msg_2d.polygons[0].points[0].y, msg_2d_back->polygons[0].points[0].y, kTol);
  EXPECT_NEAR(msg_2d.polygons[0].points[1].x, msg_2d_back->polygons[0].points[1].x, kTol);
  EXPECT_NEAR(msg_2d.polygons[0].points[1].y, msg_2d_back->polygons[0].points[1].y, kTol);
  EXPECT_NEAR(msg_2d.polygons[0].points[2].x, msg_2d_back->polygons[0].points[2].x, kTol);
  EXPECT_NEAR(msg_2d.polygons[0].points[2].y, msg_2d_back->polygons[0].points[2].y, kTol);

  ASSERT_EQ(2u, msg_2d_back->polygons[1].points.size());
  EXPECT_NEAR(msg_2d.polygons[1].points[0].x, msg_2d_back->polygons[1].points[0].x, kTol);
  EXPECT_NEAR(msg_2d.polygons[1].points[0].y, msg_2d_back->polygons[1].points[0].y, kTol);
  EXPECT_NEAR(msg_2d.polygons[1].points[1].x, msg_2d_back->polygons[1].points[1].x, kTol);
  EXPECT_NEAR(msg_2d.polygons[1].points[1].y, msg_2d_back->polygons[1].points[1].y, kTol);
}

TEST(Convert2D3D, RoundTripPolygon64) {  // NOLINT
  auto msg_2d = polygon_msgs::msg::Polygon2DCollection();
  msg_2d.header.frame_id = kCollectionFrame;
  msg_2d.header.stamp = createTime(7, 8);
  msg_2d.polygons = {createPolygon2D(kFlatVertices), createPolygon2D({{0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}})};
  msg_2d.colors = {kFirstColor, kSecondColor};

  const auto msg_3d = cras_geo_tools::convert2DTo3D<polygon_msgs::msg::Polygon2DCollection,
      cras_geo_msgs::msg::Polygon64Collection>(msg_2d);
  const auto msg_2d_back = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64Collection,
      polygon_msgs::msg::Polygon2DCollection>(msg_3d, cras_geo_tools::ConversionOptions());
  ASSERT_TRUE(msg_2d_back.has_value());

  EXPECT_EQ(msg_2d.header, msg_2d_back->header);
  // the per-polygon colors are not affected by the round trip
  ASSERT_EQ(2u, msg_3d.colors.size());
  EXPECT_EQ(kFirstColor, msg_3d.colors[0]);
  EXPECT_EQ(kSecondColor, msg_3d.colors[1]);
  ASSERT_EQ(2u, msg_2d_back->colors.size());
  EXPECT_EQ(kFirstColor, msg_2d_back->colors[0]);
  EXPECT_EQ(kSecondColor, msg_2d_back->colors[1]);

  // the coordinates pass through a double-precision type, so the round trip is lossless
  EXPECT_EQ(msg_2d.polygons, msg_2d_back->polygons);

  const auto polygon_2d = createPolygon2DStamped(kPolygonFrame, createTime(9), kFlatVertices);
  const auto polygon_3d = cras_geo_tools::convert2DTo3D<polygon_msgs::msg::Polygon2DStamped,
      cras_geo_msgs::msg::Polygon64Stamped>(polygon_2d);
  const auto polygon_2d_back = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64Stamped,
      polygon_msgs::msg::Polygon2DStamped>(polygon_3d, cras_geo_tools::ConversionOptions());
  ASSERT_TRUE(polygon_2d_back.has_value());
  EXPECT_EQ(polygon_2d, polygon_2d_back);
}

TEST(Convert2D3D, EmptyPolygons) {  // NOLINT
  const auto msg_2d = createPolygon2DStamped(kPolygonFrame, createTime(1), {});
  const auto msg_3d = cras_geo_tools::convert2DTo3D<polygon_msgs::msg::Polygon2DStamped,
      geometry_msgs::msg::PolygonStamped>(msg_2d);
  EXPECT_TRUE(msg_3d.polygon.points.empty());

  const auto back = cras_geo_tools::convert3DTo2D<geometry_msgs::msg::PolygonStamped,
      polygon_msgs::msg::Polygon2DStamped>(msg_3d, cras_geo_tools::ConversionOptions());
  ASSERT_TRUE(back.has_value());
  EXPECT_TRUE(back->polygon.points.empty());

  const auto emptyCollection = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::PolygonCollection,
      polygon_msgs::msg::Polygon2DCollection>(cras_geo_msgs::msg::PolygonCollection(),
      cras_geo_tools::ConversionOptions());
  ASSERT_TRUE(emptyCollection.has_value());
  EXPECT_TRUE(emptyCollection->polygons.empty());
  EXPECT_TRUE(emptyCollection->colors.empty());
}

TEST(Convert2D3D, EmptyPolygonsPolygon64) {  // NOLINT
  const auto msg_2d = createPolygon2DStamped(kPolygonFrame, createTime(1), {});
  const auto msg_3d = cras_geo_tools::convert2DTo3D<polygon_msgs::msg::Polygon2DStamped,
      cras_geo_msgs::msg::Polygon64Stamped>(msg_2d);
  EXPECT_TRUE(msg_3d.polygon.points.empty());

  const auto back = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64Stamped,
      polygon_msgs::msg::Polygon2DStamped>(msg_3d, cras_geo_tools::ConversionOptions());
  ASSERT_TRUE(back.has_value());
  EXPECT_TRUE(back->polygon.points.empty());

  const auto emptyCollection = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64Collection,
      polygon_msgs::msg::Polygon2DCollection>(cras_geo_msgs::msg::Polygon64Collection(),
      cras_geo_tools::ConversionOptions());
  ASSERT_TRUE(emptyCollection.has_value());
  EXPECT_TRUE(emptyCollection->polygons.empty());
  EXPECT_TRUE(emptyCollection->colors.empty());

  // an empty polygon has no Z-coordinates to compare, so it passes even if flatness is required
  auto options = cras_geo_tools::ConversionOptions();
  options.z_diff_threshold = 0.0;
  options.collection_z_diff_threshold = 0.0;
  const auto emptyStamped = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64Stamped,
      polygon_msgs::msg::Polygon2DStamped>(cras_geo_msgs::msg::Polygon64Stamped(), options);
  ASSERT_TRUE(emptyStamped.has_value());
  EXPECT_TRUE(emptyStamped->polygon.points.empty());

  auto emptyStampedCollection = cras_geo_msgs::msg::Polygon64StampedCollection();
  emptyStampedCollection.polygons.emplace_back();
  const auto emptyStampedColl = cras_geo_tools::convert3DTo2D<cras_geo_msgs::msg::Polygon64StampedCollection,
      polygon_msgs::msg::Polygon2DCollection>(emptyStampedCollection, options);
  ASSERT_TRUE(emptyStampedColl.has_value());
  ASSERT_EQ(1u, emptyStampedColl->polygons.size());
  EXPECT_TRUE(emptyStampedColl->polygons[0].points.empty());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
