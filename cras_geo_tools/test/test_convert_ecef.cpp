// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for convert_ecef.hpp.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <cmath>
#include <ostream>
#include <vector>

#include <cras_geo_msgs/msg/geo_polygon2_d.hpp>
#include <cras_geo_msgs/msg/geo_polygon2_d_collection.hpp>
#include <cras_geo_msgs/msg/polygon64.hpp>
#include <cras_geo_msgs/msg/polygon64_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_instance.hpp>
#include <cras_geo_msgs/msg/polygon64_instance_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_instance_stamped.hpp>
#include <cras_geo_msgs/msg/polygon64_instance_stamped_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_stamped.hpp>
#include <cras_geo_msgs/msg/polygon64_stamped_collection.hpp>
#include <cras_geo_tools/convert_ecef.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <geographic_msgs/msg/geo_pose.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>

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

// WGS84 semi-major axis [m].
constexpr double kEquatorialRadius = 6378137.0;
// WGS84 semi-minor axis [m].
constexpr double kPolarRadius = 6356752.314245179;

// Tolerances of the double-precision conversions.
constexpr double kLlhTol = 1e-9;  // deg
constexpr double kAltTol = 1e-5;  // m
constexpr double kEcefTol = 1e-3;  // m

/**
 * \brief Create an ECEF point with the given coordinates.
 * \param[in] x X-coordinate [m].
 * \param[in] y Y-coordinate [m].
 * \param[in] z Z-coordinate [m].
 * \return The created point.
 */
geometry_msgs::msg::Point createEcefPoint(const double x, const double y, const double z) {
  auto point = geometry_msgs::msg::Point();
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

/**
 * \brief Create a geo point with the given coordinates.
 * \param[in] latitude Latitude [deg].
 * \param[in] longitude Longitude [deg].
 * \param[in] altitude Altitude above the WGS84 ellipsoid [m].
 * \return The created point.
 */
geographic_msgs::msg::GeoPoint createGeoPoint(const double latitude, const double longitude, const double altitude) {
  auto point = geographic_msgs::msg::GeoPoint();
  point.latitude = latitude;
  point.longitude = longitude;
  point.altitude = altitude;
  return point;
}

/**
 * \brief Create a geo pose with the given position and identity orientation.
 * \param[in] position The position of the pose.
 * \return The created pose.
 */
geographic_msgs::msg::GeoPose createGeoPose(const geographic_msgs::msg::GeoPoint& position) {
  auto pose = geographic_msgs::msg::GeoPose();
  pose.position = position;
  pose.orientation.w = 1.0;
  return pose;
}

/**
 * \brief Create a geo polygon with the given vertices.
 * \param[in] vertices The vertices of the polygon.
 * \return The created polygon.
 */
cras_geo_msgs::msg::GeoPolygon2D createGeoPolygon(const std::vector<geographic_msgs::msg::GeoPoint>& vertices) {
  auto polygon = cras_geo_msgs::msg::GeoPolygon2D();
  polygon.points = vertices;
  return polygon;
}

/**
 * \brief Create an ECEF polygon with the given vertices.
 * \param[in] vertices The vertices of the polygon.
 * \return The created polygon.
 */
cras_geo_msgs::msg::Polygon64 createEcefPolygon(const std::vector<geographic_msgs::msg::GeoPoint>& vertices) {
  auto polygon = cras_geo_msgs::msg::Polygon64();
  for (const auto& vertex : vertices) {
    polygon.points.emplace_back() =
      cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(vertex);
  }
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

/**
 * \brief One of the tested locations (a point in Prague).
 */
const auto kLlh1 = createGeoPoint(50.0875, 14.4213, 200.0);

/**
 * \brief One of the tested locations (a point in Brno).
 */
const auto kLlh2 = createGeoPoint(49.195, 16.606, 240.0);

/**
 * \brief One of the tested locations (a point in the Pacific ocean, below the ellipsoid surface).
 */
const auto kLlh3 = createGeoPoint(-33.0, -70.5, -100.0);

}  // namespace

TEST(ConvertECEF, ECEFToLLHPoint) {  // NOLINT
  // points on the WGS84 ellipsoid surface with known latitude and longitude
  const auto llhEquatorX = cras_geo_tools::ecefToLlh<geometry_msgs::msg::Point, geographic_msgs::msg::GeoPoint>(
    createEcefPoint(kEquatorialRadius, 0.0, 0.0));
  EXPECT_NEAR(0.0, llhEquatorX.latitude, kLlhTol);
  EXPECT_NEAR(0.0, llhEquatorX.longitude, kLlhTol);
  EXPECT_NEAR(0.0, llhEquatorX.altitude, kAltTol);

  const auto llhEquatorY = cras_geo_tools::ecefToLlh<geometry_msgs::msg::Point, geographic_msgs::msg::GeoPoint>(
    createEcefPoint(0.0, kEquatorialRadius, 0.0));
  EXPECT_NEAR(0.0, llhEquatorY.latitude, kLlhTol);
  EXPECT_NEAR(90.0, llhEquatorY.longitude, kLlhTol);
  EXPECT_NEAR(0.0, llhEquatorY.altitude, kAltTol);

  const auto llhNorthPole = cras_geo_tools::ecefToLlh<geometry_msgs::msg::Point, geographic_msgs::msg::GeoPoint>(
    createEcefPoint(0.0, 0.0, kPolarRadius));
  EXPECT_NEAR(90.0, llhNorthPole.latitude, kLlhTol);
  EXPECT_NEAR(0.0, llhNorthPole.longitude, kLlhTol);
  EXPECT_NEAR(0.0, llhNorthPole.altitude, kAltTol);

  // a point computed by an independent implementation of the conversion
  const auto ecef = createEcefPoint(3970913.1943419, 1018818.5082705, 4870161.4762244);
  const auto llhKnown = cras_geo_tools::ecefToLlh<geometry_msgs::msg::Point, geographic_msgs::msg::GeoPoint>(ecef);
  EXPECT_NEAR(50.1, llhKnown.latitude, kLlhTol);
  EXPECT_NEAR(14.39, llhKnown.longitude, kLlhTol);
  EXPECT_NEAR(300.0, llhKnown.altitude, kAltTol);
}

TEST(ConvertECEF, LLHToECEFPoint) {  // NOLINT
  // points on the WGS84 ellipsoid surface with known ECEF coordinates
  const auto ecefEquatorX = cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(
    createGeoPoint(0.0, 0.0, 0.0));
  EXPECT_NEAR(kEquatorialRadius, ecefEquatorX.x, kEcefTol);
  EXPECT_NEAR(0.0, ecefEquatorX.y, kEcefTol);
  EXPECT_NEAR(0.0, ecefEquatorX.z, kEcefTol);

  const auto ecefEquatorY = cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(
    createGeoPoint(0.0, 90.0, 0.0));
  EXPECT_NEAR(0.0, ecefEquatorY.x, kEcefTol);
  EXPECT_NEAR(kEquatorialRadius, ecefEquatorY.y, kEcefTol);
  EXPECT_NEAR(0.0, ecefEquatorY.z, kEcefTol);

  const auto ecefNorthPole = cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(
    createGeoPoint(90.0, 0.0, 0.0));
  EXPECT_NEAR(0.0, ecefNorthPole.x, kEcefTol);
  EXPECT_NEAR(0.0, ecefNorthPole.y, kEcefTol);
  EXPECT_NEAR(kPolarRadius, ecefNorthPole.z, kEcefTol);

  // a point computed by an independent implementation of the conversion
  const auto ecefKnown = cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(kLlh3);
  EXPECT_NEAR(1787332.4160579, ecefKnown.x, kEcefTol);
  EXPECT_NEAR(-5047271.0405578, ecefKnown.y, kEcefTol);
  EXPECT_NEAR(-3453904.1772742, ecefKnown.z, kEcefTol);
}

TEST(ConvertECEF, PointRoundTrip) {  // NOLINT
  const auto ecef1 = cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(kLlh1);
  const auto llh1Back = cras_geo_tools::ecefToLlh<geometry_msgs::msg::Point, geographic_msgs::msg::GeoPoint>(ecef1);
  EXPECT_NEAR(kLlh1.latitude, llh1Back.latitude, kLlhTol);
  EXPECT_NEAR(kLlh1.longitude, llh1Back.longitude, kLlhTol);
  EXPECT_NEAR(kLlh1.altitude, llh1Back.altitude, kAltTol);

  const auto ecef1_2 = cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(kLlh1);
  EXPECT_NEAR(ecef1.x, ecef1_2.x, kEcefTol);
  EXPECT_NEAR(ecef1.y, ecef1_2.y, kEcefTol);
  EXPECT_NEAR(ecef1.z, ecef1_2.z, kEcefTol);

  const auto llh1Back_2 =
    cras_geo_tools::ecefToLlh<geometry_msgs::msg::Point, geographic_msgs::msg::GeoPoint>(ecef1_2);
  EXPECT_NEAR(kLlh1.latitude, llh1Back_2.latitude, kLlhTol);
  EXPECT_NEAR(kLlh1.longitude, llh1Back_2.longitude, kLlhTol);
  EXPECT_NEAR(kLlh1.altitude, llh1Back_2.altitude, kAltTol);

  const auto ecef2 = cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(kLlh2);
  const auto llh2Back = cras_geo_tools::ecefToLlh<geometry_msgs::msg::Point, geographic_msgs::msg::GeoPoint>(ecef2);
  EXPECT_NEAR(kLlh2.latitude, llh2Back.latitude, kLlhTol);
  EXPECT_NEAR(kLlh2.longitude, llh2Back.longitude, kLlhTol);
  EXPECT_NEAR(kLlh2.altitude, llh2Back.altitude, kAltTol);

  const auto ecef2_2 = cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(kLlh2);
  EXPECT_NEAR(ecef2.x, ecef2_2.x, kEcefTol);
  EXPECT_NEAR(ecef2.y, ecef2_2.y, kEcefTol);
  EXPECT_NEAR(ecef2.z, ecef2_2.z, kEcefTol);

  const auto llh2Back_2 =
    cras_geo_tools::ecefToLlh<geometry_msgs::msg::Point, geographic_msgs::msg::GeoPoint>(ecef2_2);
  EXPECT_NEAR(kLlh2.latitude, llh2Back_2.latitude, kLlhTol);
  EXPECT_NEAR(kLlh2.longitude, llh2Back_2.longitude, kLlhTol);
  EXPECT_NEAR(kLlh2.altitude, llh2Back_2.altitude, kAltTol);

  const auto ecef3 = cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(kLlh3);
  const auto llh3Back = cras_geo_tools::ecefToLlh<geometry_msgs::msg::Point, geographic_msgs::msg::GeoPoint>(ecef3);
  EXPECT_NEAR(kLlh3.latitude, llh3Back.latitude, kLlhTol);
  EXPECT_NEAR(kLlh3.longitude, llh3Back.longitude, kLlhTol);
  EXPECT_NEAR(kLlh3.altitude, llh3Back.altitude, kAltTol);

  const auto ecef3_2 = cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(kLlh3);
  EXPECT_NEAR(ecef3.x, ecef3_2.x, kEcefTol);
  EXPECT_NEAR(ecef3.y, ecef3_2.y, kEcefTol);
  EXPECT_NEAR(ecef3.z, ecef3_2.z, kEcefTol);

  const auto llh3Back_2 =
    cras_geo_tools::ecefToLlh<geometry_msgs::msg::Point, geographic_msgs::msg::GeoPoint>(ecef3_2);
  EXPECT_NEAR(kLlh3.latitude, llh3Back_2.latitude, kLlhTol);
  EXPECT_NEAR(kLlh3.longitude, llh3Back_2.longitude, kLlhTol);
  EXPECT_NEAR(kLlh3.altitude, llh3Back_2.altitude, kAltTol);
}

TEST(ConvertECEF, PointStamped) {  // NOLINT
  auto ecef = geometry_msgs::msg::PointStamped();
  ecef.header.stamp.sec = 5;
  ecef.header.stamp.nanosec = 6;
  ecef.header.frame_id = "ecef_frame";
  ecef.point = cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(kLlh1);

  const auto llh = cras_geo_tools::ecefToLlh<geometry_msgs::msg::PointStamped, geographic_msgs::msg::GeoPointStamped>(
    ecef);
  EXPECT_EQ(ecef.header.stamp, llh.header.stamp);
  EXPECT_EQ("wgs84", llh.header.frame_id);  // the frame is always replaced by the ellipsoid name
  EXPECT_NEAR(kLlh1.latitude, llh.position.latitude, kLlhTol);
  EXPECT_NEAR(kLlh1.longitude, llh.position.longitude, kLlhTol);
  EXPECT_NEAR(kLlh1.altitude, llh.position.altitude, kAltTol);

  const auto ecef_back =
    cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPointStamped, geometry_msgs::msg::PointStamped>(llh);
  EXPECT_EQ(llh.header.stamp, ecef_back.header.stamp);
  EXPECT_EQ("ecef", ecef_back.header.frame_id);  // the frame is always replaced by the ECEF frame name
  EXPECT_NEAR(ecef.point.x, ecef_back.point.x, kEcefTol);
  EXPECT_NEAR(ecef.point.y, ecef_back.point.y, kEcefTol);
  EXPECT_NEAR(ecef.point.z, ecef_back.point.z, kEcefTol);
}

TEST(ConvertECEF, Pose) {  // NOLINT
  auto llh_pose = createGeoPose(kLlh2);
  llh_pose.orientation.x = 0.5;
  llh_pose.orientation.y = -0.5;
  llh_pose.orientation.z = 0.5;
  llh_pose.orientation.w = 0.5;

  const auto expectedPosition =
    cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(kLlh2);
  const auto ecef_pose =
    cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPose, geometry_msgs::msg::Pose>(llh_pose);
  EXPECT_NEAR(expectedPosition.x, ecef_pose.position.x, kEcefTol);
  EXPECT_NEAR(expectedPosition.y, ecef_pose.position.y, kEcefTol);
  EXPECT_NEAR(expectedPosition.z, ecef_pose.position.z, kEcefTol);
  // the orientation is not rotated between the frames, it is only passed through
  EXPECT_DOUBLE_EQ(llh_pose.orientation.x, ecef_pose.orientation.x);
  EXPECT_DOUBLE_EQ(llh_pose.orientation.y, ecef_pose.orientation.y);
  EXPECT_DOUBLE_EQ(llh_pose.orientation.z, ecef_pose.orientation.z);
  EXPECT_DOUBLE_EQ(llh_pose.orientation.w, ecef_pose.orientation.w);
  const auto ecefNorm = std::sqrt(
    ecef_pose.orientation.x * ecef_pose.orientation.x + ecef_pose.orientation.y * ecef_pose.orientation.y +
    ecef_pose.orientation.z * ecef_pose.orientation.z + ecef_pose.orientation.w * ecef_pose.orientation.w);
  EXPECT_NEAR(1.0, ecefNorm, 1e-9);

  const auto llh_pose_back =
    cras_geo_tools::ecefToLlh<geometry_msgs::msg::Pose, geographic_msgs::msg::GeoPose>(ecef_pose);
  EXPECT_NEAR(kLlh2.latitude, llh_pose_back.position.latitude, kLlhTol);
  EXPECT_NEAR(kLlh2.longitude, llh_pose_back.position.longitude, kLlhTol);
  EXPECT_NEAR(kLlh2.altitude, llh_pose_back.position.altitude, kAltTol);
  EXPECT_DOUBLE_EQ(llh_pose.orientation.x, llh_pose_back.orientation.x);
  EXPECT_DOUBLE_EQ(llh_pose.orientation.y, llh_pose_back.orientation.y);
  EXPECT_DOUBLE_EQ(llh_pose.orientation.z, llh_pose_back.orientation.z);
  EXPECT_DOUBLE_EQ(llh_pose.orientation.w, llh_pose_back.orientation.w);
}

TEST(ConvertECEF, PoseStamped) {  // NOLINT
  auto ecef = geometry_msgs::msg::PoseStamped();
  ecef.header.stamp.sec = 7;
  ecef.header.stamp.nanosec = 8;
  ecef.header.frame_id = "ecef_frame";
  ecef.pose.position = cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(kLlh3);
  ecef.pose.orientation.w = 1.0;

  const auto llh = cras_geo_tools::ecefToLlh<geometry_msgs::msg::PoseStamped, geographic_msgs::msg::GeoPoseStamped>(
    ecef);
  EXPECT_EQ(ecef.header.stamp, llh.header.stamp);
  EXPECT_EQ("wgs84", llh.header.frame_id);
  EXPECT_NEAR(kLlh3.latitude, llh.pose.position.latitude, kLlhTol);
  EXPECT_NEAR(kLlh3.longitude, llh.pose.position.longitude, kLlhTol);
  EXPECT_NEAR(kLlh3.altitude, llh.pose.position.altitude, kAltTol);
  const auto llhNorm = std::sqrt(
    llh.pose.orientation.x * llh.pose.orientation.x + llh.pose.orientation.y * llh.pose.orientation.y +
    llh.pose.orientation.z * llh.pose.orientation.z + llh.pose.orientation.w * llh.pose.orientation.w);
  EXPECT_NEAR(1.0, llhNorm, 1e-9);

  const auto ecef_back =
    cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoseStamped, geometry_msgs::msg::PoseStamped>(llh);
  EXPECT_EQ(llh.header.stamp, ecef_back.header.stamp);
  EXPECT_EQ("ecef", ecef_back.header.frame_id);
  EXPECT_NEAR(ecef.pose.position.x, ecef_back.pose.position.x, kEcefTol);
  EXPECT_NEAR(ecef.pose.position.y, ecef_back.pose.position.y, kEcefTol);
  EXPECT_NEAR(ecef.pose.position.z, ecef_back.pose.position.z, kEcefTol);
}

TEST(ConvertECEF, Polygon) {  // NOLINT
  const auto ecef = createEcefPolygon({kLlh1, kLlh2, kLlh3});
  ASSERT_EQ(3u, ecef.points.size());

  const auto llh = cras_geo_tools::ecefToLlh<cras_geo_msgs::msg::Polygon64, cras_geo_msgs::msg::GeoPolygon2D>(ecef);
  ASSERT_EQ(3u, llh.points.size());
  EXPECT_NEAR(kLlh1.latitude, llh.points[0].latitude, kLlhTol);
  EXPECT_NEAR(kLlh1.longitude, llh.points[0].longitude, kLlhTol);
  EXPECT_NEAR(kLlh1.altitude, llh.points[0].altitude, kAltTol);
  EXPECT_NEAR(kLlh2.latitude, llh.points[1].latitude, kLlhTol);
  EXPECT_NEAR(kLlh2.longitude, llh.points[1].longitude, kLlhTol);
  EXPECT_NEAR(kLlh2.altitude, llh.points[1].altitude, kAltTol);
  EXPECT_NEAR(kLlh3.latitude, llh.points[2].latitude, kLlhTol);
  EXPECT_NEAR(kLlh3.longitude, llh.points[2].longitude, kLlhTol);
  EXPECT_NEAR(kLlh3.altitude, llh.points[2].altitude, kAltTol);

  const auto ecef_back =
    cras_geo_tools::llhtoEcef<cras_geo_msgs::msg::GeoPolygon2D, cras_geo_msgs::msg::Polygon64>(llh);
  ASSERT_EQ(3u, ecef_back.points.size());
  EXPECT_NEAR(ecef.points[0].x, ecef_back.points[0].x, kEcefTol);
  EXPECT_NEAR(ecef.points[0].y, ecef_back.points[0].y, kEcefTol);
  EXPECT_NEAR(ecef.points[0].z, ecef_back.points[0].z, kEcefTol);
  EXPECT_NEAR(ecef.points[1].x, ecef_back.points[1].x, kEcefTol);
  EXPECT_NEAR(ecef.points[1].y, ecef_back.points[1].y, kEcefTol);
  EXPECT_NEAR(ecef.points[1].z, ecef_back.points[1].z, kEcefTol);
  EXPECT_NEAR(ecef.points[2].x, ecef_back.points[2].x, kEcefTol);
  EXPECT_NEAR(ecef.points[2].y, ecef_back.points[2].y, kEcefTol);
  EXPECT_NEAR(ecef.points[2].z, ecef_back.points[2].z, kEcefTol);
}

TEST(ConvertECEF, StampedPolygons) {  // NOLINT
  const auto ecef = createEcefPolygon({kLlh1, kLlh2, kLlh3});

  auto ecef_stamped = cras_geo_msgs::msg::Polygon64Stamped();
  ecef_stamped.header.frame_id = "ecef_frame";
  ecef_stamped.polygon = ecef;

  auto ecef_instance = cras_geo_msgs::msg::Polygon64Instance();
  ecef_instance.id = 42;
  ecef_instance.polygon = ecef;

  auto ecef_instance_stamped = cras_geo_msgs::msg::Polygon64InstanceStamped();
  ecef_instance_stamped.header.frame_id = "ecef_frame";
  ecef_instance_stamped.polygon = ecef_instance;

  // the header and the polygon instance ID are dropped by the conversion (the target type has no such field)
  const auto llhStamped =
    cras_geo_tools::ecefToLlh<cras_geo_msgs::msg::Polygon64Stamped, cras_geo_msgs::msg::GeoPolygon2D>(ecef_stamped);
  const auto llhInstance =
    cras_geo_tools::ecefToLlh<cras_geo_msgs::msg::Polygon64Instance, cras_geo_msgs::msg::GeoPolygon2D>(ecef_instance);
  const auto llhInstanceStamped =
    cras_geo_tools::ecefToLlh<cras_geo_msgs::msg::Polygon64InstanceStamped, cras_geo_msgs::msg::GeoPolygon2D>(
      ecef_instance_stamped);

  ASSERT_EQ(3u, llhStamped.points.size());
  EXPECT_NEAR(kLlh1.latitude, llhStamped.points[0].latitude, kLlhTol);
  EXPECT_NEAR(kLlh1.longitude, llhStamped.points[0].longitude, kLlhTol);
  EXPECT_NEAR(kLlh1.altitude, llhStamped.points[0].altitude, kAltTol);
  EXPECT_NEAR(kLlh2.latitude, llhStamped.points[1].latitude, kLlhTol);
  EXPECT_NEAR(kLlh2.longitude, llhStamped.points[1].longitude, kLlhTol);
  EXPECT_NEAR(kLlh2.altitude, llhStamped.points[1].altitude, kAltTol);
  EXPECT_NEAR(kLlh3.latitude, llhStamped.points[2].latitude, kLlhTol);
  EXPECT_NEAR(kLlh3.longitude, llhStamped.points[2].longitude, kLlhTol);
  EXPECT_NEAR(kLlh3.altitude, llhStamped.points[2].altitude, kAltTol);

  ASSERT_EQ(3u, llhInstance.points.size());
  EXPECT_NEAR(kLlh1.latitude, llhInstance.points[0].latitude, kLlhTol);
  EXPECT_NEAR(kLlh1.longitude, llhInstance.points[0].longitude, kLlhTol);
  EXPECT_NEAR(kLlh1.altitude, llhInstance.points[0].altitude, kAltTol);
  EXPECT_NEAR(kLlh2.latitude, llhInstance.points[1].latitude, kLlhTol);
  EXPECT_NEAR(kLlh2.longitude, llhInstance.points[1].longitude, kLlhTol);
  EXPECT_NEAR(kLlh2.altitude, llhInstance.points[1].altitude, kAltTol);
  EXPECT_NEAR(kLlh3.latitude, llhInstance.points[2].latitude, kLlhTol);
  EXPECT_NEAR(kLlh3.longitude, llhInstance.points[2].longitude, kLlhTol);
  EXPECT_NEAR(kLlh3.altitude, llhInstance.points[2].altitude, kAltTol);

  ASSERT_EQ(3u, llhInstanceStamped.points.size());
  EXPECT_NEAR(kLlh1.latitude, llhInstanceStamped.points[0].latitude, kLlhTol);
  EXPECT_NEAR(kLlh1.longitude, llhInstanceStamped.points[0].longitude, kLlhTol);
  EXPECT_NEAR(kLlh1.altitude, llhInstanceStamped.points[0].altitude, kAltTol);
  EXPECT_NEAR(kLlh2.latitude, llhInstanceStamped.points[1].latitude, kLlhTol);
  EXPECT_NEAR(kLlh2.longitude, llhInstanceStamped.points[1].longitude, kLlhTol);
  EXPECT_NEAR(kLlh2.altitude, llhInstanceStamped.points[1].altitude, kAltTol);
  EXPECT_NEAR(kLlh3.latitude, llhInstanceStamped.points[2].latitude, kLlhTol);
  EXPECT_NEAR(kLlh3.longitude, llhInstanceStamped.points[2].longitude, kLlhTol);
  EXPECT_NEAR(kLlh3.altitude, llhInstanceStamped.points[2].altitude, kAltTol);
}

TEST(ConvertECEF, PolygonCollections) {  // NOLINT
  const auto ecef1 = createEcefPolygon({kLlh1, kLlh2});
  const auto ecef2 = createEcefPolygon({kLlh2, kLlh3});

  // the colors of the two polygons differ so that a possible mix-up of the polygons and their colors is detected
  const auto firstColor = createColor(1.0, 0.0, 0.0);
  const auto secondColor = createColor(0.0, 1.0, 0.2);

  auto ecef_collection = cras_geo_msgs::msg::Polygon64Collection();
  ecef_collection.header.stamp.sec = 9;
  ecef_collection.header.stamp.nanosec = 10;
  ecef_collection.header.frame_id = "ecef_frame";
  ecef_collection.polygons = {ecef1, ecef2};
  ecef_collection.colors = {firstColor, secondColor};

  // the instance IDs and the per-polygon headers are dropped by the conversion (the target has no field for them)
  auto ecef_instance_collection = cras_geo_msgs::msg::Polygon64InstanceCollection();
  ecef_instance_collection.header = ecef_collection.header;
  ecef_instance_collection.polygons = {
    cras_geo_msgs::msg::Polygon64Instance{}, cras_geo_msgs::msg::Polygon64Instance{}};
  ecef_instance_collection.polygons[0].polygon = ecef1;
  ecef_instance_collection.polygons[1].polygon = ecef2;
  ecef_instance_collection.polygons[0].id = 7;
  ecef_instance_collection.polygons[1].id = 3;
  ecef_instance_collection.colors = ecef_collection.colors;

  auto ecef_stamped_collection = cras_geo_msgs::msg::Polygon64StampedCollection();
  ecef_stamped_collection.header = ecef_collection.header;
  ecef_stamped_collection.polygons = {cras_geo_msgs::msg::Polygon64Stamped{}, cras_geo_msgs::msg::Polygon64Stamped{}};
  ecef_stamped_collection.polygons[0].polygon = ecef1;
  ecef_stamped_collection.polygons[1].polygon = ecef2;
  ecef_stamped_collection.polygons[0].header.frame_id = "ecef_frame_1";
  ecef_stamped_collection.polygons[0].header.stamp.sec = 21;
  ecef_stamped_collection.polygons[1].header.frame_id = "ecef_frame_2";
  ecef_stamped_collection.polygons[1].header.stamp.sec = 22;
  ecef_stamped_collection.colors = ecef_collection.colors;

  auto ecef_instance_stamped_collection = cras_geo_msgs::msg::Polygon64InstanceStampedCollection();
  ecef_instance_stamped_collection.header = ecef_collection.header;
  ecef_instance_stamped_collection.polygons = {
    cras_geo_msgs::msg::Polygon64InstanceStamped{}, cras_geo_msgs::msg::Polygon64InstanceStamped{}};
  ecef_instance_stamped_collection.polygons[0].polygon.polygon = ecef1;
  ecef_instance_stamped_collection.polygons[1].polygon.polygon = ecef2;
  ecef_instance_stamped_collection.polygons[0].polygon.id = 7;
  ecef_instance_stamped_collection.polygons[1].polygon.id = 3;
  ecef_instance_stamped_collection.polygons[0].header.frame_id = "ecef_frame_1";
  ecef_instance_stamped_collection.polygons[0].header.stamp.sec = 21;
  ecef_instance_stamped_collection.polygons[1].header.frame_id = "ecef_frame_2";
  ecef_instance_stamped_collection.polygons[1].header.stamp.sec = 22;
  ecef_instance_stamped_collection.colors = ecef_collection.colors;

  const auto llhCollection =
    cras_geo_tools::ecefToLlh<cras_geo_msgs::msg::Polygon64Collection, cras_geo_msgs::msg::GeoPolygon2DCollection>(
      ecef_collection);
  const auto llhInstanceCollection =
    cras_geo_tools::ecefToLlh<cras_geo_msgs::msg::Polygon64InstanceCollection,
        cras_geo_msgs::msg::GeoPolygon2DCollection>(ecef_instance_collection);
  const auto llhStampedCollection =
    cras_geo_tools::ecefToLlh<cras_geo_msgs::msg::Polygon64StampedCollection,
        cras_geo_msgs::msg::GeoPolygon2DCollection>(ecef_stamped_collection);
  const auto llhInstanceStampedCollection =
    cras_geo_tools::ecefToLlh<cras_geo_msgs::msg::Polygon64InstanceStampedCollection,
        cras_geo_msgs::msg::GeoPolygon2DCollection>(ecef_instance_stamped_collection);

  EXPECT_EQ(ecef_collection.header.stamp, llhCollection.header.stamp);
  EXPECT_EQ("wgs84", llhCollection.header.frame_id);
  EXPECT_EQ(ecef_collection.colors, llhCollection.colors);
  ASSERT_EQ(2u, llhCollection.polygons.size());
  ASSERT_EQ(2u, llhCollection.polygons[0].points.size());
  ASSERT_EQ(2u, llhCollection.polygons[1].points.size());
  EXPECT_NEAR(kLlh1.latitude, llhCollection.polygons[0].points[0].latitude, kLlhTol);
  EXPECT_NEAR(kLlh1.longitude, llhCollection.polygons[0].points[0].longitude, kLlhTol);
  EXPECT_NEAR(kLlh1.altitude, llhCollection.polygons[0].points[0].altitude, kAltTol);
  EXPECT_NEAR(kLlh2.latitude, llhCollection.polygons[0].points[1].latitude, kLlhTol);
  EXPECT_NEAR(kLlh2.longitude, llhCollection.polygons[0].points[1].longitude, kLlhTol);
  EXPECT_NEAR(kLlh2.altitude, llhCollection.polygons[0].points[1].altitude, kAltTol);
  EXPECT_NEAR(kLlh2.latitude, llhCollection.polygons[1].points[0].latitude, kLlhTol);
  EXPECT_NEAR(kLlh2.longitude, llhCollection.polygons[1].points[0].longitude, kLlhTol);
  EXPECT_NEAR(kLlh2.altitude, llhCollection.polygons[1].points[0].altitude, kAltTol);
  EXPECT_NEAR(kLlh3.latitude, llhCollection.polygons[1].points[1].latitude, kLlhTol);
  EXPECT_NEAR(kLlh3.longitude, llhCollection.polygons[1].points[1].longitude, kLlhTol);
  EXPECT_NEAR(kLlh3.altitude, llhCollection.polygons[1].points[1].altitude, kAltTol);

  EXPECT_EQ(ecef_collection.header.stamp, llhInstanceCollection.header.stamp);
  EXPECT_EQ("wgs84", llhInstanceCollection.header.frame_id);
  EXPECT_EQ(ecef_collection.colors, llhInstanceCollection.colors);
  ASSERT_EQ(2u, llhInstanceCollection.polygons.size());
  ASSERT_EQ(2u, llhInstanceCollection.polygons[0].points.size());
  ASSERT_EQ(2u, llhInstanceCollection.polygons[1].points.size());
  EXPECT_NEAR(kLlh1.latitude, llhInstanceCollection.polygons[0].points[0].latitude, kLlhTol);
  EXPECT_NEAR(kLlh1.longitude, llhInstanceCollection.polygons[0].points[0].longitude, kLlhTol);
  EXPECT_NEAR(kLlh1.altitude, llhInstanceCollection.polygons[0].points[0].altitude, kAltTol);
  EXPECT_NEAR(kLlh2.latitude, llhInstanceCollection.polygons[0].points[1].latitude, kLlhTol);
  EXPECT_NEAR(kLlh2.longitude, llhInstanceCollection.polygons[0].points[1].longitude, kLlhTol);
  EXPECT_NEAR(kLlh2.altitude, llhInstanceCollection.polygons[0].points[1].altitude, kAltTol);
  EXPECT_NEAR(kLlh2.latitude, llhInstanceCollection.polygons[1].points[0].latitude, kLlhTol);
  EXPECT_NEAR(kLlh2.longitude, llhInstanceCollection.polygons[1].points[0].longitude, kLlhTol);
  EXPECT_NEAR(kLlh2.altitude, llhInstanceCollection.polygons[1].points[0].altitude, kAltTol);
  EXPECT_NEAR(kLlh3.latitude, llhInstanceCollection.polygons[1].points[1].latitude, kLlhTol);
  EXPECT_NEAR(kLlh3.longitude, llhInstanceCollection.polygons[1].points[1].longitude, kLlhTol);
  EXPECT_NEAR(kLlh3.altitude, llhInstanceCollection.polygons[1].points[1].altitude, kAltTol);

  EXPECT_EQ(ecef_collection.header.stamp, llhStampedCollection.header.stamp);
  EXPECT_EQ("wgs84", llhStampedCollection.header.frame_id);
  EXPECT_EQ(ecef_collection.colors, llhStampedCollection.colors);
  ASSERT_EQ(2u, llhStampedCollection.polygons.size());
  ASSERT_EQ(2u, llhStampedCollection.polygons[0].points.size());
  ASSERT_EQ(2u, llhStampedCollection.polygons[1].points.size());
  EXPECT_NEAR(kLlh1.latitude, llhStampedCollection.polygons[0].points[0].latitude, kLlhTol);
  EXPECT_NEAR(kLlh1.longitude, llhStampedCollection.polygons[0].points[0].longitude, kLlhTol);
  EXPECT_NEAR(kLlh1.altitude, llhStampedCollection.polygons[0].points[0].altitude, kAltTol);
  EXPECT_NEAR(kLlh2.latitude, llhStampedCollection.polygons[0].points[1].latitude, kLlhTol);
  EXPECT_NEAR(kLlh2.longitude, llhStampedCollection.polygons[0].points[1].longitude, kLlhTol);
  EXPECT_NEAR(kLlh2.altitude, llhStampedCollection.polygons[0].points[1].altitude, kAltTol);
  EXPECT_NEAR(kLlh2.latitude, llhStampedCollection.polygons[1].points[0].latitude, kLlhTol);
  EXPECT_NEAR(kLlh2.longitude, llhStampedCollection.polygons[1].points[0].longitude, kLlhTol);
  EXPECT_NEAR(kLlh2.altitude, llhStampedCollection.polygons[1].points[0].altitude, kAltTol);
  EXPECT_NEAR(kLlh3.latitude, llhStampedCollection.polygons[1].points[1].latitude, kLlhTol);
  EXPECT_NEAR(kLlh3.longitude, llhStampedCollection.polygons[1].points[1].longitude, kLlhTol);
  EXPECT_NEAR(kLlh3.altitude, llhStampedCollection.polygons[1].points[1].altitude, kAltTol);

  EXPECT_EQ(ecef_collection.header.stamp, llhInstanceStampedCollection.header.stamp);
  EXPECT_EQ("wgs84", llhInstanceStampedCollection.header.frame_id);
  EXPECT_EQ(ecef_collection.colors, llhInstanceStampedCollection.colors);
  ASSERT_EQ(2u, llhInstanceStampedCollection.polygons.size());
  ASSERT_EQ(2u, llhInstanceStampedCollection.polygons[0].points.size());
  ASSERT_EQ(2u, llhInstanceStampedCollection.polygons[1].points.size());
  EXPECT_NEAR(kLlh1.latitude, llhInstanceStampedCollection.polygons[0].points[0].latitude, kLlhTol);
  EXPECT_NEAR(kLlh1.longitude, llhInstanceStampedCollection.polygons[0].points[0].longitude, kLlhTol);
  EXPECT_NEAR(kLlh1.altitude, llhInstanceStampedCollection.polygons[0].points[0].altitude, kAltTol);
  EXPECT_NEAR(kLlh2.latitude, llhInstanceStampedCollection.polygons[0].points[1].latitude, kLlhTol);
  EXPECT_NEAR(kLlh2.longitude, llhInstanceStampedCollection.polygons[0].points[1].longitude, kLlhTol);
  EXPECT_NEAR(kLlh2.altitude, llhInstanceStampedCollection.polygons[0].points[1].altitude, kAltTol);
  EXPECT_NEAR(kLlh2.latitude, llhInstanceStampedCollection.polygons[1].points[0].latitude, kLlhTol);
  EXPECT_NEAR(kLlh2.longitude, llhInstanceStampedCollection.polygons[1].points[0].longitude, kLlhTol);
  EXPECT_NEAR(kLlh2.altitude, llhInstanceStampedCollection.polygons[1].points[0].altitude, kAltTol);
  EXPECT_NEAR(kLlh3.latitude, llhInstanceStampedCollection.polygons[1].points[1].latitude, kLlhTol);
  EXPECT_NEAR(kLlh3.longitude, llhInstanceStampedCollection.polygons[1].points[1].longitude, kLlhTol);
  EXPECT_NEAR(kLlh3.altitude, llhInstanceStampedCollection.polygons[1].points[1].altitude, kAltTol);
}

TEST(ConvertECEF, GeoPolygon2DCollectionToECEF) {  // NOLINT
  // the colors of the two polygons differ so that a possible mix-up of the polygons and their colors is detected
  const auto firstColor = createColor(0.0, 1.0, 0.0);
  const auto secondColor = createColor(0.0, 0.0, 1.0);

  auto llh_collection = cras_geo_msgs::msg::GeoPolygon2DCollection();
  llh_collection.header.stamp.sec = 11;
  llh_collection.header.stamp.nanosec = 12;
  llh_collection.header.frame_id = "wgs84";
  llh_collection.polygons = {createGeoPolygon({kLlh1, kLlh2}), createGeoPolygon({kLlh2, kLlh3})};
  llh_collection.colors = {firstColor, secondColor};

  const auto ecef_collection =
    cras_geo_tools::llhtoEcef<cras_geo_msgs::msg::GeoPolygon2DCollection,
        cras_geo_msgs::msg::Polygon64Collection>(llh_collection);
  EXPECT_EQ(llh_collection.header.stamp, ecef_collection.header.stamp);
  EXPECT_EQ("ecef", ecef_collection.header.frame_id);
  EXPECT_EQ(llh_collection.colors, ecef_collection.colors);
  ASSERT_EQ(2u, ecef_collection.polygons.size());
  ASSERT_EQ(2u, ecef_collection.polygons[0].points.size());
  const auto expectedEcef1 =
    cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(kLlh1);
  EXPECT_NEAR(expectedEcef1.x, ecef_collection.polygons[0].points[0].x, kEcefTol);
  EXPECT_NEAR(expectedEcef1.y, ecef_collection.polygons[0].points[0].y, kEcefTol);
  EXPECT_NEAR(expectedEcef1.z, ecef_collection.polygons[0].points[0].z, kEcefTol);
  const auto expectedEcef3 =
    cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(kLlh3);
  EXPECT_NEAR(expectedEcef3.x, ecef_collection.polygons[1].points[1].x, kEcefTol);
  EXPECT_NEAR(expectedEcef3.y, ecef_collection.polygons[1].points[1].y, kEcefTol);
  EXPECT_NEAR(expectedEcef3.z, ecef_collection.polygons[1].points[1].z, kEcefTol);

  const auto ecef_stamped_collection =
    cras_geo_tools::llhtoEcef<cras_geo_msgs::msg::GeoPolygon2DCollection,
        cras_geo_msgs::msg::Polygon64StampedCollection>(llh_collection);
  EXPECT_EQ(llh_collection.header.stamp, ecef_stamped_collection.header.stamp);
  EXPECT_EQ("ecef", ecef_stamped_collection.header.frame_id);
  EXPECT_EQ(llh_collection.colors, ecef_stamped_collection.colors);
  ASSERT_EQ(2u, ecef_stamped_collection.polygons.size());
  // each polygon gets the header of the collection
  EXPECT_EQ(ecef_stamped_collection.header.stamp, ecef_stamped_collection.polygons[0].header.stamp);
  EXPECT_EQ("ecef", ecef_stamped_collection.polygons[0].header.frame_id);
  EXPECT_EQ(ecef_stamped_collection.header.stamp, ecef_stamped_collection.polygons[1].header.stamp);
  EXPECT_EQ("ecef", ecef_stamped_collection.polygons[1].header.frame_id);
  ASSERT_EQ(2u, ecef_stamped_collection.polygons[1].polygon.points.size());
  const auto expectedEcef2 =
    cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(kLlh2);
  EXPECT_NEAR(expectedEcef2.x, ecef_stamped_collection.polygons[1].polygon.points[0].x, kEcefTol);
  EXPECT_NEAR(expectedEcef2.y, ecef_stamped_collection.polygons[1].polygon.points[0].y, kEcefTol);
  EXPECT_NEAR(expectedEcef2.z, ecef_stamped_collection.polygons[1].polygon.points[0].z, kEcefTol);
}

TEST(ConvertECEF, GeoPolygon2DCollectionToECEFInstances) {  // NOLINT
  // the colors of the two polygons differ so that a possible mix-up of the polygons and their colors is detected
  const auto firstColor = createColor(1.0, 1.0, 0.0);
  const auto secondColor = createColor(0.0, 1.0, 1.0);

  auto llh_collection = cras_geo_msgs::msg::GeoPolygon2DCollection();
  llh_collection.header.stamp.sec = 13;
  llh_collection.header.stamp.nanosec = 14;
  llh_collection.header.frame_id = "wgs84";
  llh_collection.polygons = {createGeoPolygon({kLlh1, kLlh2}), createGeoPolygon({kLlh2, kLlh3})};
  llh_collection.colors = {firstColor, secondColor};

  const auto expectedEcef1 =
    cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(kLlh1);
  const auto expectedEcef3 =
    cras_geo_tools::llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(kLlh3);

  const auto ecef_instances =
    cras_geo_tools::llhtoEcef<cras_geo_msgs::msg::GeoPolygon2DCollection,
        cras_geo_msgs::msg::Polygon64InstanceCollection>(llh_collection);
  EXPECT_EQ(llh_collection.header.stamp, ecef_instances.header.stamp);
  EXPECT_EQ("ecef", ecef_instances.header.frame_id);
  EXPECT_EQ(llh_collection.colors, ecef_instances.colors);
  ASSERT_EQ(2u, ecef_instances.polygons.size());
  // the source type has no instance IDs, so the instances are numbered in the order of the polygons
  EXPECT_EQ(0, ecef_instances.polygons[0].id);
  EXPECT_EQ(1, ecef_instances.polygons[1].id);

  ASSERT_EQ(2u, ecef_instances.polygons[0].polygon.points.size());
  EXPECT_NEAR(expectedEcef1.x, ecef_instances.polygons[0].polygon.points[0].x, kEcefTol);
  EXPECT_NEAR(expectedEcef1.y, ecef_instances.polygons[0].polygon.points[0].y, kEcefTol);
  EXPECT_NEAR(expectedEcef1.z, ecef_instances.polygons[0].polygon.points[0].z, kEcefTol);
  ASSERT_EQ(2u, ecef_instances.polygons[1].polygon.points.size());
  EXPECT_NEAR(expectedEcef3.x, ecef_instances.polygons[1].polygon.points[1].x, kEcefTol);
  EXPECT_NEAR(expectedEcef3.y, ecef_instances.polygons[1].polygon.points[1].y, kEcefTol);
  EXPECT_NEAR(expectedEcef3.z, ecef_instances.polygons[1].polygon.points[1].z, kEcefTol);

  const auto ecef_stamped_instances =
    cras_geo_tools::llhtoEcef<cras_geo_msgs::msg::GeoPolygon2DCollection,
        cras_geo_msgs::msg::Polygon64InstanceStampedCollection>(llh_collection);
  EXPECT_EQ(llh_collection.header.stamp, ecef_stamped_instances.header.stamp);
  EXPECT_EQ("ecef", ecef_stamped_instances.header.frame_id);
  EXPECT_EQ(llh_collection.colors, ecef_stamped_instances.colors);
  ASSERT_EQ(2u, ecef_stamped_instances.polygons.size());
  // each polygon gets the header of the collection and an ID numbered in the order of the polygons
  EXPECT_EQ(ecef_stamped_instances.header, ecef_stamped_instances.polygons[0].header);
  EXPECT_EQ(ecef_stamped_instances.header, ecef_stamped_instances.polygons[1].header);
  EXPECT_EQ(0, ecef_stamped_instances.polygons[0].polygon.id);
  EXPECT_EQ(1, ecef_stamped_instances.polygons[1].polygon.id);

  ASSERT_EQ(2u, ecef_stamped_instances.polygons[0].polygon.polygon.points.size());
  EXPECT_NEAR(expectedEcef1.x, ecef_stamped_instances.polygons[0].polygon.polygon.points[0].x, kEcefTol);
  EXPECT_NEAR(expectedEcef1.y, ecef_stamped_instances.polygons[0].polygon.polygon.points[0].y, kEcefTol);
  EXPECT_NEAR(expectedEcef1.z, ecef_stamped_instances.polygons[0].polygon.polygon.points[0].z, kEcefTol);
  ASSERT_EQ(2u, ecef_stamped_instances.polygons[1].polygon.polygon.points.size());
  EXPECT_NEAR(expectedEcef3.x, ecef_stamped_instances.polygons[1].polygon.polygon.points[1].x, kEcefTol);
  EXPECT_NEAR(expectedEcef3.y, ecef_stamped_instances.polygons[1].polygon.polygon.points[1].y, kEcefTol);
  EXPECT_NEAR(expectedEcef3.z, ecef_stamped_instances.polygons[1].polygon.polygon.points[1].z, kEcefTol);

  // converting back drops the IDs and the per-polygon headers, but keeps the geometry and the colors
  const auto llhBack =
    cras_geo_tools::ecefToLlh<cras_geo_msgs::msg::Polygon64InstanceStampedCollection,
        cras_geo_msgs::msg::GeoPolygon2DCollection>(ecef_stamped_instances);
  EXPECT_EQ(llh_collection.header.stamp, llhBack.header.stamp);
  EXPECT_EQ("wgs84", llhBack.header.frame_id);
  EXPECT_EQ(llh_collection.colors, llhBack.colors);
  ASSERT_EQ(2u, llhBack.polygons.size());
  ASSERT_EQ(2u, llhBack.polygons[0].points.size());
  EXPECT_NEAR(kLlh1.latitude, llhBack.polygons[0].points[0].latitude, kLlhTol);
  EXPECT_NEAR(kLlh1.longitude, llhBack.polygons[0].points[0].longitude, kLlhTol);
  EXPECT_NEAR(kLlh1.altitude, llhBack.polygons[0].points[0].altitude, kAltTol);
  ASSERT_EQ(2u, llhBack.polygons[1].points.size());
  EXPECT_NEAR(kLlh3.latitude, llhBack.polygons[1].points[1].latitude, kLlhTol);
  EXPECT_NEAR(kLlh3.longitude, llhBack.polygons[1].points[1].longitude, kLlhTol);
  EXPECT_NEAR(kLlh3.altitude, llhBack.polygons[1].points[1].altitude, kAltTol);
}

TEST(ConvertECEF, EmptyMessages) {  // NOLINT
  const auto empty_ecef_polygon = cras_geo_msgs::msg::Polygon64();
  const auto empty_ecef_stamped_polygon = cras_geo_msgs::msg::Polygon64Stamped();
  const auto empty_ecef_instance = cras_geo_msgs::msg::Polygon64Instance();
  const auto empty_ecef_instance_stamped = cras_geo_msgs::msg::Polygon64InstanceStamped();
  const auto empty_llh_polygon = cras_geo_msgs::msg::GeoPolygon2D();
  const auto empty_ecef_collection = cras_geo_msgs::msg::Polygon64Collection();
  const auto empty_ecef_stamped_collection = cras_geo_msgs::msg::Polygon64StampedCollection();
  const auto empty_ecef_instance_collection = cras_geo_msgs::msg::Polygon64InstanceCollection();
  const auto empty_ecef_instance_stamped_collection = cras_geo_msgs::msg::Polygon64InstanceStampedCollection();
  const auto empty_llh_collection = cras_geo_msgs::msg::GeoPolygon2DCollection();

  const auto llh_polygon =
    cras_geo_tools::ecefToLlh<cras_geo_msgs::msg::Polygon64, cras_geo_msgs::msg::GeoPolygon2D>(empty_ecef_polygon);
  EXPECT_TRUE(llh_polygon.points.empty());

  const auto ecef_polygon =
    cras_geo_tools::llhtoEcef<cras_geo_msgs::msg::GeoPolygon2D, cras_geo_msgs::msg::Polygon64>(empty_llh_polygon);
  EXPECT_TRUE(ecef_polygon.points.empty());

  // the headerless variants cannot produce any non-empty output, but they must not fail either
  const auto llhStampedPolygon =
    cras_geo_tools::ecefToLlh<cras_geo_msgs::msg::Polygon64Stamped, cras_geo_msgs::msg::GeoPolygon2D>(
      empty_ecef_stamped_polygon);
  EXPECT_TRUE(llhStampedPolygon.points.empty());

  const auto llhInstance =
    cras_geo_tools::ecefToLlh<cras_geo_msgs::msg::Polygon64Instance, cras_geo_msgs::msg::GeoPolygon2D>(
      empty_ecef_instance);
  EXPECT_TRUE(llhInstance.points.empty());

  const auto llhInstanceStamped =
    cras_geo_tools::ecefToLlh<cras_geo_msgs::msg::Polygon64InstanceStamped, cras_geo_msgs::msg::GeoPolygon2D>(
      empty_ecef_instance_stamped);
  EXPECT_TRUE(llhInstanceStamped.points.empty());

  const auto llh_collection =
    cras_geo_tools::ecefToLlh<cras_geo_msgs::msg::Polygon64Collection, cras_geo_msgs::msg::GeoPolygon2DCollection>(
      empty_ecef_collection);
  EXPECT_TRUE(llh_collection.polygons.empty());
  EXPECT_TRUE(llh_collection.colors.empty());
  // even an empty collection is assigned the frame of the target type
  EXPECT_EQ("wgs84", llh_collection.header.frame_id);

  const auto llhStampedCollection =
    cras_geo_tools::ecefToLlh<cras_geo_msgs::msg::Polygon64StampedCollection,
        cras_geo_msgs::msg::GeoPolygon2DCollection>(empty_ecef_stamped_collection);
  EXPECT_TRUE(llhStampedCollection.polygons.empty());
  EXPECT_TRUE(llhStampedCollection.colors.empty());
  EXPECT_EQ("wgs84", llhStampedCollection.header.frame_id);

  const auto llhInstanceCollection =
    cras_geo_tools::ecefToLlh<cras_geo_msgs::msg::Polygon64InstanceCollection,
        cras_geo_msgs::msg::GeoPolygon2DCollection>(empty_ecef_instance_collection);
  EXPECT_TRUE(llhInstanceCollection.polygons.empty());
  EXPECT_TRUE(llhInstanceCollection.colors.empty());
  EXPECT_EQ("wgs84", llhInstanceCollection.header.frame_id);

  const auto llhInstanceStampedCollection =
    cras_geo_tools::ecefToLlh<cras_geo_msgs::msg::Polygon64InstanceStampedCollection,
        cras_geo_msgs::msg::GeoPolygon2DCollection>(empty_ecef_instance_stamped_collection);
  EXPECT_TRUE(llhInstanceStampedCollection.polygons.empty());
  EXPECT_TRUE(llhInstanceStampedCollection.colors.empty());
  EXPECT_EQ("wgs84", llhInstanceStampedCollection.header.frame_id);

  const auto ecef_collection =
    cras_geo_tools::llhtoEcef<cras_geo_msgs::msg::GeoPolygon2DCollection, cras_geo_msgs::msg::Polygon64Collection>(
      empty_llh_collection);
  EXPECT_TRUE(ecef_collection.polygons.empty());
  EXPECT_TRUE(ecef_collection.colors.empty());
  EXPECT_EQ("ecef", ecef_collection.header.frame_id);

  const auto ecef_stamped_collection =
    cras_geo_tools::llhtoEcef<cras_geo_msgs::msg::GeoPolygon2DCollection,
        cras_geo_msgs::msg::Polygon64StampedCollection>(empty_llh_collection);
  EXPECT_TRUE(ecef_stamped_collection.polygons.empty());
  EXPECT_TRUE(ecef_stamped_collection.colors.empty());
  EXPECT_EQ("ecef", ecef_stamped_collection.header.frame_id);

  const auto ecef_instance_collection =
    cras_geo_tools::llhtoEcef<cras_geo_msgs::msg::GeoPolygon2DCollection,
        cras_geo_msgs::msg::Polygon64InstanceCollection>(empty_llh_collection);
  EXPECT_TRUE(ecef_instance_collection.polygons.empty());
  EXPECT_TRUE(ecef_instance_collection.colors.empty());
  EXPECT_EQ("ecef", ecef_instance_collection.header.frame_id);

  const auto ecef_instance_stamped_collection =
    cras_geo_tools::llhtoEcef<cras_geo_msgs::msg::GeoPolygon2DCollection,
        cras_geo_msgs::msg::Polygon64InstanceStampedCollection>(empty_llh_collection);
  EXPECT_TRUE(ecef_instance_stamped_collection.polygons.empty());
  EXPECT_TRUE(ecef_instance_stamped_collection.colors.empty());
  EXPECT_EQ("ecef", ecef_instance_stamped_collection.header.frame_id);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
