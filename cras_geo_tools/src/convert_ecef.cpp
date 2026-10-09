// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Conversions between messages with ECEF coordinates and messages with spherical (geodetic) coordinates.
 * \author Martin Pecka
 *
 * geometry_msgs/Polygon* types are not supported because they cannot hold ECEF coordinates with useful precision.
 */

#include <cras_geo_msgs/msg/geo_polygon2_d.hpp>
#include <cras_geo_msgs/msg/geo_polygon2_d_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_instance_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_instance_stamped.hpp>
#include <cras_geo_msgs/msg/polygon64_instance_stamped_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_stamped.hpp>
#include <cras_geo_msgs/msg/polygon64_stamped_collection.hpp>
#include <cras_geo_tools/convert_ecef.hpp>
#include <geodesy/ecef.h>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <geographic_msgs/msg/geo_pose.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace cras_geo_tools {

template<>
geographic_msgs::msg::GeoPoint ecefToLlh(const geometry_msgs::msg::Point& ecef) {
  return geodesy::toMsg(geodesy::ECEFPoint(ecef));
}

template<>
geographic_msgs::msg::GeoPose ecefToLlh(const geometry_msgs::msg::Pose& ecef) {
  return geodesy::toMsg(geodesy::ECEFPose(geodesy::ECEFPoint(ecef.position), ecef.orientation));
}

template<>
geographic_msgs::msg::GeoPointStamped ecefToLlh(const geometry_msgs::msg::PointStamped& ecef) {
  geographic_msgs::msg::GeoPointStamped out;
  out.header.stamp = ecef.header.stamp;
  out.header.frame_id = "wgs84";
  out.position = ecefToLlh<geometry_msgs::msg::Point, geographic_msgs::msg::GeoPoint>(ecef.point);
  return out;
}

template<>
geographic_msgs::msg::GeoPoseStamped ecefToLlh(const geometry_msgs::msg::PoseStamped& ecef) {
  geographic_msgs::msg::GeoPoseStamped out;
  out.header.stamp = ecef.header.stamp;
  out.header.frame_id = "wgs84";
  out.pose = ecefToLlh<geometry_msgs::msg::Pose, geographic_msgs::msg::GeoPose>(ecef.pose);
  return out;
}

template<>
cras_geo_msgs::msg::GeoPolygon2D ecefToLlh(const cras_geo_msgs::msg::Polygon64& ecef) {
  cras_geo_msgs::msg::GeoPolygon2D out;
  out.points.reserve(ecef.points.size());
  for (const auto& pt : ecef.points) {
    out.points.emplace_back() = ecefToLlh<geometry_msgs::msg::Point, geographic_msgs::msg::GeoPoint>(pt);
  }
  return out;
}

template<>
cras_geo_msgs::msg::GeoPolygon2D ecefToLlh(const cras_geo_msgs::msg::Polygon64Stamped& ecef) {
  return ecefToLlh<cras_geo_msgs::msg::Polygon64, cras_geo_msgs::msg::GeoPolygon2D>(ecef.polygon);
}

template<>
cras_geo_msgs::msg::GeoPolygon2D ecefToLlh(const cras_geo_msgs::msg::Polygon64Instance& ecef) {
  cras_geo_msgs::msg::GeoPolygon2D out;
  out.points.reserve(ecef.polygon.points.size());
  for (const auto& pt : ecef.polygon.points) {
    out.points.emplace_back() = ecefToLlh<geometry_msgs::msg::Point, geographic_msgs::msg::GeoPoint>(pt);
  }
  return out;
}

template<>
cras_geo_msgs::msg::GeoPolygon2D ecefToLlh(const cras_geo_msgs::msg::Polygon64InstanceStamped& ecef) {
  return ecefToLlh<cras_geo_msgs::msg::Polygon64Instance, cras_geo_msgs::msg::GeoPolygon2D>(ecef.polygon);
}

template<>
cras_geo_msgs::msg::GeoPolygon2DCollection ecefToLlh(const cras_geo_msgs::msg::Polygon64Collection& ecef) {
  cras_geo_msgs::msg::GeoPolygon2DCollection out;
  out.header.stamp = ecef.header.stamp;
  out.header.frame_id = "wgs84";
  out.colors = ecef.colors;
  out.polygons.reserve(ecef.polygons.size());
  for (const auto& polygon : ecef.polygons) {
    out.polygons.emplace_back() = ecefToLlh<cras_geo_msgs::msg::Polygon64, cras_geo_msgs::msg::GeoPolygon2D>(polygon);
  }
  return out;
}

template<>
cras_geo_msgs::msg::GeoPolygon2DCollection ecefToLlh(const cras_geo_msgs::msg::Polygon64InstanceCollection& ecef) {
  cras_geo_msgs::msg::GeoPolygon2DCollection out;
  out.header.stamp = ecef.header.stamp;
  out.header.frame_id = "wgs84";
  out.colors = ecef.colors;
  out.polygons.reserve(ecef.polygons.size());
  for (const auto& polygon : ecef.polygons) {
    out.polygons.emplace_back() =
      ecefToLlh<cras_geo_msgs::msg::Polygon64Instance, cras_geo_msgs::msg::GeoPolygon2D>(polygon);
  }
  return out;
}

template<>
cras_geo_msgs::msg::GeoPolygon2DCollection ecefToLlh(
    const cras_geo_msgs::msg::Polygon64InstanceStampedCollection& ecef) {
  cras_geo_msgs::msg::GeoPolygon2DCollection out;
  out.header.stamp = ecef.header.stamp;
  out.header.frame_id = "wgs84";
  out.colors = ecef.colors;
  out.polygons.reserve(ecef.polygons.size());
  for (const auto& polygon : ecef.polygons) {
    out.polygons.emplace_back() =
      ecefToLlh<cras_geo_msgs::msg::Polygon64InstanceStamped, cras_geo_msgs::msg::GeoPolygon2D>(polygon);
  }
  return out;
}

template<>
cras_geo_msgs::msg::GeoPolygon2DCollection ecefToLlh(const cras_geo_msgs::msg::Polygon64StampedCollection& ecef) {
  cras_geo_msgs::msg::GeoPolygon2DCollection out;
  out.header.stamp = ecef.header.stamp;
  out.header.frame_id = "wgs84";
  out.colors = ecef.colors;
  out.polygons.reserve(ecef.polygons.size());
  for (const auto& polygon : ecef.polygons) {
    out.polygons.emplace_back() =
      ecefToLlh<cras_geo_msgs::msg::Polygon64Stamped, cras_geo_msgs::msg::GeoPolygon2D>(polygon);
  }
  return out;
}

template<>
geometry_msgs::msg::Point llhtoEcef(const geographic_msgs::msg::GeoPoint& llh) {
  return geodesy::toGeometry(geodesy::ECEFPoint(llh));
}

template<>
geometry_msgs::msg::Pose llhtoEcef(const geographic_msgs::msg::GeoPose& llh) {
  return geodesy::toGeometry(geodesy::ECEFPose(llh));
}

template<>
geometry_msgs::msg::PointStamped llhtoEcef(const geographic_msgs::msg::GeoPointStamped& llh) {
  geometry_msgs::msg::PointStamped out;
  out.header.stamp = llh.header.stamp;
  out.header.frame_id = "ecef";
  out.point = llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(llh.position);
  return out;
}

template<>
geometry_msgs::msg::PoseStamped llhtoEcef(const geographic_msgs::msg::GeoPoseStamped& llh) {
  geometry_msgs::msg::PoseStamped out;
  out.header.stamp = llh.header.stamp;
  out.header.frame_id = "ecef";
  out.pose = llhtoEcef<geographic_msgs::msg::GeoPose, geometry_msgs::msg::Pose>(llh.pose);
  return out;
}

template<>
cras_geo_msgs::msg::Polygon64 llhtoEcef(const cras_geo_msgs::msg::GeoPolygon2D& llh) {
  cras_geo_msgs::msg::Polygon64 out;
  out.points.reserve(llh.points.size());
  for (const auto& point : llh.points) {
    out.points.emplace_back() = llhtoEcef<geographic_msgs::msg::GeoPoint, geometry_msgs::msg::Point>(point);
  }
  return out;
}

template<>
cras_geo_msgs::msg::Polygon64Collection llhtoEcef(const cras_geo_msgs::msg::GeoPolygon2DCollection& llh) {
  cras_geo_msgs::msg::Polygon64Collection out;
  out.header.stamp = llh.header.stamp;
  out.header.frame_id = "ecef";
  out.colors = llh.colors;
  out.polygons.reserve(llh.polygons.size());
  for (const auto& geo_polygon : llh.polygons) {
    out.polygons.emplace_back() =
      llhtoEcef<cras_geo_msgs::msg::GeoPolygon2D, cras_geo_msgs::msg::Polygon64>(geo_polygon);
  }
  return out;
}

template<>
cras_geo_msgs::msg::Polygon64StampedCollection llhtoEcef(const cras_geo_msgs::msg::GeoPolygon2DCollection& llh) {
  cras_geo_msgs::msg::Polygon64StampedCollection out;
  out.header.stamp = llh.header.stamp;
  out.header.frame_id = "ecef";
  out.colors = llh.colors;
  out.polygons.reserve(llh.polygons.size());
  for (const auto& geo_polygon : llh.polygons) {
    auto& polygon = out.polygons.emplace_back();
    polygon.header = out.header;
    polygon.polygon = llhtoEcef<cras_geo_msgs::msg::GeoPolygon2D, cras_geo_msgs::msg::Polygon64>(geo_polygon);
  }
  return out;
}

template<>
cras_geo_msgs::msg::Polygon64InstanceCollection llhtoEcef(const cras_geo_msgs::msg::GeoPolygon2DCollection& llh) {
  cras_geo_msgs::msg::Polygon64InstanceCollection out;
  out.header.stamp = llh.header.stamp;
  out.header.frame_id = "ecef";
  out.colors = llh.colors;
  out.polygons.reserve(llh.polygons.size());
  size_t i = 0;
  for (const auto& geo_polygon : llh.polygons) {
    auto& polygon_ecef = out.polygons.emplace_back();
    polygon_ecef.polygon = llhtoEcef<cras_geo_msgs::msg::GeoPolygon2D, cras_geo_msgs::msg::Polygon64>(geo_polygon);
    polygon_ecef.id = static_cast<int64_t>(i);
    i++;
  }
  return out;
}

template<>
cras_geo_msgs::msg::Polygon64InstanceStampedCollection llhtoEcef(
    const cras_geo_msgs::msg::GeoPolygon2DCollection& llh) {
  cras_geo_msgs::msg::Polygon64InstanceStampedCollection out;
  out.header.stamp = llh.header.stamp;
  out.header.frame_id = "ecef";
  out.colors = llh.colors;
  out.polygons.reserve(llh.polygons.size());
  size_t i = 0;
  for (const auto& geo_polygon : llh.polygons) {
    auto& polygon = out.polygons.emplace_back();
    polygon.header = out.header;
    polygon.polygon.polygon = llhtoEcef<cras_geo_msgs::msg::GeoPolygon2D, cras_geo_msgs::msg::Polygon64>(geo_polygon);
    polygon.polygon.id = static_cast<int64_t>(i);
    i++;
  }
  return out;
}

}  // namespace cras_geo_tools
