// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Conversions between messages with ECEF coordinates and messages with spherical (geodetic) coordinates.
 * \author Martin Pecka
 *
 * geometry_msgs/Polygon* types are not supported because they cannot hold ECEF coordinates with useful precision.
 */

#pragma once

#include <cras_geo_msgs/msg/geo_polygon2_d.hpp>
#include <cras_geo_msgs/msg/geo_polygon2_d_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_instance_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_instance_stamped.hpp>
#include <cras_geo_msgs/msg/polygon64_instance_stamped_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_stamped.hpp>
#include <cras_geo_msgs/msg/polygon64_stamped_collection.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <geographic_msgs/msg/geo_pose.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace cras_geo_tools {

/**
 * \brief Convert the given message with Cartesian coordinates in the ECEF frame to a message with spherical coordinates
 *        (latitude, longitude, altitude) in the WGS84 ellipsoid frame.
 * \tparam Cartesian The type of the input message.
 * \tparam Spherical The type of the output message.
 * \param[in] ecef The message in the ECEF frame.
 * \return The converted message. Latitudes and longitudes are in degrees, altitudes in metres above the WGS84
 *         ellipsoid. If the message has a header, its stamp is copied and its frame_id is replaced by "wgs84".
 */
template<typename Cartesian, typename Spherical>
Spherical ecefToLlh(const Cartesian& ecef);

/**
 * \brief Convert the given message with spherical coordinates (latitude, longitude, altitude) in the WGS84 ellipsoid
 *        frame to a message with Cartesian coordinates in the ECEF frame.
 * \tparam Spherical The type of the input message.
 * \tparam Cartesian The type of the output message.
 * \param[in] llh The message with spherical coordinates. Latitude and longitude are expected in degrees, altitude
 *                in metres above the WGS84 ellipsoid.
 * \return The converted message. If the message has a header, its stamp is copied and its frame_id is replaced by ecef.
 */
template<typename Spherical, typename Cartesian>
Cartesian llhtoEcef(const Spherical& llh);

template<>
geographic_msgs::msg::GeoPoint ecefToLlh(const geometry_msgs::msg::Point& ecef);

template<>
geographic_msgs::msg::GeoPose ecefToLlh(const geometry_msgs::msg::Pose& ecef);

template<>
geographic_msgs::msg::GeoPointStamped ecefToLlh(const geometry_msgs::msg::PointStamped& ecef);

template<>
geographic_msgs::msg::GeoPoseStamped ecefToLlh(const geometry_msgs::msg::PoseStamped& ecef);

template<>
cras_geo_msgs::msg::GeoPolygon2D ecefToLlh(const cras_geo_msgs::msg::Polygon64& ecef);

template<>
cras_geo_msgs::msg::GeoPolygon2D ecefToLlh(const cras_geo_msgs::msg::Polygon64Stamped& ecef);

template<>
cras_geo_msgs::msg::GeoPolygon2D ecefToLlh(const cras_geo_msgs::msg::Polygon64Instance& ecef);

template<>
cras_geo_msgs::msg::GeoPolygon2D ecefToLlh(const cras_geo_msgs::msg::Polygon64InstanceStamped& ecef);

template<>
cras_geo_msgs::msg::GeoPolygon2DCollection ecefToLlh(const cras_geo_msgs::msg::Polygon64Collection& ecef);

template<>
cras_geo_msgs::msg::GeoPolygon2DCollection ecefToLlh(const cras_geo_msgs::msg::Polygon64InstanceCollection& ecef);

template<>
cras_geo_msgs::msg::GeoPolygon2DCollection ecefToLlh(
    const cras_geo_msgs::msg::Polygon64InstanceStampedCollection& ecef);

template<>
cras_geo_msgs::msg::GeoPolygon2DCollection ecefToLlh(const cras_geo_msgs::msg::Polygon64StampedCollection& ecef);

template<>
geometry_msgs::msg::Point llhtoEcef(const geographic_msgs::msg::GeoPoint& llh);

template<>
geometry_msgs::msg::Pose llhtoEcef(const geographic_msgs::msg::GeoPose& llh);

template<>
geometry_msgs::msg::PointStamped llhtoEcef(const geographic_msgs::msg::GeoPointStamped& llh);

template<>
geometry_msgs::msg::PoseStamped llhtoEcef(const geographic_msgs::msg::GeoPoseStamped& llh);

template<>
cras_geo_msgs::msg::Polygon64 llhtoEcef(const cras_geo_msgs::msg::GeoPolygon2D& llh);

template<>
cras_geo_msgs::msg::Polygon64Collection llhtoEcef(const cras_geo_msgs::msg::GeoPolygon2DCollection& llh);

template<>
cras_geo_msgs::msg::Polygon64StampedCollection llhtoEcef(const cras_geo_msgs::msg::GeoPolygon2DCollection& llh);

template<>
cras_geo_msgs::msg::Polygon64InstanceCollection llhtoEcef(const cras_geo_msgs::msg::GeoPolygon2DCollection& llh);

template<>
cras_geo_msgs::msg::Polygon64InstanceStampedCollection llhtoEcef(const cras_geo_msgs::msg::GeoPolygon2DCollection& llh);

}  // namespace cras_geo_tools
