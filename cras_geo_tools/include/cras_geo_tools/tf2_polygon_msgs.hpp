// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief tf2 transformations of the polygon messages from the polygon_msgs and cras_geo_msgs packages.
 * \author Martin Pecka
 */

#pragma once

#include <string>

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
#include <geometry_msgs/msg/polygon_instance.hpp>
#include <geometry_msgs/msg/polygon_instance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <polygon_msgs/msg/complex_polygon2_d.hpp>
#include <polygon_msgs/msg/complex_polygon2_d_collection.hpp>
#include <polygon_msgs/msg/complex_polygon2_d_stamped.hpp>
#include <polygon_msgs/msg/point2_d.hpp>
#include <polygon_msgs/msg/polygon2_d.hpp>
#include <polygon_msgs/msg/polygon2_d_collection.hpp>
#include <polygon_msgs/msg/polygon2_d_stamped.hpp>
#include <tf2/convert.hpp>

namespace tf2 {

template<>
void doTransform(
    const polygon_msgs::msg::Point2D& data_in, polygon_msgs::msg::Point2D& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const polygon_msgs::msg::Polygon2D& data_in, polygon_msgs::msg::Polygon2D& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const polygon_msgs::msg::Polygon2DStamped& data_in, polygon_msgs::msg::Polygon2DStamped& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const polygon_msgs::msg::Polygon2DCollection& data_in, polygon_msgs::msg::Polygon2DCollection& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const polygon_msgs::msg::ComplexPolygon2D& data_in, polygon_msgs::msg::ComplexPolygon2D& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const polygon_msgs::msg::ComplexPolygon2DStamped& data_in, polygon_msgs::msg::ComplexPolygon2DStamped& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const polygon_msgs::msg::ComplexPolygon2DCollection& data_in,
    polygon_msgs::msg::ComplexPolygon2DCollection& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const geometry_msgs::msg::PolygonInstance& data_in, geometry_msgs::msg::PolygonInstance& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const geometry_msgs::msg::PolygonInstanceStamped& data_in, geometry_msgs::msg::PolygonInstanceStamped& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const cras_geo_msgs::msg::PolygonCollection& data_in,
    cras_geo_msgs::msg::PolygonCollection& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const cras_geo_msgs::msg::PolygonInstanceCollection& data_in,
    cras_geo_msgs::msg::PolygonInstanceCollection& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const cras_geo_msgs::msg::PolygonInstanceStampedCollection& data_in,
    cras_geo_msgs::msg::PolygonInstanceStampedCollection& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const cras_geo_msgs::msg::PolygonStampedCollection& data_in,
    cras_geo_msgs::msg::PolygonStampedCollection& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const cras_geo_msgs::msg::Polygon64& data_in,
    cras_geo_msgs::msg::Polygon64& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const cras_geo_msgs::msg::Polygon64Instance& data_in,
    cras_geo_msgs::msg::Polygon64Instance& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const cras_geo_msgs::msg::Polygon64InstanceStamped& data_in,
    cras_geo_msgs::msg::Polygon64InstanceStamped& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const cras_geo_msgs::msg::Polygon64Stamped& data_in,
    cras_geo_msgs::msg::Polygon64Stamped& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const cras_geo_msgs::msg::Polygon64Collection& data_in,
    cras_geo_msgs::msg::Polygon64Collection& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const cras_geo_msgs::msg::Polygon64InstanceCollection& data_in,
    cras_geo_msgs::msg::Polygon64InstanceCollection& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const cras_geo_msgs::msg::Polygon64InstanceStampedCollection& data_in,
    cras_geo_msgs::msg::Polygon64InstanceStampedCollection& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
void doTransform(
    const cras_geo_msgs::msg::Polygon64StampedCollection& data_in,
    cras_geo_msgs::msg::Polygon64StampedCollection& data_out,
    const geometry_msgs::msg::TransformStamped& transform);

template<>
tf2::TimePoint getTimestamp(const polygon_msgs::msg::Polygon2DStamped& t);

template<>
tf2::TimePoint getTimestamp(const polygon_msgs::msg::Polygon2DCollection& t);

template<>
tf2::TimePoint getTimestamp(const polygon_msgs::msg::ComplexPolygon2DStamped& t);

template<>
tf2::TimePoint getTimestamp(const polygon_msgs::msg::ComplexPolygon2DCollection& t);

template<>
tf2::TimePoint getTimestamp(const cras_geo_msgs::msg::Polygon64Stamped& t);

template<>
tf2::TimePoint getTimestamp(const cras_geo_msgs::msg::Polygon64InstanceStamped& t);

template<>
tf2::TimePoint getTimestamp(const cras_geo_msgs::msg::Polygon64Collection& t);

template<>
tf2::TimePoint getTimestamp(const cras_geo_msgs::msg::Polygon64InstanceCollection& t);

template<>
tf2::TimePoint getTimestamp(const cras_geo_msgs::msg::Polygon64InstanceStampedCollection& t);

template<>
tf2::TimePoint getTimestamp(const cras_geo_msgs::msg::Polygon64StampedCollection& t);

template<>
tf2::TimePoint getTimestamp(const cras_geo_msgs::msg::PolygonCollection& t);

template<>
tf2::TimePoint getTimestamp(const cras_geo_msgs::msg::PolygonInstanceCollection& t);

template<>
tf2::TimePoint getTimestamp(const cras_geo_msgs::msg::PolygonInstanceStampedCollection& t);

template<>
tf2::TimePoint getTimestamp(const cras_geo_msgs::msg::PolygonStampedCollection& t);

template<>
std::string getFrameId(const polygon_msgs::msg::Polygon2DStamped& t);

template<>
std::string getFrameId(const polygon_msgs::msg::Polygon2DCollection& t);

template<>
std::string getFrameId(const polygon_msgs::msg::ComplexPolygon2DStamped& t);

template<>
std::string getFrameId(const polygon_msgs::msg::ComplexPolygon2DCollection& t);

template<>
std::string getFrameId(const cras_geo_msgs::msg::Polygon64Stamped& t);

template<>
std::string getFrameId(const cras_geo_msgs::msg::Polygon64InstanceStamped& t);

template<>
std::string getFrameId(const cras_geo_msgs::msg::Polygon64Collection& t);

template<>
std::string getFrameId(const cras_geo_msgs::msg::Polygon64InstanceCollection& t);

template<>
std::string getFrameId(const cras_geo_msgs::msg::Polygon64InstanceStampedCollection& t);

template<>
std::string getFrameId(const cras_geo_msgs::msg::Polygon64StampedCollection& t);

template<>
std::string getFrameId(const cras_geo_msgs::msg::PolygonCollection& t);

template<>
std::string getFrameId(const cras_geo_msgs::msg::PolygonInstanceCollection& t);

template<>
std::string getFrameId(const cras_geo_msgs::msg::PolygonInstanceStampedCollection& t);

template<>
std::string getFrameId(const cras_geo_msgs::msg::PolygonStampedCollection& t);

}  // namespace tf2
