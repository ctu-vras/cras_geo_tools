// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief tf2 transformations of the polygon messages from the polygon_msgs and cras_geo_msgs packages.
 * \author Martin Pecka
 */

#include <cras_geo_msgs/msg/polygon_collection.hpp>
#include <cras_geo_msgs/msg/polygon_instance_collection.hpp>
#include <cras_geo_msgs/msg/polygon_instance_stamped_collection.hpp>
#include <cras_geo_msgs/msg/polygon_stamped_collection.hpp>
#include <cras_geo_tools/tf2_polygon_msgs.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <polygon_msgs/msg/complex_polygon2_d.hpp>
#include <polygon_msgs/msg/complex_polygon2_d_collection.hpp>
#include <polygon_msgs/msg/complex_polygon2_d_stamped.hpp>
#include <polygon_msgs/msg/point2_d.hpp>
#include <polygon_msgs/msg/polygon2_d.hpp>
#include <polygon_msgs/msg/polygon2_d_collection.hpp>
#include <polygon_msgs/msg/polygon2_d_stamped.hpp>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tf2 {

template<>
void doTransform(
    const polygon_msgs::msg::Point2D& data_in, polygon_msgs::msg::Point2D& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  geometry_msgs::msg::Point point_out;
  geometry_msgs::msg::Point point_in;
  point_in.x = data_in.x;
  point_in.y = data_in.y;
  point_in.z = 0.0;
  doTransform(point_in, point_out, transform);
  data_out.x = point_out.x;
  data_out.y = point_out.y;
}

template<>
void doTransform(
    const polygon_msgs::msg::Polygon2D& data_in, polygon_msgs::msg::Polygon2D& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  data_out.points.reserve(data_in.points.size());
  for (auto& points : data_in.points) {
    doTransform(points, data_out.points.emplace_back(), transform);
  }
}

template<>
void doTransform(
    const polygon_msgs::msg::Polygon2DStamped& data_in, polygon_msgs::msg::Polygon2DStamped& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  data_out.header.stamp = transform.header.stamp;
  data_out.header.frame_id = transform.child_frame_id;
  doTransform(data_in.polygon, data_out.polygon, transform);
}

template<>
void doTransform(
    const polygon_msgs::msg::Polygon2DCollection& data_in, polygon_msgs::msg::Polygon2DCollection& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  data_out.header.stamp = transform.header.stamp;
  data_out.header.frame_id = transform.child_frame_id;
  data_out.colors = data_in.colors;
  data_out.polygons.reserve(data_in.polygons.size());
  for (auto& polygon : data_in.polygons) {
    doTransform(polygon, data_out.polygons.emplace_back(), transform);
  }
}

template<>
void doTransform(
    const polygon_msgs::msg::ComplexPolygon2D& data_in, polygon_msgs::msg::ComplexPolygon2D& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  data_out.inner.reserve(data_in.inner.size());
  for (auto& inner : data_in.inner) {
    doTransform(inner, data_out.inner.emplace_back(), transform);
  }
  doTransform(data_in.outer, data_out.outer, transform);
}

template<>
void doTransform(
    const polygon_msgs::msg::ComplexPolygon2DStamped& data_in, polygon_msgs::msg::ComplexPolygon2DStamped& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  data_out.header.stamp = transform.header.stamp;
  data_out.header.frame_id = transform.child_frame_id;
  doTransform(data_in.polygon, data_out.polygon, transform);
}

template<>
void doTransform(
    const polygon_msgs::msg::ComplexPolygon2DCollection& data_in,
    polygon_msgs::msg::ComplexPolygon2DCollection& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  data_out.header.stamp = transform.header.stamp;
  data_out.header.frame_id = transform.child_frame_id;
  data_out.colors = data_in.colors;
  data_out.polygons.reserve(data_in.polygons.size());
  for (auto& polygon : data_in.polygons) {
    doTransform(polygon, data_out.polygons.emplace_back(), transform);
  }
}

template<>
void doTransform(
    const geometry_msgs::msg::PolygonInstance& data_in, geometry_msgs::msg::PolygonInstance& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  data_out.id = data_in.id;
  doTransform(data_in.polygon, data_out.polygon, transform);
}

template<>
void doTransform(
    const geometry_msgs::msg::PolygonInstanceStamped& data_in, geometry_msgs::msg::PolygonInstanceStamped& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  data_out.header.stamp = transform.header.stamp;
  data_out.header.frame_id = transform.child_frame_id;
  doTransform(data_in.polygon, data_out.polygon, transform);
}

template<>
void doTransform(
    const cras_geo_msgs::msg::PolygonCollection& data_in, cras_geo_msgs::msg::PolygonCollection& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  data_out.header.stamp = transform.header.stamp;
  data_out.header.frame_id = transform.child_frame_id;
  data_out.colors = data_in.colors;
  data_out.polygons.reserve(data_in.polygons.size());
  for (auto& polygon : data_in.polygons) {
    doTransform(polygon, data_out.polygons.emplace_back(), transform);
  }
}

template<>
void doTransform(
    const cras_geo_msgs::msg::PolygonInstanceCollection& data_in,
    cras_geo_msgs::msg::PolygonInstanceCollection& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  data_out.header.stamp = transform.header.stamp;
  data_out.header.frame_id = transform.child_frame_id;
  data_out.colors = data_in.colors;
  data_out.polygons.reserve(data_in.polygons.size());
  for (auto& polygon : data_in.polygons) {
    doTransform(polygon, data_out.polygons.emplace_back(), transform);
  }
}

template<>
void doTransform(
    const cras_geo_msgs::msg::PolygonInstanceStampedCollection& data_in,
    cras_geo_msgs::msg::PolygonInstanceStampedCollection& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  data_out.header.stamp = transform.header.stamp;
  data_out.header.frame_id = transform.child_frame_id;
  data_out.colors = data_in.colors;
  data_out.polygons.reserve(data_in.polygons.size());
  for (auto& polygon : data_in.polygons) {
    doTransform(polygon, data_out.polygons.emplace_back(), transform);
  }
}

template<>
void doTransform(
    const cras_geo_msgs::msg::PolygonStampedCollection& data_in,
    cras_geo_msgs::msg::PolygonStampedCollection& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  data_out.header.stamp = transform.header.stamp;
  data_out.header.frame_id = transform.child_frame_id;
  data_out.colors = data_in.colors;
  data_out.polygons.reserve(data_in.polygons.size());
  for (auto& polygon : data_in.polygons) {
    doTransform(polygon, data_out.polygons.emplace_back(), transform);
  }
}

template<>
void doTransform(
    const cras_geo_msgs::msg::Polygon64& data_in, cras_geo_msgs::msg::Polygon64& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  for (auto& point : data_in.points) {
    doTransform(point, data_out.points.emplace_back(), transform);
  }
}

template<>
void doTransform(
    const cras_geo_msgs::msg::Polygon64Instance& data_in, cras_geo_msgs::msg::Polygon64Instance& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  data_out.id = data_in.id;
  for (auto& point : data_in.polygon.points) {
    doTransform(point, data_out.polygon.points.emplace_back(), transform);
  }
}

template<>
void doTransform(
    const cras_geo_msgs::msg::Polygon64InstanceStamped& data_in, cras_geo_msgs::msg::Polygon64InstanceStamped& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  data_out.header = data_in.header;
  data_out.polygon.id = data_in.polygon.id;
  for (auto& point : data_in.polygon.polygon.points) {
    doTransform(point, data_out.polygon.polygon.points.emplace_back(), transform);
  }
}

template<>
void doTransform(
    const cras_geo_msgs::msg::Polygon64Stamped& data_in, cras_geo_msgs::msg::Polygon64Stamped& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  data_out.header = data_in.header;
  for (auto& point : data_in.polygon.points) {
    doTransform(point, data_out.polygon.points.emplace_back(), transform);
  }
}

template<>
void doTransform(
    const cras_geo_msgs::msg::Polygon64Collection& data_in, cras_geo_msgs::msg::Polygon64Collection& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  data_out.header.stamp = transform.header.stamp;
  data_out.header.frame_id = transform.child_frame_id;
  data_out.colors = data_in.colors;
  data_out.polygons.reserve(data_in.polygons.size());
  for (auto& polygon : data_in.polygons) {
    doTransform(polygon, data_out.polygons.emplace_back(), transform);
  }
}

template<>
void doTransform(
    const cras_geo_msgs::msg::Polygon64InstanceCollection& data_in,
    cras_geo_msgs::msg::Polygon64InstanceCollection& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  data_out.header.stamp = transform.header.stamp;
  data_out.header.frame_id = transform.child_frame_id;
  data_out.colors = data_in.colors;
  data_out.polygons.reserve(data_in.polygons.size());
  for (auto& polygon : data_in.polygons) {
    doTransform(polygon, data_out.polygons.emplace_back(), transform);
  }
}

template<>
void doTransform(
    const cras_geo_msgs::msg::Polygon64InstanceStampedCollection& data_in,
    cras_geo_msgs::msg::Polygon64InstanceStampedCollection& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  data_out.header.stamp = transform.header.stamp;
  data_out.header.frame_id = transform.child_frame_id;
  data_out.colors = data_in.colors;
  data_out.polygons.reserve(data_in.polygons.size());
  for (auto& polygon : data_in.polygons) {
    doTransform(polygon, data_out.polygons.emplace_back(), transform);
  }
}

template<>
void doTransform(
    const cras_geo_msgs::msg::Polygon64StampedCollection& data_in,
    cras_geo_msgs::msg::Polygon64StampedCollection& data_out,
    const geometry_msgs::msg::TransformStamped& transform) {
  data_out.header.stamp = transform.header.stamp;
  data_out.header.frame_id = transform.child_frame_id;
  data_out.colors = data_in.colors;
  data_out.polygons.reserve(data_in.polygons.size());
  for (auto& polygon : data_in.polygons) {
    doTransform(polygon, data_out.polygons.emplace_back(), transform);
  }
}

template<>
tf2::TimePoint getTimestamp<polygon_msgs::msg::Polygon2DStamped>(const polygon_msgs::msg::Polygon2DStamped& t) {
  return tf2_ros::fromMsg(t.header.stamp);
}

template<>
tf2::TimePoint getTimestamp<polygon_msgs::msg::Polygon2DCollection>(const polygon_msgs::msg::Polygon2DCollection& t) {
  return tf2_ros::fromMsg(t.header.stamp);
}

template<>
tf2::TimePoint getTimestamp<polygon_msgs::msg::ComplexPolygon2DStamped>(
    const polygon_msgs::msg::ComplexPolygon2DStamped& t) {
  return tf2_ros::fromMsg(t.header.stamp);
}

template<>
tf2::TimePoint getTimestamp<polygon_msgs::msg::ComplexPolygon2DCollection>(
    const polygon_msgs::msg::ComplexPolygon2DCollection& t) {
  return tf2_ros::fromMsg(t.header.stamp);
}

template<>
tf2::TimePoint getTimestamp<cras_geo_msgs::msg::Polygon64Stamped>(const cras_geo_msgs::msg::Polygon64Stamped& t) {
  return tf2_ros::fromMsg(t.header.stamp);
}

template<>
tf2::TimePoint getTimestamp<cras_geo_msgs::msg::Polygon64InstanceStamped>(
    const cras_geo_msgs::msg::Polygon64InstanceStamped& t) {
  return tf2_ros::fromMsg(t.header.stamp);
}

template<>
tf2::TimePoint getTimestamp<cras_geo_msgs::msg::Polygon64Collection>(const cras_geo_msgs::msg::Polygon64Collection& t) {
  return tf2_ros::fromMsg(t.header.stamp);
}

template<>
tf2::TimePoint getTimestamp<cras_geo_msgs::msg::Polygon64InstanceCollection>(
    const cras_geo_msgs::msg::Polygon64InstanceCollection& t) {
  return tf2_ros::fromMsg(t.header.stamp);
}

template<>
tf2::TimePoint getTimestamp<cras_geo_msgs::msg::Polygon64InstanceStampedCollection>(
    const cras_geo_msgs::msg::Polygon64InstanceStampedCollection& t) {
  return tf2_ros::fromMsg(t.header.stamp);
}

template<>
tf2::TimePoint getTimestamp<cras_geo_msgs::msg::Polygon64StampedCollection>(
    const cras_geo_msgs::msg::Polygon64StampedCollection& t) {
  return tf2_ros::fromMsg(t.header.stamp);
}

template<>
tf2::TimePoint getTimestamp<cras_geo_msgs::msg::PolygonCollection>(const cras_geo_msgs::msg::PolygonCollection& t) {
  return tf2_ros::fromMsg(t.header.stamp);
}

template<>
tf2::TimePoint getTimestamp<cras_geo_msgs::msg::PolygonInstanceCollection>(
    const cras_geo_msgs::msg::PolygonInstanceCollection& t) {
  return tf2_ros::fromMsg(t.header.stamp);
}

template<>
tf2::TimePoint getTimestamp<cras_geo_msgs::msg::PolygonInstanceStampedCollection>(
    const cras_geo_msgs::msg::PolygonInstanceStampedCollection& t) {
  return tf2_ros::fromMsg(t.header.stamp);
}

template<>
tf2::TimePoint getTimestamp<cras_geo_msgs::msg::PolygonStampedCollection>(
    const cras_geo_msgs::msg::PolygonStampedCollection& t) {
  return tf2_ros::fromMsg(t.header.stamp);
}

template<>
std::string getFrameId<polygon_msgs::msg::Polygon2DStamped>(const polygon_msgs::msg::Polygon2DStamped& t) {
  return t.header.frame_id;
}

template<>
std::string getFrameId<polygon_msgs::msg::Polygon2DCollection>(const polygon_msgs::msg::Polygon2DCollection& t) {
  return t.header.frame_id;
}

template<>
std::string getFrameId<polygon_msgs::msg::ComplexPolygon2DStamped>(
    const polygon_msgs::msg::ComplexPolygon2DStamped& t) {
  return t.header.frame_id;
}

template<>
std::string getFrameId<polygon_msgs::msg::ComplexPolygon2DCollection>(
    const polygon_msgs::msg::ComplexPolygon2DCollection& t) {
  return t.header.frame_id;
}

template<>
std::string getFrameId<cras_geo_msgs::msg::Polygon64Stamped>(const cras_geo_msgs::msg::Polygon64Stamped& t) {
  return t.header.frame_id;
}

template<>
std::string getFrameId<cras_geo_msgs::msg::Polygon64InstanceStamped>(
    const cras_geo_msgs::msg::Polygon64InstanceStamped& t) {
  return t.header.frame_id;
}

template<>
std::string getFrameId<cras_geo_msgs::msg::Polygon64Collection>(const cras_geo_msgs::msg::Polygon64Collection& t) {
  return t.header.frame_id;
}

template<>
std::string getFrameId<cras_geo_msgs::msg::Polygon64InstanceCollection>(
    const cras_geo_msgs::msg::Polygon64InstanceCollection& t) {
  return t.header.frame_id;
}

template<>
std::string getFrameId<cras_geo_msgs::msg::Polygon64InstanceStampedCollection>(
    const cras_geo_msgs::msg::Polygon64InstanceStampedCollection& t) {
  return t.header.frame_id;
}

template<>
std::string getFrameId<cras_geo_msgs::msg::Polygon64StampedCollection>(
    const cras_geo_msgs::msg::Polygon64StampedCollection& t) {
  return t.header.frame_id;
}

template<>
std::string getFrameId<cras_geo_msgs::msg::PolygonCollection>(const cras_geo_msgs::msg::PolygonCollection& t) {
  return t.header.frame_id;
}

template<>
std::string getFrameId<cras_geo_msgs::msg::PolygonInstanceCollection>(
    const cras_geo_msgs::msg::PolygonInstanceCollection& t) {
  return t.header.frame_id;
}

template<>
std::string getFrameId<cras_geo_msgs::msg::PolygonInstanceStampedCollection>(
    const cras_geo_msgs::msg::PolygonInstanceStampedCollection& t) {
  return t.header.frame_id;
}

template<>
std::string getFrameId<cras_geo_msgs::msg::PolygonStampedCollection>(
    const cras_geo_msgs::msg::PolygonStampedCollection& t) {
  return t.header.frame_id;
}

}  // namespace tf2
