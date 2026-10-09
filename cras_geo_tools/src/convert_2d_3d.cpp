// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Conversions between 2D and 3D polygon messages.
 * \author Martin Pecka
 */

#include <cmath>
#include <string>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/math_utils.hpp>
#include <cras_cpp_common/time_utils.hpp>
#include <cras_geo_msgs/msg/polygon_collection.hpp>
#include <cras_geo_msgs/msg/polygon_instance_collection.hpp>
#include <cras_geo_msgs/msg/polygon_instance_stamped_collection.hpp>
#include <cras_geo_msgs/msg/polygon_stamped_collection.hpp>
#include <cras_geo_tools/convert_2d_3d.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_instance.hpp>
#include <geometry_msgs/msg/polygon_instance_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <polygon_msgs/msg/polygon2_d_collection.hpp>
#include <polygon_msgs/msg/polygon2_d_stamped.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace cras_geo_tools {

template<>
cras::expected<polygon_msgs::msg::Polygon2DStamped, ConversionError> convert3DTo2D(
    const geometry_msgs::msg::PolygonStamped& msg_3d, const ConversionOptions& options) {
  polygon_msgs::msg::Polygon2DStamped out;
  out.header = msg_3d.header;
  auto min_z {std::numeric_limits<float>::max()};
  auto max_z {std::numeric_limits<float>::min()};
  out.polygon.points.reserve(msg_3d.polygon.points.size());
  for (const auto& point_3d : msg_3d.polygon.points) {
    auto& point_2d = out.polygon.points.emplace_back();
    point_2d.x = point_3d.x;
    point_2d.y = point_3d.y;
    min_z = std::min(min_z, point_3d.z);
    max_z = std::max(max_z, point_3d.z);
  }
  if (!std::isnan(options.z_diff_threshold) && max_z - min_z > options.z_diff_threshold) {
    return cras::make_unexpected(
      ConversionError{.z_diff_too_large = true, .error_message = "Polygon is not flat"});
  }
  return out;
}

template<>
cras::expected<polygon_msgs::msg::Polygon2DStamped, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::Polygon64Stamped& msg_3d, const ConversionOptions& options) {
  polygon_msgs::msg::Polygon2DStamped out;
  out.header = msg_3d.header;
  auto min_z {std::numeric_limits<double>::max()};
  auto max_z {std::numeric_limits<double>::min()};
  out.polygon.points.reserve(msg_3d.polygon.points.size());
  for (const auto& point_3d : msg_3d.polygon.points) {
    auto& point_2d = out.polygon.points.emplace_back();
    point_2d.x = point_3d.x;
    point_2d.y = point_3d.y;
    min_z = std::min(min_z, point_3d.z);
    max_z = std::max(max_z, point_3d.z);
  }
  if (!std::isnan(options.z_diff_threshold) && max_z - min_z > options.z_diff_threshold) {
    return cras::make_unexpected(
      ConversionError{.z_diff_too_large = true, .error_message = "Polygon is not flat"});
  }
  return out;
}

template<>
cras::expected<polygon_msgs::msg::Polygon2DStamped, ConversionError> convert3DTo2D(
    const geometry_msgs::msg::PolygonInstanceStamped& msg_3d, const ConversionOptions& options) {
  polygon_msgs::msg::Polygon2DStamped out;
  out.header = msg_3d.header;
  auto min_z {std::numeric_limits<float>::max()};
  auto max_z {std::numeric_limits<float>::min()};
  out.polygon.points.reserve(msg_3d.polygon.polygon.points.size());
  for (const auto& point_3d : msg_3d.polygon.polygon.points) {
    auto& point_2d = out.polygon.points.emplace_back();
    point_2d.x = point_3d.x;
    point_2d.y = point_3d.y;
    min_z = std::min(min_z, point_3d.z);
    max_z = std::max(max_z, point_3d.z);
  }
  if (!std::isnan(options.z_diff_threshold) && max_z - min_z > options.z_diff_threshold) {
    return cras::make_unexpected(
      ConversionError{.z_diff_too_large = true, .error_message = "Polygon is not flat"});
  }
  return out;
}

template<>
cras::expected<polygon_msgs::msg::Polygon2DStamped, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::Polygon64InstanceStamped& msg_3d, const ConversionOptions& options) {
  polygon_msgs::msg::Polygon2DStamped out;
  out.header = msg_3d.header;
  auto min_z {std::numeric_limits<double>::max()};
  auto max_z {std::numeric_limits<double>::min()};
  out.polygon.points.reserve(msg_3d.polygon.polygon.points.size());
  for (const auto& point_3d : msg_3d.polygon.polygon.points) {
    auto& point_2d = out.polygon.points.emplace_back();
    point_2d.x = point_3d.x;
    point_2d.y = point_3d.y;
    min_z = std::min(min_z, point_3d.z);
    max_z = std::max(max_z, point_3d.z);
  }
  if (!std::isnan(options.z_diff_threshold) && max_z - min_z > options.z_diff_threshold) {
    return cras::make_unexpected(
      ConversionError{.z_diff_too_large = true, .error_message = "Polygon is not flat"});
  }
  return out;
}

template<>
cras::expected<polygon_msgs::msg::Polygon2DCollection, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::PolygonCollection& msg_3d, const ConversionOptions& options) {
  polygon_msgs::msg::Polygon2DCollection out;
  out.header = msg_3d.header;
  out.colors = msg_3d.colors;
  auto collection_min_z {std::numeric_limits<float>::max()};
  auto collection_max_z {std::numeric_limits<float>::min()};

  out.polygons.reserve(msg_3d.polygons.size());
  for (const auto& polygon_3d : msg_3d.polygons) {
    auto min_z {std::numeric_limits<float>::max()};
    auto max_z {std::numeric_limits<float>::min()};
    auto& polygon_2d = out.polygons.emplace_back();
    polygon_2d.points.reserve(polygon_3d.points.size());
    for (const auto& point_3d : polygon_3d.points) {
      auto& point_2d = polygon_2d.points.emplace_back();
      point_2d.x = point_3d.x;
      point_2d.y = point_3d.y;
      min_z = std::min(min_z, point_3d.z);
      max_z = std::max(max_z, point_3d.z);
      collection_min_z = std::min(collection_min_z, point_3d.z);
      collection_max_z = std::max(collection_max_z, point_3d.z);
    }
    if (!options.skip_errors && !std::isnan(options.z_diff_threshold) && (max_z - min_z) > options.z_diff_threshold) {
      return cras::make_unexpected(
        ConversionError{.z_diff_too_large = true, .error_message = "Polygon is not flat"});
    }
  }

  if (!std::isnan(options.collection_z_diff_threshold) &&
      (collection_max_z - collection_min_z) > options.collection_z_diff_threshold)
  {
    return cras::make_unexpected(
      ConversionError{.z_diff_too_large = true, .error_message = "Polygons in collection do not have the same z."});
  }

  return out;
}

template<>
cras::expected<polygon_msgs::msg::Polygon2DCollection, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::Polygon64Collection& msg_3d, const ConversionOptions& options) {
  polygon_msgs::msg::Polygon2DCollection out;
  out.header = msg_3d.header;
  out.colors = msg_3d.colors;
  auto collection_min_z {std::numeric_limits<double>::max()};
  auto collection_max_z {std::numeric_limits<double>::min()};

  out.polygons.reserve(msg_3d.polygons.size());
  for (const auto& polygon_3d : msg_3d.polygons) {
    auto min_z {std::numeric_limits<double>::max()};
    auto max_z {std::numeric_limits<double>::min()};
    auto& polygon_2d = out.polygons.emplace_back();
    polygon_2d.points.reserve(polygon_3d.points.size());
    for (const auto& point_3d : polygon_3d.points) {
      auto& point_2d = polygon_2d.points.emplace_back();
      point_2d.x = point_3d.x;
      point_2d.y = point_3d.y;
      min_z = std::min(min_z, point_3d.z);
      max_z = std::max(max_z, point_3d.z);
      collection_min_z = std::min(collection_min_z, point_3d.z);
      collection_max_z = std::max(collection_max_z, point_3d.z);
    }
    if (!options.skip_errors && !std::isnan(options.z_diff_threshold) && (max_z - min_z) > options.z_diff_threshold) {
      return cras::make_unexpected(
        ConversionError{.z_diff_too_large = true, .error_message = "Polygon is not flat"});
    }
  }

  if (!std::isnan(options.collection_z_diff_threshold) &&
      (collection_max_z - collection_min_z) > options.collection_z_diff_threshold)
  {
    return cras::make_unexpected(
      ConversionError{.z_diff_too_large = true, .error_message = "Polygons in collection do not have the same z."});
  }

  return out;
}

template<>
cras::expected<polygon_msgs::msg::Polygon2DCollection, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::PolygonInstanceCollection& msg_3d, const ConversionOptions& options) {
  polygon_msgs::msg::Polygon2DCollection out;
  out.header = msg_3d.header;
  out.colors = msg_3d.colors;
  auto collection_min_z {std::numeric_limits<float>::max()};
  auto collection_max_z {std::numeric_limits<float>::min()};

  out.polygons.reserve(msg_3d.polygons.size());
  for (const auto& polygon_3d : msg_3d.polygons) {
    auto min_z {std::numeric_limits<float>::max()};
    auto max_z {std::numeric_limits<float>::min()};
    auto& polygon_2d = out.polygons.emplace_back();
    polygon_2d.points.reserve(polygon_3d.polygon.points.size());
    for (const auto& point_3d : polygon_3d.polygon.points) {
      auto& point_2d = polygon_2d.points.emplace_back();
      point_2d.x = point_3d.x;
      point_2d.y = point_3d.y;
      min_z = std::min(min_z, point_3d.z);
      max_z = std::max(max_z, point_3d.z);
      collection_min_z = std::min(collection_min_z, point_3d.z);
      collection_max_z = std::max(collection_max_z, point_3d.z);
    }
    if (!options.skip_errors && !std::isnan(options.z_diff_threshold) && (max_z - min_z) > options.z_diff_threshold) {
      return cras::make_unexpected(
        ConversionError{.z_diff_too_large = true, .error_message = "Polygon is not flat"});
    }
  }

  if (!std::isnan(options.collection_z_diff_threshold) &&
      (collection_max_z - collection_min_z) > options.collection_z_diff_threshold)
  {
    return cras::make_unexpected(
      ConversionError{.z_diff_too_large = true, .error_message = "Polygons in collection do not have the same z."});
  }

  return out;
}

template<>
cras::expected<polygon_msgs::msg::Polygon2DCollection, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::Polygon64InstanceCollection& msg_3d, const ConversionOptions& options) {
  polygon_msgs::msg::Polygon2DCollection out;
  out.header = msg_3d.header;
  out.colors = msg_3d.colors;
  auto collection_min_z {std::numeric_limits<double>::max()};
  auto collection_max_z {std::numeric_limits<double>::min()};

  out.polygons.reserve(msg_3d.polygons.size());
  for (const auto& polygon_3d : msg_3d.polygons) {
    auto min_z {std::numeric_limits<double>::max()};
    auto max_z {std::numeric_limits<double>::min()};
    auto& polygon_2d = out.polygons.emplace_back();
    polygon_2d.points.reserve(polygon_3d.polygon.points.size());
    for (const auto& point_3d : polygon_3d.polygon.points) {
      auto& point_2d = polygon_2d.points.emplace_back();
      point_2d.x = point_3d.x;
      point_2d.y = point_3d.y;
      min_z = std::min(min_z, point_3d.z);
      max_z = std::max(max_z, point_3d.z);
      collection_min_z = std::min(collection_min_z, point_3d.z);
      collection_max_z = std::max(collection_max_z, point_3d.z);
    }
    if (!options.skip_errors && !std::isnan(options.z_diff_threshold) && (max_z - min_z) > options.z_diff_threshold) {
      return cras::make_unexpected(
        ConversionError{.z_diff_too_large = true, .error_message = "Polygon is not flat"});
    }
  }

  if (!std::isnan(options.collection_z_diff_threshold) &&
      (collection_max_z - collection_min_z) > options.collection_z_diff_threshold)
  {
    return cras::make_unexpected(
      ConversionError{.z_diff_too_large = true, .error_message = "Polygons in collection do not have the same z."});
  }

  return out;
}

template<>
cras::expected<polygon_msgs::msg::Polygon2DCollection, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::PolygonInstanceStampedCollection& msg_3d, const ConversionOptions& options) {
  polygon_msgs::msg::Polygon2DCollection out;
  out.header = msg_3d.header;
  out.colors.reserve(msg_3d.colors.size());
  auto collection_min_z {std::numeric_limits<double>::max()};
  auto collection_max_z {std::numeric_limits<double>::min()};

  out.polygons.reserve(msg_3d.polygons.size());
  for (size_t i = 0u; i < msg_3d.polygons.size(); ++i) {
    const auto& polygon_3d = msg_3d.polygons[i];
    const auto& polygon_header = polygon_3d.header;
    if (options.require_same_frame && polygon_header.frame_id != msg_3d.header.frame_id) {
      if (options.skip_errors) {
        continue;
      } else {
        return cras::make_unexpected(
          ConversionError{.frame_mismatch = true, .error_message = "Polygon frame does not match collection frame"});
      }
    }
    if (options.require_same_stamp && polygon_header.stamp != msg_3d.header.stamp) {
      if (options.skip_errors) {
        continue;
      } else {
        return cras::make_unexpected(
          ConversionError{.stamp_mismatch = true, .error_message = "Polygon stamp does not match collection stamp"});
      }
    }

    tf2::Transform tf(tf2::Transform::getIdentity());
    if ((options.transform_to_header_frame && polygon_header.frame_id != msg_3d.header.frame_id) ||
        (options.transform_to_header_stamp && polygon_header.stamp != msg_3d.header.stamp))
    {
      if (options.tf == nullptr) {
        if (options.skip_errors) {
          continue;
        } else {
          return cras::make_unexpected(
            ConversionError{.tf_error = true, .error_message = "No transform buffer provided"});
        }
      }
      const auto frame = options.transform_to_header_frame ? polygon_header.frame_id : msg_3d.header.frame_id;
      const auto stamp = options.transform_to_header_stamp ? polygon_header.stamp : msg_3d.header.stamp;
      const auto tf_stamp = cras::convertTime<tf2::TimePoint>(stamp);
      try {
        tf2::convert(
          options.tf->lookupTransform(frame, polygon_header.frame_id, tf_stamp, options.tf_timeout).transform, tf);
      } catch (const tf2::TransformException& ex) {
        if (options.skip_errors) {
          continue;
        } else {
          return cras::make_unexpected(
            ConversionError{.tf_error = true, .error_message = ex.what()});
        }
      }
    }

    auto min_z {std::numeric_limits<double>::max()};
    auto max_z {std::numeric_limits<double>::min()};
    auto& polygon_2d = out.polygons.emplace_back();
    polygon_2d.points.reserve(polygon_3d.polygon.polygon.points.size());
    if (i < msg_3d.colors.size()) {
      out.colors.emplace_back(msg_3d.colors[i]);
    }
    for (const auto& point_3d : polygon_3d.polygon.polygon.points) {
      tf2::Vector3 transformed_point;
      tf2::convert(point_3d, transformed_point);
      transformed_point = tf * transformed_point;
      auto& point_2d = polygon_2d.points.emplace_back();
      point_2d.x = transformed_point.x();
      point_2d.y = transformed_point.y();
      min_z = std::min(min_z, transformed_point.z());
      max_z = std::max(max_z, transformed_point.z());
      collection_min_z = std::min(collection_min_z, transformed_point.z());
      collection_max_z = std::max(collection_max_z, transformed_point.z());
    }
    if (!options.skip_errors && !std::isnan(options.z_diff_threshold) && (max_z - min_z) > options.z_diff_threshold) {
      return cras::make_unexpected(
        ConversionError{.z_diff_too_large = true, .error_message = "Polygon is not flat"});
    }
  }

  if (!std::isnan(options.collection_z_diff_threshold) &&
      (collection_max_z - collection_min_z) > options.collection_z_diff_threshold)
  {
    return cras::make_unexpected(
      ConversionError{.z_diff_too_large = true, .error_message = "Polygons in collection do not have the same z."});
  }

  return out;
}

template<>
cras::expected<polygon_msgs::msg::Polygon2DCollection, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::Polygon64InstanceStampedCollection& msg_3d, const ConversionOptions& options) {
  polygon_msgs::msg::Polygon2DCollection out;
  out.header = msg_3d.header;
  out.colors.reserve(msg_3d.colors.size());
  auto collection_min_z {std::numeric_limits<double>::max()};
  auto collection_max_z {std::numeric_limits<double>::min()};

  out.polygons.reserve(msg_3d.polygons.size());
  for (size_t i = 0u; i < msg_3d.polygons.size(); ++i) {
    const auto& polygon_3d = msg_3d.polygons[i];
    const auto& polygon_header = polygon_3d.header;
    if (options.require_same_frame && polygon_header.frame_id != msg_3d.header.frame_id) {
      if (options.skip_errors) {
        continue;
      } else {
        return cras::make_unexpected(
          ConversionError{.frame_mismatch = true, .error_message = "Polygon frame does not match collection frame"});
      }
    }
    if (options.require_same_stamp && polygon_header.stamp != msg_3d.header.stamp) {
      if (options.skip_errors) {
        continue;
      } else {
        return cras::make_unexpected(
          ConversionError{.stamp_mismatch = true, .error_message = "Polygon stamp does not match collection stamp"});
      }
    }

    tf2::Transform tf(tf2::Transform::getIdentity());
    if ((options.transform_to_header_frame && polygon_header.frame_id != msg_3d.header.frame_id) ||
        (options.transform_to_header_stamp && polygon_header.stamp != msg_3d.header.stamp))
    {
      if (options.tf == nullptr) {
        if (options.skip_errors) {
          continue;
        } else {
          return cras::make_unexpected(
            ConversionError{.tf_error = true, .error_message = "No transform buffer provided"});
        }
      }
      const auto frame = options.transform_to_header_frame ? polygon_header.frame_id : msg_3d.header.frame_id;
      const auto stamp = options.transform_to_header_stamp ? polygon_header.stamp : msg_3d.header.stamp;
      const auto tf_stamp = cras::convertTime<tf2::TimePoint>(stamp);
      try {
        tf2::convert(
          options.tf->lookupTransform(frame, polygon_header.frame_id, tf_stamp, options.tf_timeout).transform, tf);
      } catch (const tf2::TransformException& ex) {
        if (options.skip_errors) {
          continue;
        } else {
          return cras::make_unexpected(
            ConversionError{.tf_error = true, .error_message = ex.what()});
        }
      }
    }

    auto min_z {std::numeric_limits<double>::max()};
    auto max_z {std::numeric_limits<double>::min()};
    auto& polygon_2d = out.polygons.emplace_back();
    polygon_2d.points.reserve(polygon_3d.polygon.polygon.points.size());
    if (i < msg_3d.colors.size()) {
      out.colors.emplace_back(msg_3d.colors[i]);
    }
    for (const auto& point_3d : polygon_3d.polygon.polygon.points) {
      tf2::Vector3 transformed_point;
      tf2::convert(point_3d, transformed_point);
      transformed_point = tf * transformed_point;
      auto& point_2d = polygon_2d.points.emplace_back();
      point_2d.x = transformed_point.x();
      point_2d.y = transformed_point.y();
      min_z = std::min(min_z, transformed_point.z());
      max_z = std::max(max_z, transformed_point.z());
      collection_min_z = std::min(collection_min_z, transformed_point.z());
      collection_max_z = std::max(collection_max_z, transformed_point.z());
    }
    if (!options.skip_errors && !std::isnan(options.z_diff_threshold) && (max_z - min_z) > options.z_diff_threshold) {
      return cras::make_unexpected(
        ConversionError{.z_diff_too_large = true, .error_message = "Polygon is not flat"});
    }
  }

  if (!std::isnan(options.collection_z_diff_threshold) &&
      (collection_max_z - collection_min_z) > options.collection_z_diff_threshold)
  {
    return cras::make_unexpected(
      ConversionError{.z_diff_too_large = true, .error_message = "Polygons in collection do not have the same z."});
  }

  return out;
}

template<>
cras::expected<polygon_msgs::msg::Polygon2DCollection, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::PolygonStampedCollection& msg_3d, const ConversionOptions& options) {
  polygon_msgs::msg::Polygon2DCollection out;
  out.header = msg_3d.header;
  out.colors.reserve(msg_3d.colors.size());
  auto collection_min_z {std::numeric_limits<double>::max()};
  auto collection_max_z {std::numeric_limits<double>::min()};

  out.polygons.reserve(msg_3d.polygons.size());
  for (size_t i = 0u; i < msg_3d.polygons.size(); ++i) {
    const auto& polygon_3d = msg_3d.polygons[i];
    const auto& polygon_header = polygon_3d.header;
    if (options.require_same_frame && polygon_header.frame_id != msg_3d.header.frame_id) {
      if (options.skip_errors) {
        continue;
      } else {
        return cras::make_unexpected(
          ConversionError{.frame_mismatch = true, .error_message = "Polygon frame does not match collection frame"});
      }
    }
    if (options.require_same_stamp && polygon_header.stamp != msg_3d.header.stamp) {
      if (options.skip_errors) {
        continue;
      } else {
        return cras::make_unexpected(
          ConversionError{.stamp_mismatch = true, .error_message = "Polygon stamp does not match collection stamp"});
      }
    }

    tf2::Transform tf(tf2::Transform::getIdentity());
    if ((options.transform_to_header_frame && polygon_header.frame_id != msg_3d.header.frame_id) ||
        (options.transform_to_header_stamp && polygon_header.stamp != msg_3d.header.stamp))
    {
      if (options.tf == nullptr) {
        if (options.skip_errors) {
          continue;
        } else {
          return cras::make_unexpected(
            ConversionError{.tf_error = true, .error_message = "No transform buffer provided"});
        }
      }
      const auto frame = options.transform_to_header_frame ? polygon_header.frame_id : msg_3d.header.frame_id;
      const auto stamp = options.transform_to_header_stamp ? polygon_header.stamp : msg_3d.header.stamp;
      const auto tf_stamp = cras::convertTime<tf2::TimePoint>(stamp);
      try {
        tf2::convert(
          options.tf->lookupTransform(frame, polygon_header.frame_id, tf_stamp, options.tf_timeout).transform, tf);
      } catch (const tf2::TransformException& ex) {
        if (options.skip_errors) {
          continue;
        } else {
          return cras::make_unexpected(
            ConversionError{.tf_error = true, .error_message = ex.what()});
        }
      }
    }

    auto min_z {std::numeric_limits<double>::max()};
    auto max_z {std::numeric_limits<double>::min()};
    auto& polygon_2d = out.polygons.emplace_back();
    polygon_2d.points.reserve(polygon_3d.polygon.points.size());
    if (i < msg_3d.colors.size()) {
      out.colors.emplace_back(msg_3d.colors[i]);
    }
    for (const auto& point_3d : polygon_3d.polygon.points) {
      tf2::Vector3 transformed_point;
      tf2::convert(point_3d, transformed_point);
      transformed_point = tf * transformed_point;
      auto& point_2d = polygon_2d.points.emplace_back();
      point_2d.x = transformed_point.x();
      point_2d.y = transformed_point.y();
      min_z = std::min(min_z, transformed_point.z());
      max_z = std::max(max_z, transformed_point.z());
      collection_min_z = std::min(collection_min_z, transformed_point.z());
      collection_max_z = std::max(collection_max_z, transformed_point.z());
    }
    if (!options.skip_errors && !std::isnan(options.z_diff_threshold) && (max_z - min_z) > options.z_diff_threshold) {
      return cras::make_unexpected(
        ConversionError{.z_diff_too_large = true, .error_message = "Polygon is not flat"});
    }
  }

  if (!std::isnan(options.collection_z_diff_threshold) &&
      (collection_max_z - collection_min_z) > options.collection_z_diff_threshold)
  {
    return cras::make_unexpected(
      ConversionError{.z_diff_too_large = true, .error_message = "Polygons in collection do not have the same z."});
  }

  return out;
}

template<>
cras::expected<polygon_msgs::msg::Polygon2DCollection, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::Polygon64StampedCollection& msg_3d, const ConversionOptions& options) {
  polygon_msgs::msg::Polygon2DCollection out;
  out.header = msg_3d.header;
  out.colors.reserve(msg_3d.colors.size());
  auto collection_min_z {std::numeric_limits<double>::max()};
  auto collection_max_z {std::numeric_limits<double>::min()};

  out.polygons.reserve(msg_3d.polygons.size());
  for (size_t i = 0u; i < msg_3d.polygons.size(); ++i) {
    const auto& polygon_3d = msg_3d.polygons[i];
    const auto& polygon_header = polygon_3d.header;
    if (options.require_same_frame && polygon_header.frame_id != msg_3d.header.frame_id) {
      if (options.skip_errors) {
        continue;
      } else {
        return cras::make_unexpected(
          ConversionError{.frame_mismatch = true, .error_message = "Polygon frame does not match collection frame"});
      }
    }
    if (options.require_same_stamp && polygon_header.stamp != msg_3d.header.stamp) {
      if (options.skip_errors) {
        continue;
      } else {
        return cras::make_unexpected(
          ConversionError{.stamp_mismatch = true, .error_message = "Polygon stamp does not match collection stamp"});
      }
    }

    tf2::Transform tf(tf2::Transform::getIdentity());
    if ((options.transform_to_header_frame && polygon_header.frame_id != msg_3d.header.frame_id) ||
        (options.transform_to_header_stamp && polygon_header.stamp != msg_3d.header.stamp))
    {
      if (options.tf == nullptr) {
        if (options.skip_errors) {
          continue;
        } else {
          return cras::make_unexpected(
            ConversionError{.tf_error = true, .error_message = "No transform buffer provided"});
        }
      }
      const auto frame = options.transform_to_header_frame ? polygon_header.frame_id : msg_3d.header.frame_id;
      const auto stamp = options.transform_to_header_stamp ? polygon_header.stamp : msg_3d.header.stamp;
      const auto tf_stamp = cras::convertTime<tf2::TimePoint>(stamp);
      try {
        tf2::convert(
          options.tf->lookupTransform(frame, polygon_header.frame_id, tf_stamp, options.tf_timeout).transform, tf);
      } catch (const tf2::TransformException& ex) {
        if (options.skip_errors) {
          continue;
        } else {
          return cras::make_unexpected(
            ConversionError{.tf_error = true, .error_message = ex.what()});
        }
      }
    }

    auto min_z {std::numeric_limits<double>::max()};
    auto max_z {std::numeric_limits<double>::min()};
    auto& polygon_2d = out.polygons.emplace_back();
    polygon_2d.points.reserve(polygon_3d.polygon.points.size());
    if (i < msg_3d.colors.size()) {
      out.colors.emplace_back(msg_3d.colors[i]);
    }
    for (const auto& point_3d : polygon_3d.polygon.points) {
      tf2::Vector3 transformed_point;
      tf2::convert(point_3d, transformed_point);
      transformed_point = tf * transformed_point;
      auto& point_2d = polygon_2d.points.emplace_back();
      point_2d.x = transformed_point.x();
      point_2d.y = transformed_point.y();
      min_z = std::min(min_z, transformed_point.z());
      max_z = std::max(max_z, transformed_point.z());
      collection_min_z = std::min(collection_min_z, transformed_point.z());
      collection_max_z = std::max(collection_max_z, transformed_point.z());
    }
    if (!options.skip_errors && !std::isnan(options.z_diff_threshold) && (max_z - min_z) > options.z_diff_threshold) {
      return cras::make_unexpected(
        ConversionError{.z_diff_too_large = true, .error_message = "Polygon is not flat"});
    }
  }

  if (!std::isnan(options.collection_z_diff_threshold) &&
      (collection_max_z - collection_min_z) > options.collection_z_diff_threshold)
  {
    return cras::make_unexpected(
      ConversionError{.z_diff_too_large = true, .error_message = "Polygons in collection do not have the same z."});
  }

  return out;
}

template<>
geometry_msgs::msg::PolygonStamped convert2DTo3D(const polygon_msgs::msg::Polygon2DStamped& msg_2d) {
  geometry_msgs::msg::PolygonStamped out;
  out.header = msg_2d.header;
  out.polygon.points.reserve(msg_2d.polygon.points.size());
  for (const auto& point_2d : msg_2d.polygon.points) {
    auto& point_3d = out.polygon.points.emplace_back();
    point_3d.x = cras::saturating_cast<float>(point_2d.x);
    point_3d.y = cras::saturating_cast<float>(point_2d.y);
    point_3d.z = 0.0;
  }
  return out;
}

template<>
cras_geo_msgs::msg::Polygon64Stamped convert2DTo3D(const polygon_msgs::msg::Polygon2DStamped& msg_2d) {
  cras_geo_msgs::msg::Polygon64Stamped out;
  out.header = msg_2d.header;
  out.polygon.points.reserve(msg_2d.polygon.points.size());
  for (const auto& point_2d : msg_2d.polygon.points) {
    auto& point_3d = out.polygon.points.emplace_back();
    point_3d.x = point_2d.x;
    point_3d.y = point_2d.y;
    point_3d.z = 0.0;
  }
  return out;
}

template<>
cras_geo_msgs::msg::PolygonCollection convert2DTo3D(const polygon_msgs::msg::Polygon2DCollection& msg_2d) {
  cras_geo_msgs::msg::PolygonCollection out;
  out.header = msg_2d.header;
  out.colors = msg_2d.colors;
  out.polygons.reserve(msg_2d.polygons.size());
  for (const auto& polygon_2d : msg_2d.polygons) {
    auto& polygon_3d = out.polygons.emplace_back();
    polygon_3d.points.reserve(polygon_2d.points.size());
    for (const auto& point_2d : polygon_2d.points) {
      auto& point_3d = polygon_3d.points.emplace_back();
      point_3d.x = cras::saturating_cast<float>(point_2d.x);
      point_3d.y = cras::saturating_cast<float>(point_2d.y);
      point_3d.z = 0.0;
    }
  }
  return out;
}

template<>
cras_geo_msgs::msg::Polygon64Collection convert2DTo3D(const polygon_msgs::msg::Polygon2DCollection& msg_2d) {
  cras_geo_msgs::msg::Polygon64Collection out;
  out.header = msg_2d.header;
  out.colors = msg_2d.colors;
  out.polygons.reserve(msg_2d.polygons.size());
  for (const auto& polygon_2d : msg_2d.polygons) {
    auto& polygon_3d = out.polygons.emplace_back();
    polygon_3d.points.reserve(polygon_2d.points.size());
    for (const auto& point_2d : polygon_2d.points) {
      auto& point_3d = polygon_3d.points.emplace_back();
      point_3d.x = point_2d.x;
      point_3d.y = point_2d.y;
      point_3d.z = 0.0;
    }
  }
  return out;
}

template<>
cras_geo_msgs::msg::PolygonStampedCollection convert2DTo3D(const polygon_msgs::msg::Polygon2DCollection& msg_2d) {
  cras_geo_msgs::msg::PolygonStampedCollection out;
  out.header = msg_2d.header;
  out.colors = msg_2d.colors;
  out.polygons.reserve(msg_2d.polygons.size());
  for (const auto& polygon_2d : msg_2d.polygons) {
    auto& polygon_3d = out.polygons.emplace_back();
    polygon_3d.header = msg_2d.header;
    polygon_3d.polygon.points.reserve(polygon_2d.points.size());
    for (const auto& point_2d : polygon_2d.points) {
      auto& point_3d = polygon_3d.polygon.points.emplace_back();
      point_3d.x = cras::saturating_cast<float>(point_2d.x);
      point_3d.y = cras::saturating_cast<float>(point_2d.y);
      point_3d.z = 0.0;
    }
  }
  return out;
}

template<>
cras_geo_msgs::msg::Polygon64StampedCollection convert2DTo3D(const polygon_msgs::msg::Polygon2DCollection& msg_2d) {
  cras_geo_msgs::msg::Polygon64StampedCollection out;
  out.header = msg_2d.header;
  out.colors = msg_2d.colors;
  out.polygons.reserve(msg_2d.polygons.size());
  for (const auto& polygon_2d : msg_2d.polygons) {
    auto& polygon_3d = out.polygons.emplace_back();
    polygon_3d.header = msg_2d.header;
    polygon_3d.polygon.points.reserve(polygon_2d.points.size());
    for (const auto& point_2d : polygon_2d.points) {
      auto& point_3d = polygon_3d.polygon.points.emplace_back();
      point_3d.x = point_2d.x;
      point_3d.y = point_2d.y;
      point_3d.z = 0.0;
    }
  }
  return out;
}

}  // namespace cras_geo_tools
