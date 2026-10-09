// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Conversions between 2D and 3D polygon messages.
 * \author Martin Pecka
 */

#pragma once

#include <limits>
#include <string>

#include <cras_cpp_common/expected.hpp>
#include <cras_geo_msgs/msg/polygon64_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_instance_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_instance_stamped.hpp>
#include <cras_geo_msgs/msg/polygon64_instance_stamped_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_stamped.hpp>
#include <cras_geo_msgs/msg/polygon64_stamped_collection.hpp>
#include <cras_geo_msgs/msg/polygon_collection.hpp>
#include <cras_geo_msgs/msg/polygon_instance_collection.hpp>
#include <cras_geo_msgs/msg/polygon_instance_stamped_collection.hpp>
#include <cras_geo_msgs/msg/polygon_stamped_collection.hpp>
#include <geometry_msgs/msg/polygon_instance_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <polygon_msgs/msg/polygon2_d_collection.hpp>
#include <polygon_msgs/msg/polygon2_d_stamped.hpp>
#include <tf2_ros/buffer_interface.hpp>

namespace cras_geo_tools {

/**
 * \brief Options for conversion from 3D to 2D.
 */
struct ConversionOptions {
  bool skip_errors {false};  //!< Collections only: skip polygons with errors instead of failing.
  //! Maximal difference of the Z-coordinates within one polygon. NaN disables the check.
  double z_diff_threshold {std::numeric_limits<double>::quiet_NaN()};
  //! Maximal difference of the Z-coordinates among all polygons in the collection. NaN disables the check.
  double collection_z_diff_threshold {std::numeric_limits<double>::quiet_NaN()};
  bool require_same_frame {false};  //!< Collections only: all polygons must have the same frame as the collection.
  bool require_same_stamp {false};  //!< Collections only: all polygons must have the same stamp as the collection.
  //! Collections only: transform polygons in a different frame to the frame of the collection.
  //! Requires the TF buffer in tf.
  bool transform_to_header_frame {false};
  //! Collections only: transform polygons with a different stamp to the stamp of the collection.
  //! Requires the TF buffer in tf.
  bool transform_to_header_stamp {false};
  tf2_ros::BufferInterface* tf {nullptr};  //!< TF buffer.
  tf2::Duration tf_timeout {0};  //!< How long to wait for the transform to be available in the TF buffer.
};

/**
 * \brief Description of why the conversion from 3D to 2D failed.
 */
struct ConversionError {
  bool z_diff_too_large {false};  //!< The polygon (or the collection) is not flat enough.
  bool frame_mismatch {false};  //!< A polygon has a different frame than the collection.
  bool stamp_mismatch {false};  //!< A polygon has a different stamp than the collection.
  bool tf_error {false};  //!< The transform to the collection header could not be obtained.
  std::string error_message;  //!< Human-readable description of the error.
};

/**
 * Convert the given 2D message to a 3D message.
 * \tparam Type2D The 2D type.
 * \tparam Type3D The 3D type.
 * \param[in] msg_2d The 2D message.
 * \return The converted 3D message. The Z-coordinate of all vertices is set to 0 and the double coordinates are
 *         saturated when converted to the single precision used by the 3D messages.
 * \note Only the explicit specializations declared in this header are implemented.
 */
template<typename Type2D, typename Type3D>
Type3D convert2DTo3D(const Type2D& msg_2d);

/**
 * Convert the given 3D message to a 2D message.
 * The Z-coordinates of the 3D vertices are dropped, but they can be optionally checked by the flatness checks
 * configured in \p options.
 * \tparam Type3D The 3D type.
 * \tparam Type2D The 2D type.
 * \param[in] msg_3d The 3D message.
 * \param[in] options Conversion options.
 * \return The converted 2D message or struct describing the error.
 * \note Only the explicit specializations declared in this header are implemented.
 */
template<typename Type3D, typename Type2D>
cras::expected<Type2D, ConversionError> convert3DTo2D(const Type3D& msg_3d, const ConversionOptions& options);

/**
 * Convert the given 3D message to a 2D message.
 * \tparam Type3D The 3D type.
 * \tparam Type2D The 2D type.
 * \param[in] msg_3d The 3D message.
 * \return The converted 2D message.
 * \throws std::runtime_error If the conversion fails (should not happen).
 */
template<typename Type3D, typename Type2D>
Type2D convert3DTo2D(const Type3D& msg_3d) {
  const auto maybe_result = convert3DTo2D<Type3D, Type2D>(msg_3d, ConversionOptions());
  if (!maybe_result.has_value()) {
    throw std::runtime_error(maybe_result.error().error_message);
  }
  return maybe_result.value();
}

template<>
cras::expected<polygon_msgs::msg::Polygon2DStamped, ConversionError> convert3DTo2D(
    const geometry_msgs::msg::PolygonStamped& msg_3d, const ConversionOptions& options);

template<>
cras::expected<polygon_msgs::msg::Polygon2DStamped, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::Polygon64Stamped& msg_3d, const ConversionOptions& options);

template<>
cras::expected<polygon_msgs::msg::Polygon2DStamped, ConversionError> convert3DTo2D(
    const geometry_msgs::msg::PolygonInstanceStamped& msg_3d, const ConversionOptions& options);

template<>
cras::expected<polygon_msgs::msg::Polygon2DStamped, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::Polygon64InstanceStamped& msg_3d, const ConversionOptions& options);

template<>
cras::expected<polygon_msgs::msg::Polygon2DCollection, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::PolygonCollection& msg_3d, const ConversionOptions& options);

template<>
cras::expected<polygon_msgs::msg::Polygon2DCollection, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::Polygon64Collection& msg_3d, const ConversionOptions& options);

template<>
cras::expected<polygon_msgs::msg::Polygon2DCollection, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::PolygonInstanceCollection& msg_3d, const ConversionOptions& options);

template<>
cras::expected<polygon_msgs::msg::Polygon2DCollection, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::Polygon64InstanceCollection& msg_3d, const ConversionOptions& options);

template<>
cras::expected<polygon_msgs::msg::Polygon2DCollection, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::PolygonInstanceStampedCollection& msg_3d, const ConversionOptions& options);

template<>
cras::expected<polygon_msgs::msg::Polygon2DCollection, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::Polygon64InstanceStampedCollection& msg_3d, const ConversionOptions& options);

template<>
cras::expected<polygon_msgs::msg::Polygon2DCollection, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::PolygonStampedCollection& msg_3d, const ConversionOptions& options);

template<>
cras::expected<polygon_msgs::msg::Polygon2DCollection, ConversionError> convert3DTo2D(
    const cras_geo_msgs::msg::Polygon64StampedCollection& msg_3d, const ConversionOptions& options);

template<>
geometry_msgs::msg::PolygonStamped convert2DTo3D(const polygon_msgs::msg::Polygon2DStamped& msg_2d);

template<>
cras_geo_msgs::msg::Polygon64Stamped convert2DTo3D(const polygon_msgs::msg::Polygon2DStamped& msg_2d);

template<>
cras_geo_msgs::msg::PolygonCollection convert2DTo3D(const polygon_msgs::msg::Polygon2DCollection& msg_2d);

template<>
cras_geo_msgs::msg::Polygon64Collection convert2DTo3D(const polygon_msgs::msg::Polygon2DCollection& msg_2d);

template<>
cras_geo_msgs::msg::PolygonStampedCollection convert2DTo3D(const polygon_msgs::msg::Polygon2DCollection& msg_2d);

template<>
cras_geo_msgs::msg::Polygon64StampedCollection convert2DTo3D(const polygon_msgs::msg::Polygon2DCollection& msg_2d);

}  // namespace cras_geo_tools
