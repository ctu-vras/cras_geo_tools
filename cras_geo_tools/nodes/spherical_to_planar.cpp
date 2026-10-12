// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <memory>
#include <string>

#include <cras_cpp_common/type_utils.hpp>
#include <cras_geo_msgs/msg/polygon64_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_stamped.hpp>
#include <cras_geo_tools/convert_2d_3d.hpp>
#include <cras_geo_tools/convert_ecef.hpp>
#include <cras_geo_tools/tf2_polygon_msgs.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace cras_geo_tools {

template<typename Spherical, typename Cartesian, typename Planar>
class SphericalToPlanar : public rclcpp::Node {
public:
  explicit SphericalToPlanar(const rclcpp::NodeOptions& options) : Node("spherical_to_planar", options) {
    ecef_frame_ = declare_parameter("ecef_frame", "ecef");
    planar_frame_ = declare_parameter("planar_frame", "map");
    const auto latch = declare_parameter("latch", false);

    tf_timeout_ = tf2::durationFromSec(declare_parameter("tf_timeout", 0.1));

    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_);

    auto qos = rclcpp::QoS(10).reliable();
    if (latch) {
      qos = qos.transient_local();
    }

    pub_ = create_publisher<Planar>("planar", qos);

    sub_ = create_subscription<Spherical>(
      "spherical", qos,
        [this](const typename Spherical::ConstSharedPtr& msg) {
          const auto msg_3d_ecef = llhtoEcef<Spherical, Cartesian>(*msg);
          auto msg_3d_ecef2 = msg_3d_ecef;
          msg_3d_ecef2.header.frame_id = ecef_frame_;
          try {
            const auto msg_3d_planar = tf_->transform(msg_3d_ecef2, planar_frame_, tf_timeout_);
            const auto maybe_msg_2d = convert3DTo2D<Cartesian, Planar>(msg_3d_planar, {});
            if (maybe_msg_2d.has_value()) {
              pub_->publish(maybe_msg_2d.value());
            } else {
              RCLCPP_ERROR(get_logger(), "%s", maybe_msg_2d.error().error_message.c_str());
            }
          } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(), "Transform error: %s", ex.what());
          }
        });
    RCLCPP_INFO(
      get_logger(), "Initialized SphericalToPlanar<%s, %s, %s>",
      cras::getTypeName<Spherical>().c_str(), cras::getTypeName<Cartesian>().c_str(),
      cras::getTypeName<Planar>().c_str());
  }

protected:
  std::string ecef_frame_;
  std::string planar_frame_;
  tf2_ros::Buffer::SharedPtr tf_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2::Duration tf_timeout_;
  rclcpp::Subscription<Spherical>::SharedPtr sub_;
  rclcpp::Publisher<Planar>::SharedPtr pub_;
};

struct SphericalToPlanarPolygonStamped
    : SphericalToPlanar<
        cras_geo_msgs::msg::GeoPolygon2D, cras_geo_msgs::msg::Polygon64Stamped, polygon_msgs::msg::Polygon2DStamped> {
  explicit SphericalToPlanarPolygonStamped(const rclcpp::NodeOptions& options) : SphericalToPlanar(options) {}
};

struct SphericalToPlanarPolygonCollection
    : SphericalToPlanar<
        cras_geo_msgs::msg::GeoPolygon2DCollection, cras_geo_msgs::msg::Polygon64Collection,
        polygon_msgs::msg::Polygon2DCollection> {
  explicit SphericalToPlanarPolygonCollection(const rclcpp::NodeOptions& options) : SphericalToPlanar(options) {}
};

}  // namespace cras_geo_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cras_geo_tools::SphericalToPlanarPolygonStamped)
RCLCPP_COMPONENTS_REGISTER_NODE(cras_geo_tools::SphericalToPlanarPolygonCollection)
