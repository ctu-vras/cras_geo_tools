// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <memory>
#include <string>

#include <cras_cpp_common/type_utils.hpp>
#include <cras_geo_msgs/msg/polygon64_stamped.hpp>
#include <cras_geo_msgs/msg/polygon64_stamped_collection.hpp>
#include <cras_geo_tools/convert_ecef.hpp>
#include <cras_geo_tools/tf2_polygon_msgs.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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

template<typename Spherical, typename Cartesian>
class SphericalToCartesian : public rclcpp::Node {
public:
  explicit SphericalToCartesian(const rclcpp::NodeOptions& options) : Node("spherical_to_cartesian", options) {
    ecef_frame_ = declare_parameter("ecef_frame", "ecef");
    target_frame_ = declare_parameter("target_frame", "map");
    const auto latch = declare_parameter("latch", false);

    tf_timeout_ = tf2::durationFromSec(declare_parameter("tf_timeout", 0.1));

    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_);

    auto qos = rclcpp::QoS(10).reliable();
    if (latch) {
      qos = qos.transient_local();
    }

    pub_ = create_publisher<Cartesian>("cartesian", qos);

    sub_ = create_subscription<Spherical>(
      "spherical", qos,
        [this](const typename Spherical::ConstSharedPtr& msg) {
          const auto ecef_pt = llhtoEcef<Spherical, Cartesian>(*msg);
          try {
            pub_->publish(tf_->transform(ecef_pt, target_frame_, tf_timeout_));
          } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(), "Transform error: %s", ex.what());
          }
        });
    RCLCPP_INFO(
      get_logger(), "Initialized SphericalToCartesian<%s, %s>",
      cras::getTypeName<Spherical>().c_str(), cras::getTypeName<Cartesian>().c_str());
  }

protected:
  std::string ecef_frame_;
  std::string target_frame_;
  tf2_ros::Buffer::SharedPtr tf_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2::Duration tf_timeout_;
  rclcpp::Subscription<Spherical>::SharedPtr sub_;
  rclcpp::Publisher<Cartesian>::SharedPtr pub_;
};

struct SphericalToCartesianPointStamped
    : SphericalToCartesian<geographic_msgs::msg::GeoPointStamped, geometry_msgs::msg::PointStamped> {
  explicit SphericalToCartesianPointStamped(const rclcpp::NodeOptions& options) : SphericalToCartesian(options) {}
};

struct SphericalToCartesianPoseStamped
    : SphericalToCartesian<geographic_msgs::msg::GeoPoseStamped, geometry_msgs::msg::PoseStamped> {
  explicit SphericalToCartesianPoseStamped(const rclcpp::NodeOptions& options) : SphericalToCartesian(options) {}
};

struct SphericalToCartesianPolygon
    : SphericalToCartesian<cras_geo_msgs::msg::GeoPolygon2D, cras_geo_msgs::msg::Polygon64Stamped> {
  explicit SphericalToCartesianPolygon(const rclcpp::NodeOptions& options) : SphericalToCartesian(options) {}
};

struct SphericalToCartesianPolygonCollection
    : SphericalToCartesian<cras_geo_msgs::msg::GeoPolygon2DCollection, cras_geo_msgs::msg::Polygon64StampedCollection> {
  explicit SphericalToCartesianPolygonCollection(const rclcpp::NodeOptions& options) : SphericalToCartesian(options) {}
};

}  // namespace cras_geo_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cras_geo_tools::SphericalToCartesianPointStamped)
RCLCPP_COMPONENTS_REGISTER_NODE(cras_geo_tools::SphericalToCartesianPoseStamped)
RCLCPP_COMPONENTS_REGISTER_NODE(cras_geo_tools::SphericalToCartesianPolygon)
RCLCPP_COMPONENTS_REGISTER_NODE(cras_geo_tools::SphericalToCartesianPolygonCollection)
