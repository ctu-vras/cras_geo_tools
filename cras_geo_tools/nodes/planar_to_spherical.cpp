// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <memory>
#include <string>

#include <cras_cpp_common/type_utils.hpp>
#include <cras_geo_msgs/msg/geo_polygon2_d.hpp>
#include <cras_geo_msgs/msg/geo_polygon2_d_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_stamped_collection.hpp>
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

template<typename Planar, typename Cartesian, typename Spherical>
class PlanarToSpherical : public rclcpp::Node {
public:
  explicit PlanarToSpherical(const rclcpp::NodeOptions& options) : Node("planar_to_spherical", options) {
    ecef_frame_ = declare_parameter("ecef_frame", "ecef");
    const auto latch = declare_parameter("latch", false);

    tf_timeout_ = tf2::durationFromSec(declare_parameter("tf_timeout", 0.1));

    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_);

    auto qos = rclcpp::QoS(10).reliable();
    if (latch) {
      qos = qos.transient_local();
    }

    pub_ = create_publisher<Spherical>("spherical", qos);

    sub_ = create_subscription<Planar>("planar", qos,
        [this](const typename Planar::ConstSharedPtr& planar_msg) {
          try {
            const auto cartesian_msg = convert2DTo3D<Planar, Cartesian>(*planar_msg);
            const auto ecef_msg = tf_->transform(cartesian_msg, ecef_frame_, tf_timeout_);
            pub_->publish(ecefToLlh<Cartesian, Spherical>(ecef_msg));
          } catch (const tf2::TransformException& e) {
            RCLCPP_WARN(get_logger(), "Transform error: %s", e.what());
          }
        });
    RCLCPP_INFO(
      get_logger(), "Initialized PlanarToSpherical<%s, %s, %s>",
      cras::getTypeName<Planar>().c_str(), cras::getTypeName<Cartesian>().c_str(),
      cras::getTypeName<Spherical>().c_str());
  }

protected:
  std::string ecef_frame_;
  tf2_ros::Buffer::SharedPtr tf_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2::Duration tf_timeout_ {tf2::durationFromSec(0.1)};
  typename rclcpp::Subscription<Planar>::SharedPtr sub_;
  typename rclcpp::Publisher<Spherical>::SharedPtr pub_;
};

struct PlanarToSphericalPolygonStamped
    : public PlanarToSpherical<
        polygon_msgs::msg::Polygon2DStamped, cras_geo_msgs::msg::Polygon64Stamped, cras_geo_msgs::msg::GeoPolygon2D> {
  explicit PlanarToSphericalPolygonStamped(const rclcpp::NodeOptions& options) : PlanarToSpherical(options) {}
};

struct PlanarToSphericalPolygonCollection
    : public PlanarToSpherical<
        polygon_msgs::msg::Polygon2DCollection, cras_geo_msgs::msg::Polygon64Collection,
        cras_geo_msgs::msg::GeoPolygon2DCollection> {
  explicit PlanarToSphericalPolygonCollection(const rclcpp::NodeOptions& options) : PlanarToSpherical(options) {}
};

}  // namespace cras_geo_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cras_geo_tools::PlanarToSphericalPolygonStamped)
RCLCPP_COMPONENTS_REGISTER_NODE(cras_geo_tools::PlanarToSphericalPolygonCollection)
