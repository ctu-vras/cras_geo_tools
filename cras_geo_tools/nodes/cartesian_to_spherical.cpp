// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <memory>
#include <string>

#include <cras_cpp_common/type_utils.hpp>
#include <cras_geo_msgs/msg/geo_polygon2_d.hpp>
#include <cras_geo_msgs/msg/geo_polygon2_d_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_collection.hpp>
#include <cras_geo_msgs/msg/polygon64_stamped_collection.hpp>
#include <cras_geo_tools/convert_ecef.hpp>
#include <cras_geo_tools/tf2_polygon_msgs.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/message_filter.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace cras_geo_tools {

template<typename Cartesian, typename Spherical>
class CartesianToSpherical : public rclcpp::Node {
public:
  explicit CartesianToSpherical(const rclcpp::NodeOptions& options) : Node("cartesian_to_spherical", options) {
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

#ifndef MESSAGE_FILTERS_USE_RMW_QOS
#ifndef MESSAGE_FILTERS_DONT_HAVE_NODE_INTERFACES
    sub_.subscribe(*this, "cartesian", qos);
#else
    sub_.subscribe(this, "cartesian", qos);
#endif
#else
    sub_.subscribe(this, "cartesian", qos.get_rmw_qos_profile());
#endif

    tf_filter_ = std::make_unique<tf2_ros::MessageFilter<Cartesian>>(
      sub_, *tf_, ecef_frame_, 10, get_node_logging_interface(), get_node_clock_interface(), tf_timeout_);
    tf_filter_->registerCallback(
        [this](const typename Cartesian::ConstSharedPtr& src_frame_msg) {
          try {
            const auto ecef_msg = tf_->transform(*src_frame_msg, ecef_frame_);
            pub_->publish(ecefToLlh<Cartesian, Spherical>(ecef_msg));
          } catch (const tf2::TransformException& e) {
            RCLCPP_WARN(get_logger(), "Transform error: %s", e.what());
          }
        });
    RCLCPP_INFO(
      get_logger(), "Initialized CartesianToSpherical<%s, %s>",
      cras::getTypeName<Cartesian>().c_str(), cras::getTypeName<Spherical>().c_str());
  }

protected:
  std::string ecef_frame_;
  tf2_ros::Buffer::SharedPtr tf_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::MessageFilter<Cartesian>> tf_filter_;
  tf2::Duration tf_timeout_ {tf2::durationFromSec(0.1)};
  message_filters::Subscriber<Cartesian> sub_;
  typename rclcpp::Publisher<Spherical>::SharedPtr pub_;
};

struct CartesianToSphericalPointStamped
    : public CartesianToSpherical<geometry_msgs::msg::PointStamped, geographic_msgs::msg::GeoPointStamped> {
  explicit CartesianToSphericalPointStamped(const rclcpp::NodeOptions& options) : CartesianToSpherical(options) {}
};

struct CartesianToSphericalPoseStamped
    : public CartesianToSpherical<geometry_msgs::msg::PoseStamped, geographic_msgs::msg::GeoPoseStamped> {
  explicit CartesianToSphericalPoseStamped(const rclcpp::NodeOptions& options) : CartesianToSpherical(options) {}
};

struct CartesianToSphericalPolygon
    : public CartesianToSpherical<cras_geo_msgs::msg::Polygon64Stamped, cras_geo_msgs::msg::GeoPolygon2D> {
  explicit CartesianToSphericalPolygon(const rclcpp::NodeOptions& options)
      : CartesianToSpherical(options) {}
};

struct CartesianToSphericalPolygonCollection
    : public CartesianToSpherical<
        cras_geo_msgs::msg::Polygon64Collection, cras_geo_msgs::msg::GeoPolygon2DCollection> {
  explicit CartesianToSphericalPolygonCollection(const rclcpp::NodeOptions& options)
      : CartesianToSpherical(options) {}
};

struct CartesianToSphericalPolygonStampedCollection
    : public CartesianToSpherical<
        cras_geo_msgs::msg::Polygon64StampedCollection, cras_geo_msgs::msg::GeoPolygon2DCollection> {
  explicit CartesianToSphericalPolygonStampedCollection(const rclcpp::NodeOptions& options)
      : CartesianToSpherical(options) {}
};

}  // namespace cras_geo_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cras_geo_tools::CartesianToSphericalPointStamped)
RCLCPP_COMPONENTS_REGISTER_NODE(cras_geo_tools::CartesianToSphericalPoseStamped)
RCLCPP_COMPONENTS_REGISTER_NODE(cras_geo_tools::CartesianToSphericalPolygon)
RCLCPP_COMPONENTS_REGISTER_NODE(cras_geo_tools::CartesianToSphericalPolygonCollection)
RCLCPP_COMPONENTS_REGISTER_NODE(cras_geo_tools::CartesianToSphericalPolygonStampedCollection)
