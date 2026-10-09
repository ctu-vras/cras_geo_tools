# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.20.2)
project(cras_geo_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GeoPolygon2D.msg"
  "msg/GeoPolygon2DCollection.msg"
  "msg/Polygon64.msg"
  "msg/Polygon64Collection.msg"
  "msg/Polygon64Instance.msg"
  "msg/Polygon64InstanceCollection.msg"
  "msg/Polygon64InstanceStamped.msg"
  "msg/Polygon64InstanceStampedCollection.msg"
  "msg/Polygon64Stamped.msg"
  "msg/Polygon64StampedCollection.msg"
  "msg/PolygonCollection.msg"
  "msg/PolygonInstanceCollection.msg"
  "msg/PolygonInstanceStampedCollection.msg"
  "msg/PolygonStampedCollection.msg"
  DEPENDENCIES
    geometry_msgs
    geographic_msgs
    std_msgs
)

if(BUILD_TESTING)
  find_package(cras_lint REQUIRED)
  cras_lint_common()
endif()

ament_package()
