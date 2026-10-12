<!-- SPDX-License-Identifier: BSD-3-Clause -->
<!-- SPDX-FileCopyrightText: Czech Technical University in Prague -->

# cras_geo_tools

![ROS 2 compatible](https://img.shields.io/badge/ROS-2-blue) ![Jazzy](https://img.shields.io/badge/jazzy-green) ![Kilted](https://img.shields.io/badge/kilted-green) ![Lyrical](https://img.shields.io/badge/lyrical-green) ![Rolling](https://img.shields.io/badge/rolling-green)

Tools for handling geographic coordinates in ROS 2, developed by the [Center for Robotics and Autonomous
Systems (CRAS)](https://robotics.fel.cvut.cz/cras/) at the [Faculty of Electrical Engineering](https://fel.cvut.cz/)
of the [Czech Technical University in Prague](https://www.cvut.cz).

The package converts between spherical coordinates (latitude, longitude and altitude on the WGS-84 ellipsoid) and
Cartesian coordinates (the global ECEF frame and local planar frames). Next to plain points and poses it focuses on
**polygon** messages, which are the natural way to represent mapped areas, no-go zones, planned operation areas etc.

It is split into small, dependency-light C++ libraries that do all the actual work, and on top of them there are thin
`rclcpp_components` nodes that wrap the libraries for convenient use inside a ROS graph (with TF2 lookups handled
automatically). The nodes are complemented by two small Python scripts that import and export the areas as GPX files,
and by launch files that visualize the polygons in RViz.

## Table of Contents

* [Coordinate conventions](#coordinate-conventions)
* [C++ API](#c-api)
  * [ECEF ↔ spherical conversions](#ecef--spherical-conversions)
  * [3D ↔ 2D polygon conversions](#3d--2d-polygon-conversions)
  * [TF2 support for polygon messages](#tf2-support-for-polygon-messages)
* [ROS 2 nodes](#ros-2-nodes)
  * [Available nodes and components](#available-nodes-and-components)
  * [Parameters](#parameters)
  * [Running the nodes](#running-the-nodes)
  * [GPX import and export](#gpx-import-and-export)
  * [Launch files](#launch-files)
* [Building and testing](#building-and-testing)
* [ROS versions support](#ros-versions-support)
* [License](#license)

## Coordinate conventions

The package works with three "domains" of coordinates:

| Domain | Message types | Coordinates | Frame ID written to headers |
|---|---|---|---|
| Spherical (geodetic) | `geographic_msgs/GeoPoint`, `GeoPointStamped`, `GeoPose`, `GeoPoseStamped`, `cras_geo_msgs/GeoPolygon2D`, `GeoPolygon2DCollection` | latitude and longitude in **degrees**, altitude in metres above the WGS-84 ellipsoid | `wgs84` |
| Global Cartesian | `geometry_msgs/Point`, `Point32`, `PointStamped`, `Pose`, `PoseStamped`, `Polygon`, `PolygonStamped`, `PolygonInstance`, `PolygonInstanceStamped`, `cras_geo_msgs/PolygonCollection`, `PolygonStampedCollection`, `PolygonInstanceCollection`, `PolygonInstanceStampedCollection`, `Polygon64`, `Polygon64Stamped`, `Polygon64Instance`, `Polygon64InstanceStamped`, `Polygon64Collection`, `Polygon64StampedCollection`, `Polygon64InstanceCollection`, `Polygon64InstanceStampedCollection` | ECEF metres | `ecef` |
| Local planar (2D) | `polygon_msgs/Point2D`, `Polygon2D`, `Polygon2DStamped`, `Polygon2DCollection`, `ComplexPolygon2D`, `ComplexPolygon2DStamped`, `ComplexPolygon2DCollection` | metres in an arbitrary local frame | taken from the TF transform |

A few properties worth knowing when converting:

* Vertices of polygons are converted **individually**, without any clipping or re-meshing. A polygon that is planar in
  ECEF is therefore usually not planar in the spherical domain anymore (and vice versa), and polygons that cross the
  antimeridian or contain a pole keep their vertices as they are.
* `Polygon`-family messages use single-precision (`float32`) vertices. Their ECEF resolution is in units of decimetres,
  which limits the accuracy of the resulting latitude/longitude. Double-precision types are used wherever possible - the
  `cras_geo_msgs/Polygon64*` family holds the vertices as `geometry_msgs/Point` (`float64`) and is the one used by the
  geodetic conversions.
* Only the position is converted for poses - the orientation is passed through unchanged. It is **not** rotated
  between the ECEF frame and any local-level (ENU-like) frame.
* Instance IDs and per-polygon headers are dropped when the destination type has no field for them.

## C++ API

The headers live in `include/cras_geo_tools/` and are installed with the package. Each library is exported as a CMake
target, so a downstream package only needs

```cmake
find_package(cras_geo_tools REQUIRED)
target_link_libraries(my_target cras_geo_tools::convert_ecef)
```

Individual targets (all prefixed with `cras_geo_tools::`):

| Target | Header | Content |
|---|---|---|
| `convert_ecef` | `cras_geo_tools/convert_ecef.hpp` | spherical ↔ ECEF conversions |
| `convert_2d_3d` | `cras_geo_tools/convert_2d_3d.hpp` | 2D ↔ 3D polygon conversions |
| `tf2_polygon_msgs` | `cras_geo_tools/tf2_polygon_msgs.hpp` | `tf2::doTransform()` for polygon messages |
| `cras_geo_tools` | – | convenience target linking all of the above |

### ECEF ↔ spherical conversions

Two function templates do all the work; the conversion is selected by the template arguments:

```cpp
#include <cras_geo_tools/convert_ecef.hpp>

// ECEF metres → latitude/longitude/altitude
const auto llh = cras_geo_tools::ecefToLlh<
    geometry_msgs::msg::PointStamped, geographic_msgs::msg::GeoPointStamped>(ecef_point);

// and back (stamp is kept, frame_id is set to "ecef")
const auto ecef = cras_geo_tools::llhtoEcef<
    geographic_msgs::msg::GeoPointStamped, geometry_msgs::msg::PointStamped>(llh);
```

The conversions are lossless. The underlying ellipsoid arithmetic is provided by the
[`geodesy`](http://wiki.ros.org/geodesy) package.

Variants for `geometry_msgs/Polygon*` are not implemented - the points carry only 32-bit coordinates, which is not
sufficient for ECEF coordinates.

### 3D ↔ 2D polygon conversions

`convert2DTo3D()` is trivial (the Z-coordinates are set to 0), while `convert3DTo2D()` drops the Z-coordinates and can
additionally *verify* that dropping them is legitimate - which is important e.g. when a 3D map of a multi-storey
building is flattened to a single 2D plan.

```cpp
#include <cras_geo_tools/convert_2d_3d.hpp>

// Simple version: throws std::runtime_error if the conversion is not possible.
const auto polygon_2d = cras_geo_tools::convert3DTo2D<
    geometry_msgs::msg::PolygonStamped, polygon_msgs::msg::Polygon2DStamped>(polygon_3d);

// Full version with options: returns cras::expected<Polygon2DCollection, ConversionError>.
cras_geo_tools::ConversionOptions options {.z_diff_threshold = 0.1, .tf = tf_buffer, .transform_to_header_frame = true};
const auto maybe_collection = cras_geo_tools::convert3DTo2D<
    cras_geo_msgs::msg::PolygonStampedCollection, polygon_msgs::msg::Polygon2DCollection>(collection_3d, options);
if (!maybe_collection) {
  RCLCPP_ERROR(get_logger(), "Cannot flatten the polygons: %s", maybe_collection.error().error_message.c_str());
}
```

A failed conversion returns a `ConversionError` struct with flags and a human-readable `error_message`.

### TF2 support for polygon messages

The plain ROS 2 polygon messages have no `tf2` traits, so `tf2::transform()` cannot be used with them out of the box.
Including `cras_geo_tools/tf2_polygon_msgs.hpp` (and linking `cras_geo_tools::tf2_polygon_msgs`) provides
`tf2::doTransform()` specializations for:

* `polygon_msgs`: `Point2D`, `Polygon2D`, `Polygon2DStamped`, `Polygon2DCollection`, `ComplexPolygon2D`,
  `ComplexPolygon2DStamped`, `ComplexPolygon2DCollection`,
* `geometry_msgs`: `PolygonInstance`, `PolygonInstanceStamped` (plain `Polygon`/`PolygonStamped` are covered by
  `tf2_geometry_msgs`),
* `cras_geo_msgs`: `PolygonCollection`, `PolygonStampedCollection`, `PolygonInstanceCollection`,
  `PolygonInstanceStampedCollection`, and all `Polygon64*` types.

`tf2::getTimestamp()` and `tf2::getFrameId()` are specialized for the stamped messages and collections as well, so they
can be used e.g. inside `tf2_ros::MessageFilter`.

```cpp
// Do not let auto-include optimizations remove this include. Most IDEs mark it as unused, but it is used.
#include <cras_geo_tools/tf2_polygon_msgs.hpp>

const auto transformed = tf_buffer.transform(msg, target_frame, timeout);
```

The 2D types are transformed as if their Z-coordinate was 0, and only the resulting X and Y are kept, so a transform
with a vertical component or a rotation around X/Y silently loses part of the information. Stamps and frame IDs of the
output messages are taken from the used transform, colors are always copied.

## ROS 2 nodes

Each conversion node subscribes to one topic, converts the message and republishes it on another topic. All of them are
implemented as `rclcpp_components` and also installed as stand-alone executables. They use a reliable QoS profile with
a queue depth of 10 and perform the necessary TF2 lookups themselves; if the transform is not available within
`tf_timeout`, the message is dropped and a warning is logged (the node keeps running).

The data flow of the individual directions is always

* `spherical_to_cartesian`: `llhtoEcef()` → TF2 transform to `target_frame`,
* `cartesian_to_spherical`: TF2 transform to `ecef_frame` (done by a `tf2_ros::MessageFilter`, so messages with no
  known transform are dropped) → `ecefToLlh()`,
* `spherical_to_planar`: `llhtoEcef()` → TF2 transform to `planar_frame` → `convert3DTo2D()`,
* `planar_to_spherical`: `convert2DTo3D()` → TF2 transform to `ecef_frame` → `ecefToLlh()`.

### Available nodes and components

| Executable | Input topic | Input type | Output topic | Output type |
|---|---|---|---|---|
| `spherical_to_cartesian_point_stamped` | `spherical` | `geographic_msgs/GeoPointStamped` | `cartesian` | `geometry_msgs/PointStamped` |
| `spherical_to_cartesian_pose_stamped` | `spherical` | `geographic_msgs/GeoPoseStamped` | `cartesian` | `geometry_msgs/PoseStamped` |
| `spherical_to_cartesian_polygon` | `spherical` | `cras_geo_msgs/GeoPolygon2D` | `cartesian` | `cras_geo_msgs/Polygon64Stamped` |
| `spherical_to_cartesian_polygon_collection` | `spherical` | `cras_geo_msgs/GeoPolygon2DCollection` | `cartesian` | `cras_geo_msgs/Polygon64StampedCollection` |
| `cartesian_to_spherical_point_stamped` | `cartesian` | `geometry_msgs/PointStamped` | `spherical` | `geographic_msgs/GeoPointStamped` |
| `cartesian_to_spherical_pose_stamped` | `cartesian` | `geometry_msgs/PoseStamped` | `spherical` | `geographic_msgs/GeoPoseStamped` |
| `cartesian_to_spherical_polygon` | `cartesian` | `cras_geo_msgs/Polygon64Stamped` | `spherical` | `cras_geo_msgs/GeoPolygon2D` |
| `cartesian_to_spherical_polygon_collection` | `cartesian` | `cras_geo_msgs/Polygon64Collection` | `spherical` | `cras_geo_msgs/GeoPolygon2DCollection` |
| `spherical_to_planar_polygon_stamped` | `spherical` | `cras_geo_msgs/GeoPolygon2D` | `planar` | `polygon_msgs/Polygon2DStamped` |
| `spherical_to_planar_polygon_collection` | `spherical` | `cras_geo_msgs/GeoPolygon2DCollection` | `planar` | `polygon_msgs/Polygon2DCollection` |
| `planar_to_spherical_polygon_stamped` | `planar` | `polygon_msgs/Polygon2DStamped` | `spherical` | `cras_geo_msgs/GeoPolygon2D` |
| `planar_to_spherical_polygon_collection` | `planar` | `polygon_msgs/Polygon2DCollection` | `spherical` | `cras_geo_msgs/GeoPolygon2DCollection` |

The executable `xxx_yyy` is always produced from a component class of a matching name - `cras_geo_tools::XxxYyy`
(e.g. `cras_geo_tools::SphericalToCartesianPointStamped`) living in the library named after the direction
(`spherical_to_cartesian` and so on). Only
`cras_geo_tools::CartesianToSphericalPolygonStampedCollection` has no dedicated executable and can be used only through
the component container.

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `ecef_frame` | `string` | `ecef` | TF2 name of the frame holding the ECEF coordinates the messages are transformed into before the geodetic conversion. |
| `target_frame` | `string` | `map` | Frame of the published Cartesian message. |
| `planar_frame` | `string` | `map` | Frame of the published 2D message. |
| `tf_timeout` | `double` [s] | `0.1` | How long to wait for the TF2 transform to become available. |
| `latch` | `bool`| `false` | Whether to configure publisher and subscriber as transient local. |

> **Note:** The frame IDs written into the converted messages are fixed - `wgs84` for the spherical messages and
> `ecef` for the ECEF ones - regardless of the parameters. If your TF tree uses a different name for the ECEF frame,
> set `ecef_frame` (and `target_frame`/`planar_frame`) accordingly and make sure the corresponding transform is
> published.

### Running the nodes

Stand-alone:

```console
$ ros2 run cras_geo_tools spherical_to_cartesian_polygon --ros-args \
    -r spherical:=/mission/geo_no_go_zone -r cartesian:=/mission/no_go_zone -p target_frame:=map
```

As a component (allows running several conversions in one process and in one callback group):

```console
$ ros2 run rclcpp_components component_container
$ ros2 component load /ComponentManager cras_geo_tools cras_geo_tools::SphericalToCartesianPointStamped \
    -r spherical:=/ground_truth/geo_pose -p target_frame:=map
```

All instances of one node type share the same default node name (`spherical_to_cartesian`,
`cartesian_to_spherical`, `spherical_to_planar` or `planar_to_spherical`), which can be changed by the usual node-name
remapping (`-r __node:=...` when running stand-alone, `-n` when loading a component).

### GPX import and export

Two Python scripts exchange the areas with the GPX file format:

| Script | Direction | Topic | Message type |
|---|---|---|---|
| `gpx_to_ros` | GPX file → ROS | `polygons_wgs84` (published) | `cras_geo_msgs/GeoPolygon2DCollection` |
| `ros_to_gpx` | ROS → GPX file | `polygons_wgs84` (subscribed) | `cras_geo_msgs/GeoPolygon2DCollection` |

The GPX file is passed either as the only positional argument or via the `gpx_file` parameter; if neither is given, the
node logs a fatal error and exits. The topic is published with a transient-local QoS and the messages use the `wgs84`
frame, so a late-joining subscriber still gets the areas and the scripts can be combined with the launch files below.

* `gpx_to_ros` parses the file when it starts and publishes the collection once (the stamp is the time of parsing); the
  node then only keeps serving the latched message. Every GPX *track* becomes one `GeoPolygon2D`, with the points of all
  its segments appended in order and `elevation` mapped to `altitude`.
* `ros_to_gpx` (over)writes the whole file whenever a collection arrives: every polygon becomes one track with a single
  segment, and the GPX `time` is taken from the message stamp.

```console
$ ros2 run cras_geo_tools gpx_to_ros ~/areas/no_go_zones.gpx
$ ros2 run cras_geo_tools ros_to_gpx ~/areas/exported.gpx
```

The conversion is not lossless - GPX knows neither polygon holes nor per-polygon colors, and `gpx_to_ros` reads only
the tracks (waypoints and routes are ignored) - so the scripts are meant for importing manually drawn areas and for
exporting them for inspection in external tools.

### Launch files

| Launch file | Runs | Arguments |
|---|---|---|
| `rviz_geo_to_polygons.launch` | `spherical_to_planar_polygon_collection`, remapped `spherical` → `polygons_wgs84` and `planar` → `polygons` | `ecef_frame` (`ecef`), `planar_frame` (`map`), `latch` (`true`), `test` (`false`) |
| `rviz_polygons_to_geo.launch` | `planar_to_spherical_polygon_collection`, remapped `planar` → `polygons` and `spherical` → `polygons_wgs84` | `ecef_frame` (`ecef`), `latch` (`true`), `test` (`false`) |

```console
$ ros2 launch cras_geo_tools rviz_geo_to_polygons.launch
```

Both files publish the converted polygons in a latched way under the topic names commonly used by the visualization and
manipulation tools (e.g. `rviz_zone_of_interest_tool`). Setting `test:=true` additionally starts two
`static_transform_publisher` instances (a fixed `ecef` → `planar_frame` transform and `planar_frame` → `base_link`) and
RViz with the `polygons.rviz` configuration from `rviz_zone_of_interest_tool`, which is convenient for testing the
conversion without a robot. `rviz_zone_of_interest_tool` is not a dependency of this package, so it has to be installed
separately to use that mode.

## Building and testing

The package is a regular `ament_cmake` package:

```console
$ rosdep install --from-paths src --ignore-src -r
$ colcon build --packages-up-to cras_geo_tools
```

## ROS versions support

| ROS | Support |
|-----|---------|
| Humble | Not supported |
| Jazzy | Until EOL |
| Kilted | Until EOL |
| Lyrical | Until EOL |
| Rolling | Best effort |
