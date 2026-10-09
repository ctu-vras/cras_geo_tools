<!--
SPDX-License-Identifier: BSD-3-Clause
SPDX-FileCopyrightText: Czech Technical University in Prague
-->

# cras\_geo\_msgs

![ROS 2 compatible](https://img.shields.io/badge/ROS-2-blue) ![Jazzy](https://img.shields.io/badge/jazzy-green) ![Kilted](https://img.shields.io/badge/kilted-green) ![Lyrical](https://img.shields.io/badge/lyrical-green) ![Rolling](https://img.shields.io/badge/rolling-green)

Geographic messages used by CRAS.

This package contains **interface definitions only** – a handful of messages describing polygons that are either
referenced by a TF frame (metric coordinates, typically ECEF) or expressed directly as geographic coordinates on the
WGS 84 ellipsoid. There are no nodes, no libraries and no other executables in this package.

## Conventions

### Vertices

* The first and the last vertex are implicitly connected, exactly like in `geometry_msgs/Polygon`. Do not repeat the
  first vertex at the end of the list.
* Only simple polygons are supported – there is no way to describe holes or nested rings. Use
  `polygon_msgs/ComplexPolygon2D` if you need them.
* Fewer than three vertices (or an empty list) is not a valid polygon – consumers should ignore or reject such input.
* The 3D types that wrap `geometry_msgs/Polygon`, use `geometry_msgs/Point32`, i.e. single precision. In
  frames with a large distance from the origin (ECEF coordinates are of order 10<sup>6</sup> m), the representable
  resolution drops to units of metres. Use the `Polygon64` types for ECEF or UTM-sized coordinates.

### Collections

* `header` belongs to the whole collection. In `PolygonStampedCollection` and `PolygonInstanceStampedCollection` each
  polygon has its own header, which may legitimately differ from the collection one (e.g. polygons observed at
  different times). The collection header then describes the frame and timestamp the data are supposed to be
  interpreted (or transformed) in.
* `colors` is optional. Either leave it empty, or fill it with exactly one color per polygon – `colors[i]` then
  belongs to `polygons[i]`. Any other content is undefined and consumers should treat it as if no colors were
  provided.

### Geographic coordinates

* `GeoPolygon2D` uses [`geographic_msgs/GeoPoint`](https://github.com/ros-geographic-info/geographic_info): latitude
  and longitude in degrees, altitude in metres above the WGS 84 ellipsoid (`NaN` when unspecified, which is the usual
  case for a 2D polygon).
* Geographic coordinates are absolute, so `header.frame_id` carries no geometric information. By convention (also used
  by the converters in `cras_geo_tools`) it is set to `wgs84`, while `header.stamp` keeps the meaning of validity of
  the data.
* There is no `tf2::doTransform()` support for the geographic types – convert them to a metric frame first (see
  [Related packages](#related-packages)).

## Related packages

* [polygon_msgs](https://index.ros.org/p/polygon_msgs/) – the 2D counterpart of these messages.
  `polygon_msgs/Polygon2DCollection` has an identical structure (header, polygons, optional colors), but its points
  are planar `float64` coordinates in a local frame, which is the most convenient representation for planning and
  computation. `cras_geo_msgs` exists for the cases where you need to send the polygons through TF or publish them with
  a global meaning.
* [geographic_msgs](https://github.com/ros-geographic-info/geographic_info) – provides `GeoPoint`, which is reused
  here, and a single `GeoPolygon`. It offers no collections and no per-polygon colors.
* [cras_geo_tools](../cras_geo_tools) – the practical companion to these messages (the sibling
  package in this repository). It provides `tf2::doTransform()` specializations for all four collection types,
  conversions between the planar, metric 3D (ECEF) and geographic representations (the `convert_2d_3d` and
  `convert_ecef` libraries), and ready-made components such as
  `cras_geo_tools::PlanarToSphericalPolygonCollection` (subscribes `planar` to `polygon_msgs/Polygon2DCollection`,
  publishes `spherical` as `cras_geo_msgs/GeoPolygon2DCollection`, parameters `ecef_frame` and `tf_timeout`),
  `cras_geo_tools::SphericalToCartesianPolygonCollection` or `cras_geo_tools::SphericalToPlanarPolygonStamped`.

## ROS versions compatibility

| ROS | Support |
|-----|---------|
| Humble | Not supported |
| Jazzy | Until EOL |
| Kilted | Until EOL |
| Lyrical | Until EOL |
| Rolling | Best effort |
