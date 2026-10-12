<!-- SPDX-License-Identifier: BSD-3-Clause -->
<!-- SPDX-FileCopyrightText: Czech Technical University in Prague -->

# cras_geo_tools

![ROS 2 compatible](https://img.shields.io/badge/ROS-2-blue) ![Jazzy](https://img.shields.io/badge/jazzy-green) ![Kilted](https://img.shields.io/badge/kilted-green) ![Lyrical](https://img.shields.io/badge/lyrical-green) ![Rolling](https://img.shields.io/badge/rolling-green)

## Overview

This package provides a set of tools for working with geographic data in ROS2.

- [cras_geo_msgs](cras_geo_msgs) define many useful messages, partly extending
  `geometry_msgs/Polygon*`, `geographic_msgs` and `polygon_msgs`.
- [cras_geo_tools](cras_geo_tools) provides nodes and libraries for converting between different geographic
  coordinate systems (WGS84, ECEF, local planar frames), for visualizing the polygons in RViz and for importing and
  exporting them as GPX files.

See the readmes of individual packages for more information.