# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# A collection of polygons. Each polygon can be expressed in a different frame or time.

std_msgs/Header header
geometry_msgs/PolygonStamped[] polygons
# Optional per-polygon coloring; Either empty or the same number of elements as polygons
std_msgs/ColorRGBA[] colors
