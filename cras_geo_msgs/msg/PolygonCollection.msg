# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# A collection of polygons, all sharing the same coordinate frame.

std_msgs/Header header
geometry_msgs/Polygon[] polygons
# Optional per-polygon coloring; Either empty or the same number of elements as polygons
std_msgs/ColorRGBA[] colors
