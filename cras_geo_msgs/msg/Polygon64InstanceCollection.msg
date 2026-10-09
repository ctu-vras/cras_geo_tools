# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# A collection of identified polygons, all sharing the same coordinate frame.

std_msgs/Header header
cras_geo_msgs/Polygon64Instance[] polygons
# Optional per-polygon coloring; Either empty or the same number of elements as polygons
std_msgs/ColorRGBA[] colors
