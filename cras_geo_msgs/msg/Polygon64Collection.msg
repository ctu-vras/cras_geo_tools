# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# A collection of polygons, all sharing the same coordinate frame.

std_msgs/Header header
cras_geo_msgs/Polygon64[] polygons
# Optional per-polygon coloring
std_msgs/ColorRGBA[] colors
