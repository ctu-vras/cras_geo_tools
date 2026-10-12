# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.20.2)
project(cras_geo_tools)

set(THIS_PACKAGE_PUBLIC_DEPS
  cras_cpp_common
  cras_geo_msgs
  geographic_msgs
  geometry_msgs
  polygon_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

set(THIS_PACKAGE_PRIVATE_DEPS
  ament_cmake
  ament_cmake_gen_version_h
  geodesy
  rclcpp_components
  tf2_geometry_msgs
)

foreach(dep IN LISTS THIS_PACKAGE_PUBLIC_DEPS THIS_PACKAGE_PRIVATE_DEPS)
  find_package(${dep} REQUIRED)
endforeach()

foreach(dep IN LISTS THIS_PACKAGE_PUBLIC_DEPS)
  ament_export_dependencies(${dep})
endforeach()

add_library(geodesy IMPORTED INTERFACE)
target_link_libraries(geodesy INTERFACE ${geodesy_LIBRARIES})
target_include_directories(geodesy INTERFACE ${geodesy_INCLUDE_DIRS})

set(EXPORTED_LIBS)
set(NON_EXPORTED_LIBS)

add_library(${PROJECT_NAME}_defaults INTERFACE)
target_include_directories(${PROJECT_NAME}_defaults INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}_defaults INTERFACE cras_cpp_common::cras_defaults)
# This does not get exported to downstream packages; each has to specify it on its own
target_compile_features(${PROJECT_NAME}_defaults INTERFACE $<BUILD_INTERFACE:cxx_std_20>)
list(APPEND EXPORTED_LIBS ${PROJECT_NAME}_defaults)

add_library(tf2_polygon_msgs SHARED src/tf2_polygon_msgs.cpp)
target_link_libraries(tf2_polygon_msgs
  PUBLIC
    cras_geo_msgs::cras_geo_msgs
    geometry_msgs::geometry_msgs
    polygon_msgs::polygon_msgs
    tf2::tf2
    ${PROJECT_NAME}_defaults
  PRIVATE
    tf2_geometry_msgs::tf2_geometry_msgs
)
list(APPEND EXPORTED_LIBS tf2_polygon_msgs)

add_library(convert_ecef SHARED src/convert_ecef.cpp)
target_link_libraries(convert_ecef
  PUBLIC
    cras_geo_msgs::cras_geo_msgs
    geographic_msgs::geographic_msgs
    geometry_msgs::geometry_msgs
    polygon_msgs::polygon_msgs
    ${PROJECT_NAME}_defaults
  PRIVATE
    geodesy
)
list(APPEND EXPORTED_LIBS convert_ecef)

add_library(convert_2d_3d SHARED src/convert_2d_3d.cpp)
target_link_libraries(convert_2d_3d
  PUBLIC
    cras_cpp_common::cras_expected
    cras_geo_msgs::cras_geo_msgs
    geometry_msgs::geometry_msgs
    polygon_msgs::polygon_msgs
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::tf2_ros
    ${PROJECT_NAME}_defaults
  PRIVATE
    cras_cpp_common::cras_math_utils
    cras_cpp_common::cras_time_utils
)
list(APPEND EXPORTED_LIBS convert_2d_3d)

add_library(spherical_to_cartesian SHARED nodes/spherical_to_cartesian.cpp)
target_link_libraries(spherical_to_cartesian
  PUBLIC
    convert_ecef
    cras_geo_msgs::cras_geo_msgs
    geographic_msgs::geographic_msgs
    geometry_msgs::geometry_msgs
    polygon_msgs::polygon_msgs
    rclcpp::rclcpp
    rclcpp_components::component
    tf2::tf2
    tf2_polygon_msgs
    tf2_ros::tf2_ros
    ${PROJECT_NAME}_defaults
  PRIVATE
    cras_cpp_common::cras_type_utils
)
rclcpp_components_register_node(spherical_to_cartesian PLUGIN "cras_geo_tools::SphericalToCartesianPointStamped"
  EXECUTABLE spherical_to_cartesian_point_stamped)
rclcpp_components_register_node(spherical_to_cartesian PLUGIN "cras_geo_tools::SphericalToCartesianPoseStamped"
  EXECUTABLE spherical_to_cartesian_pose_stamped)
rclcpp_components_register_node(spherical_to_cartesian PLUGIN "cras_geo_tools::SphericalToCartesianPolygon"
  EXECUTABLE spherical_to_cartesian_polygon)
rclcpp_components_register_node(spherical_to_cartesian PLUGIN "cras_geo_tools::SphericalToCartesianPolygonCollection"
  EXECUTABLE spherical_to_cartesian_polygon_collection)
list(APPEND EXPORTED_LIBS spherical_to_cartesian)

add_library(spherical_to_planar SHARED nodes/spherical_to_planar.cpp)
target_link_libraries(spherical_to_planar
  PUBLIC
    convert_2d_3d
    convert_ecef
    cras_geo_msgs::cras_geo_msgs
    geographic_msgs::geographic_msgs
    geometry_msgs::geometry_msgs
    polygon_msgs::polygon_msgs
    rclcpp::rclcpp
    rclcpp_components::component
    tf2::tf2
    tf2_polygon_msgs
    tf2_ros::tf2_ros
    ${PROJECT_NAME}_defaults
  PRIVATE
    cras_cpp_common::cras_type_utils
)
rclcpp_components_register_node(spherical_to_planar PLUGIN "cras_geo_tools::SphericalToPlanarPolygonStamped"
  EXECUTABLE spherical_to_planar_polygon_stamped)
rclcpp_components_register_node(spherical_to_planar PLUGIN "cras_geo_tools::SphericalToPlanarPolygonCollection"
  EXECUTABLE spherical_to_planar_polygon_collection)
list(APPEND EXPORTED_LIBS spherical_to_planar)

add_library(cartesian_to_spherical SHARED nodes/cartesian_to_spherical.cpp)
target_link_libraries(cartesian_to_spherical
  PUBLIC
    convert_ecef
    cras_geo_msgs::cras_geo_msgs
    geographic_msgs::geographic_msgs
    geometry_msgs::geometry_msgs
    polygon_msgs::polygon_msgs
    rclcpp::rclcpp
    rclcpp_components::component
    tf2::tf2
    tf2_polygon_msgs
    tf2_ros::tf2_ros
    ${PROJECT_NAME}_defaults
  PRIVATE
    cras_cpp_common::cras_type_utils
)
rclcpp_components_register_node(cartesian_to_spherical PLUGIN "cras_geo_tools::CartesianToSphericalPointStamped"
  EXECUTABLE cartesian_to_spherical_point_stamped)
rclcpp_components_register_node(cartesian_to_spherical PLUGIN "cras_geo_tools::CartesianToSphericalPoseStamped"
  EXECUTABLE cartesian_to_spherical_pose_stamped)
rclcpp_components_register_node(cartesian_to_spherical PLUGIN "cras_geo_tools::CartesianToSphericalPolygon"
  EXECUTABLE cartesian_to_spherical_polygon)
rclcpp_components_register_node(cartesian_to_spherical PLUGIN "cras_geo_tools::CartesianToSphericalPolygonCollection"
  EXECUTABLE cartesian_to_spherical_polygon_collection)
if(message_filters_VERSION VERSION_LESS 7.1.0)
  target_compile_definitions(cartesian_to_spherical PRIVATE -DMESSAGE_FILTERS_DONT_HAVE_NODE_INTERFACES=1)
endif()
if(message_filters_VERSION VERSION_LESS 5.0.0)
  target_compile_definitions(cartesian_to_spherical PRIVATE -DMESSAGE_FILTERS_USE_RMW_QOS=1)
endif()
list(APPEND EXPORTED_LIBS cartesian_to_spherical)

add_library(planar_to_spherical SHARED nodes/planar_to_spherical.cpp)
target_link_libraries(planar_to_spherical
  PUBLIC
    convert_2d_3d
    convert_ecef
    cras_geo_msgs::cras_geo_msgs
    geographic_msgs::geographic_msgs
    geometry_msgs::geometry_msgs
    polygon_msgs::polygon_msgs
    rclcpp::rclcpp
    rclcpp_components::component
    tf2::tf2
    tf2_polygon_msgs
    tf2_ros::tf2_ros
    ${PROJECT_NAME}_defaults
  PRIVATE
    cras_cpp_common::cras_type_utils
)
rclcpp_components_register_node(planar_to_spherical PLUGIN "cras_geo_tools::PlanarToSphericalPolygonStamped"
  EXECUTABLE planar_to_spherical_polygon_stamped)
rclcpp_components_register_node(planar_to_spherical PLUGIN "cras_geo_tools::PlanarToSphericalPolygonCollection"
  EXECUTABLE planar_to_spherical_polygon_collection)
list(APPEND EXPORTED_LIBS planar_to_spherical)

install(PROGRAMS
  nodes/gpx_to_ros
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME}/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Generate version.h header
add_library(${PROJECT_NAME}_version INTERFACE)
ament_generate_version_header(${PROJECT_NAME}_version)
list(APPEND EXPORTED_LIBS ${PROJECT_NAME}_version)

# Create a convenience target ${PROJECT_NAME}::${PROJECT_NAME} that links to all sub-libraries
add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE ${EXPORTED_LIBS})
list(APPEND EXPORTED_LIBS ${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
install(TARGETS ${EXPORTED_LIBS}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(cras_lint REQUIRED)
  cras_lint_common()
  cras_lint_cpp()
  cras_lint_py()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_convert_2d_3d test/test_convert_2d_3d.cpp)
  target_link_libraries(test_convert_2d_3d convert_2d_3d)

  ament_add_gtest(test_convert_ecef test/test_convert_ecef.cpp)
  target_link_libraries(test_convert_ecef convert_ecef)

  ament_add_gtest(test_tf2_polygon_msgs test/test_tf2_polygon_msgs.cpp)
  target_link_libraries(test_tf2_polygon_msgs tf2_polygon_msgs)
endif()

ament_package()
