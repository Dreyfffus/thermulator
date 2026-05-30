cmake_minimum_required(VERSION 3.8)
project(thermocator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================================================================
# Option: build digital twin nodes (requires ros_gz_interfaces + Gazebo)
# Off by default so the physical robot build has no Gazebo dependency.
# Usage: colcon build --cmake-args -DBUILD_DT=ON
# =============================================================================
option(BUILD_DT "Build digital twin nodes (requires ros_gz_interfaces)" OFF)

# =============================================================================
# Core dependencies (always required)
# =============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Digital twin dependency — only resolved when BUILD_DT=ON
if(BUILD_DT)
  find_package(ros_gz_interfaces REQUIRED)
endif()

set(THERMOCATOR_DEPS
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  nav2_msgs
  sensor_msgs
  std_msgs
  builtin_interfaces
  geometry_msgs
  visualization_msgs
)

# =============================================================================
# decision_node
# =============================================================================
add_executable(decision_node
  src/decision_node.cpp
  src/action_grid.cpp
)
ament_target_dependencies(decision_node ${THERMOCATOR_DEPS})
target_include_directories(decision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(decision_node PUBLIC cxx_std_17)

# =============================================================================
# thermocator (thermal map builder)
# =============================================================================
add_executable(thermocator
  src/thermocator.cpp
  src/map_builder.cpp
  src/thermal_grid.cpp
)
ament_target_dependencies(thermocator ${THERMOCATOR_DEPS})
target_include_directories(thermocator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(thermocator PUBLIC cxx_std_17)

# =============================================================================
# thermal_broadcaster
# =============================================================================
add_executable(thermal_broadcaster
  src/thermal_broadcaster.cpp
)
ament_target_dependencies(thermal_broadcaster ${THERMOCATOR_DEPS})
target_include_directories(thermal_broadcaster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(thermal_broadcaster PUBLIC cxx_std_17)

# =============================================================================
# thermocator_display -- RViz2 plugin shared library
# =============================================================================
add_library(thermocator_display SHARED
  src/occupancy_grid_overlay.cpp
  include/thermocator/occupancy_grid_overlay.hpp
)
ament_target_dependencies(thermocator_display
  ${THERMOCATOR_DEPS}
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  pluginlib
)
target_include_directories(thermocator_display BEFORE PUBLIC
  /opt/ros/jazzy/opt/rviz_ogre_vendor/include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(thermocator_display PUBLIC cxx_std_17)
target_link_libraries(thermocator_display
  Qt5::Core
  Qt5::Widgets
)
target_compile_options(thermocator_display PRIVATE
  -Wno-unused-parameter
  -Wno-pedantic
  -Wno-extra
)
set_property(TARGET thermocator_display PROPERTY AUTOMOC ON)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# =============================================================================
# Digital twin nodes — only built when BUILD_DT=ON
# =============================================================================
if(BUILD_DT)
  set(DT_DEPS
    ${THERMOCATOR_DEPS}
    ros_gz_interfaces
  )

  # pose_sync_node — teleports Gazebo sim robot to real robot's pose
  add_executable(pose_sync_node
    src/pose_sync_node.cpp
  )
  ament_target_dependencies(pose_sync_node ${DT_DEPS})
  target_include_directories(pose_sync_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  target_compile_features(pose_sync_node PUBLIC cxx_std_17)

  # advisory_node — runs detection in Domain 1 and publishes /advisory/goal
  add_executable(advisory_node
    src/advisory_node.cpp
  )
  ament_target_dependencies(advisory_node ${THERMOCATOR_DEPS})
  target_include_directories(advisory_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  target_compile_features(advisory_node PUBLIC cxx_std_17)
endif()

# =============================================================================
# Install
# =============================================================================
set(INSTALL_TARGETS
  thermocator
  thermocator_display
  thermal_broadcaster
  decision_node
)

if(BUILD_DT)
  list(APPEND INSTALL_TARGETS
    pose_sync_node
    advisory_node
  )
endif()

install(TARGETS ${INSTALL_TARGETS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY ../../docs/
  DESTINATION share/${PROJECT_NAME}/docs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
