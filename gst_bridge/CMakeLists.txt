cmake_minimum_required(VERSION 3.5)
project(gst_bridge)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Emulate colcon in CLion
if($ENV{CLION_IDE})
  set(audio_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/audio_msgs/share/audio_msgs/cmake")
endif()


find_package(PkgConfig REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

ament_environment_hooks(register_plugin.dsv.in)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# find gstreamer
pkg_search_module(gstreamer REQUIRED IMPORTED_TARGET gstreamer-1.0)
pkg_search_module(gstreamer-base REQUIRED IMPORTED_TARGET gstreamer-base-1.0)
pkg_search_module(gstreamer-app REQUIRED IMPORTED_TARGET gstreamer-app-1.0)
pkg_search_module(gstreamer-audio REQUIRED IMPORTED_TARGET gstreamer-audio-1.0)
pkg_search_module(gstreamer-video REQUIRED IMPORTED_TARGET gstreamer-video-1.0)


# find glib
set(glib_components gio gio-unix gobject gthread gmodule)
find_package(GLIB REQUIRED COMPONENTS ${glib_components})

## Include messages
find_package(std_msgs REQUIRED)
find_package(audio_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)


###########
## Build ##
###########


# XXX this lib build needs pruning
add_library(gst_bridge SHARED
  src/gst_bridge.cpp
  src/rosbaseiface.cpp
  src/rosbasesink.cpp
  src/rosbasesrc.cpp
)
target_include_directories(gst_bridge PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${audio_msgs_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GLIB_INCLUDE_DIRS}
  ${GOBJECT_INCLUDE_DIR}
)
target_link_libraries(gst_bridge PUBLIC
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${audio_msgs_LIBRARIES}
  ${GLIB_LIBRARIES}
  ${GLIB_GIO_LIBRARIES}
  PkgConfig::gstreamer
  PkgConfig::gstreamer-app
  PkgConfig::gstreamer-audio
  PkgConfig::gstreamer-base
  PkgConfig::gstreamer-video
)



# This is a gstreamer plugin, not a ros node
# the install location needs to be found by gst-inspect

add_library(rosgstbridge SHARED
  src/rosgstbridgeplugin.cpp
  src/rosaudiosink.cpp
  src/rosimagesink.cpp
  src/rostextsink.cpp
  src/rosaudiosrc.cpp
  src/rosimagesrc.cpp
  src/rostextsrc.cpp
)


target_include_directories(rosgstbridge PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${audio_msgs_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GLIB_INCLUDE_DIRS}
  ${GOBJECT_INCLUDE_DIR}
)

target_link_libraries(rosgstbridge PUBLIC
  gst_bridge
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${audio_msgs_LIBRARIES}
  ${GLIB_LIBRARIES}
  ${GLIB_GIO_LIBRARIES}
  ${GLIB_GOBJECT_LIBRARIES}
  PkgConfig::gstreamer
  PkgConfig::gstreamer-app
  PkgConfig::gstreamer-audio
  PkgConfig::gstreamer-base
  PkgConfig::gstreamer-video
)


if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
endif()


ament_export_include_directories(include)
ament_export_libraries(gst_bridge)

install(DIRECTORY include/
  DESTINATION include)


install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# note the nonstandard install location for the gst plugin
install(TARGETS rosgstbridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  audio_msgs
  sensor_msgs
)

ament_package(
  CONFIG_EXTRAS cmake/gst_bridge-extras.cmake
)

