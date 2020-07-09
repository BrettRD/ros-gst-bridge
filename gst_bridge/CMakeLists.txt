cmake_minimum_required(VERSION 3.5)
project(gst_bridge)


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
# find gstreamer

set(gstreamer_components app pbutils audio fft net)
find_package(GStreamer 1.8.3 REQUIRED COMPONENTS ${gstreamer_components})
# find glib
set(glib_components gio gio-unix gobject gthread gmodule)
find_package(GLIB 2.36 REQUIRED COMPONENTS ${glib_components})

## Include messages
find_package(std_msgs REQUIRED)
find_package(audio_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

## Generate added messages and services with any dependencies listed here
# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}
#   DEPENDENCIES builtin_interfaces std_msgs
# )



###########
## Build ##
###########




# This is a gstreamer plugin, not a ros node
# the install location needs to be found by gst-inspect

add_library(rosgstbridge SHARED
  src/rosgstbridgeplugin.cpp 
  src/gst_bridge.cpp
  src/rosaudiosink.cpp
  src/rosimagesink.cpp
  src/rosaudiosrc.cpp
  src/rosimagesrc.cpp
  )


#rosidl_target_interfaces(rosgstbridge
#  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ament has a habit of pruning unused linked functions required by gstreamer
# symptom of this is 'undefined symbol: gst_audio_sink_get_type' on gst-inspect
target_include_directories(rosgstbridge PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${audio_msgs_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GSTREAMER_INCLUDE_DIRS}
  ${GLIB_INCLUDE_DIRS}
  ${GOBJECT_INCLUDE_DIR}
)

target_link_libraries(rosgstbridge PUBLIC
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${audio_msgs_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GLIB_LIBRARIES}
  ${GLIB_GIO_LIBRARIES}
  gstcontroller-1.0
  gstaudio-1.0
  gstvideo-1.0
  gstbase-1.0
  gstnet-1.0
)

#ament_export_include_directories(include)
#ament_export_libraries(${PROJECT_NAME})
#install(
#  DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION include/${PROJECT_NAME}
#)

# note the nonstandard install location for the gst plugin
install(TARGETS rosgstbridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package(
  CONFIG_EXTRAS
)

