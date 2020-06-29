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
# find_package(builtin_interfaces REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

# set(msg_files
#   "msg/Audio.msg"
# )

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
  src/rosaudiosink.cpp
  include/rosaudiosink.h
  src/gstmultiudpsink.c
  include/gstmultiudpsink.h
  src/gstudpnetutils.c
  include/gstudpnetutils.h
  include/glib-compat-private.h
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
  #${GSTREAMER_BASE_INCLUDE_DIRS}
  ${GLIB_INCLUDE_DIRS}
  ${GOBJECT_INCLUDE_DIR}
)

target_link_libraries(rosgstbridge PUBLIC
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${audio_msgs_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  #${GSTREAMER_BASE_LIBRARIES}
  ${GLIB_LIBRARIES}
  ${GLIB_GIO_LIBRARIES}
  gstcontroller-1.0
  gstaudio-1.0
  gstbase-1.0
  gstnet-1.0
)



# note the nonstandard install location for the gst plugin
install(TARGETS rosgstbridge
  DESTINATION lib/${PROJECT_NAME}
)
# DESTINATION "~/.local/share/gstreamer-1.0"
# you may have to set
# GST_PLUGIN_SYSTEM_PATH="$HOME/.local/share/gstreamer-1.0"
# check that your gstreamer can find it with 
#     gst-inspect-1.0 rosgstbridge
# currently exists in colcon default install location so from the workspace:
#   gst-inspect-1.0 install/gst_bridge/lib/gst_bridge/librosgstbridge.so

ament_package(
  CONFIG_EXTRAS
)



## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gst_bridge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
