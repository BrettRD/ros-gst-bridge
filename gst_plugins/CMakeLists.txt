cmake_minimum_required(VERSION 3.5)
project(gst_plugins)


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


# find gstreamer
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(GStreamer)
find_package(GLIB2)
find_package(GObject)


## Include messages
find_package(sensor_msgs REQUIRED)
#find_package(gst_bridge_msgs REQUIRED)


## Generate added messages and services with any dependencies listed here
# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}
#   DEPENDENCIES std_msgs
# )


###########
## Build ##
###########




# This is a gstreamer plugin, not a ros node
# the install location needs to be found by gst-inspect

add_library(rosgstbridge SHARED src/rosgstbridgeplugin.cpp include/rosaudiosink.h src/rosaudiosink.cpp)


# ament has a habit of pruning unused linked functions required by gstreamer
# symptom of this is 'undefined symbol: gst_audio_sink_get_type' on gst-inspect
target_include_directories(rosgstbridge PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GSTREAMER_INCLUDE_DIRS}
  ${GLIB2_INCLUDE_DIR}
  ${GOBJECT_INCLUDE_DIR}
  # gst_bridge_msgs
  )

target_link_libraries(rosgstbridge PUBLIC
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${GSTREAMER_LIBRARY}
  ${GLIB2_LIBRARIES}
  ${GOBJECT_LIBRARIES}
  gstcontroller-1.0
  gstaudio-1.0
  gstbase-1.0
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
#   gst-inspect-1.0 install/gst_plugins/lib/gst_plugins/librosgstbridge.so

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
