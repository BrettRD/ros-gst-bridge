cmake_minimum_required(VERSION 3.12)
project(gst_pipeline_plugins_webrtc)

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


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(gst_bridge REQUIRED)
find_package(gst_pipeline REQUIRED)
find_package(pluginlib REQUIRED)


find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 IMPORTED_TARGET)
pkg_check_modules(GST_SDP REQUIRED gstreamer-sdp-1.0)
pkg_check_modules(GST_WEBRTC REQUIRED gstreamer-webrtc-1.0)


# glib-2.0
# gstreamer-1.0
# gstreamer-rtp-1.0
# gstreamer-sdp-1.0
# gstreamer-webrtc-1.0
# json-glib-1.0
# libsoup-2.4
# gstreamer-webrtc-nice-1.0

pkg_check_modules(SOUP REQUIRED libsoup-2.4 IMPORTED_TARGET)
pkg_check_modules(JSONGLIB REQUIRED json-glib-1.0 IMPORTED_TARGET)




## Include messages
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(gst_msgs REQUIRED)

###########
## Build ##
###########

include_directories(include/${PROJECT_NAME})

add_library(${PROJECT_NAME} SHARED
  src/base.cpp
  src/websockets.cpp
  src/datachannel.cpp
)

# XXX linker error regarding visibility of global variables provided by libsoup
target_link_libraries(${PROJECT_NAME} soup-2.4)
target_link_libraries(${PROJECT_NAME} json-glib-1.0 gio-2.0 gobject-2.0 glib-2.0)
target_link_libraries(${PROJECT_NAME} gstsdp-1.0 gobject-2.0 glib-2.0)
target_link_libraries(${PROJECT_NAME} gstwebrtc-1.0 gstbase-1.0 gstreamer-1.0 gobject-2.0 glib-2.0)

ament_target_dependencies(${PROJECT_NAME}
  gst_pipeline
  gst_bridge
  pluginlib
  rcl
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  gst_msgs
  GSTREAMER
  SOUP
  JSONGLIB
  GST_SDP
  GST_WEBRTC
)



# install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# install static files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)
# install pluginlib plugins
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


#if(BUILD_TESTING)
#  find_package(ament_cmake_clang_format REQUIRED)
#  find_package(ament_cmake_gtest REQUIRED)
#  find_package(osrf_testing_tools_cpp REQUIRED)
#
#  ament_clang_format(CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
#
#  ament_add_gtest(${PROJECT_NAME}_test
#    test/gst_pipeline/test_loader.cpp
#  )
#
#  target_include_directories(${PROJECT_NAME}_test PRIVATE include src)
#  target_link_libraries(${PROJECT_NAME}_test gstpipes)
#  ament_target_dependencies(${PROJECT_NAME}_test
#    gst_bridge
#    pluginlib
#    rcl
#    rclcpp
#    sensor_msgs
#    GSTREAMER
#    osrf_testing_tools_cpp
#  )
#
#endif()
#

pluginlib_export_plugin_description_file(gst_pipeline plugins.xml)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  pluginlib
  gst_bridge
  gst_pipeline
  ament_cmake
)

ament_package(
  CONFIG_EXTRAS
)

