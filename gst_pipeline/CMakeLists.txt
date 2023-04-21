cmake_minimum_required(VERSION 3.12)
project(gst_pipeline)

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
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(class_loader REQUIRED)
find_package(pluginlib REQUIRED)

find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)  # provided by tinyxml2 upstream, or tinyxml2_vendor

# Thanks clydemcqueen for gscam2
# Gstreamer doesn't provide CMake files
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 IMPORTED_TARGET)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0 IMPORTED_TARGET)



## Include messages
find_package(std_msgs REQUIRED)
find_package(audio_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)


###########
## Build ##
###########

include_directories(include/${PROJECT_NAME})


add_library(gstpipes SHARED
  src/gst_pipes.cpp
  #src/gst_pipes_plugins.cpp
)

ament_target_dependencies(gstpipes
  gst_bridge
  pluginlib
  rcl
  rclcpp
  rclcpp_components
  sensor_msgs
  GSTREAMER
)

add_library(gstpipes_plugins SHARED
  src/gst_pipes_plugins.cpp
)

ament_target_dependencies(gstpipes_plugins
  gst_bridge
  pluginlib
  rcl
  rclcpp
  sensor_msgs
  GSTREAMER
)


rclcpp_components_register_node(
  gstpipes
  PLUGIN "gst_pipes::gst_pipes"
  EXECUTABLE gst_pipes_node
  )

pluginlib_export_plugin_description_file(gst_pipeline plugins.xml)


ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  ${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}
)

# install composable node
install(TARGETS gstpipes
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)


# install pluginlib plugins
install(TARGETS gstpipes_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(
  gstpipes_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  pluginlib
  gst_bridge
)

ament_package(
  CONFIG_EXTRAS
)

