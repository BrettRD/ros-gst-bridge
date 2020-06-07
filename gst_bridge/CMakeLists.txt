cmake_minimum_required(VERSION 2.8.3)
project(gst_bridge)

set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)

## turn on the linter
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()


## System dependencies are found with CMake's conventions
##find_package(rosidl_default_generators REQUIRED)

## Include messages
#find_package(Sensormsgs/Image REQUIRED)
#find_package(gst_bridge_msgs/Audio REQUIRED)
#find_package(gst_bridge_msgs/Image REQUIRED)




## Generate added messages and services with any dependencies listed here
# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}
#   DEPENDENCIES std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
ament_export_include_directories()

## Declare a C++ library
ament_export_libraries()

ament_export_dependencies()

## Declare a C++ executable

# add_executable(${PROJECT_NAME}_node src/gst_bridge_node.cpp)

## Rename C++ executable without prefix
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

## GStreamer Plugin
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/gst-rosaudiosink)


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
