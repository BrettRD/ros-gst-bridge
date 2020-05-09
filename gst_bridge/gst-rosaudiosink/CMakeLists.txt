cmake_minimum_required(VERSION 3.10)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

project(rosaudiosink)

find_package(GStreamer 1.0)
find_package(GLIB2)
find_package(GObject)

add_library(gstrosaudiosink SHARED plugin/gstrosaudiosink.h plugin/gstrosaudiosink.c plugin/gstrosaudiosinkplugin.c)

target_include_directories(gstrosaudiosink PRIVATE ${GSTREAMER_INCLUDE_DIRS} ${GLIB2_INCLUDE_DIR} ${GOBJECT_INCLUDE_DIR})
target_link_libraries(gstrosaudiosink PUBLIC ${GSTREAMER_LIBRARY} ${GLIB2_LIBRARIES} ${GOBJECT_LIBRARIES} gstcontroller-1.0 gstaudio-1.0 gstbase-1.0)

install(TARGETS gstrosaudiosink LIBRARY DESTINATION "~/.local/share/gstreamer-1.0")