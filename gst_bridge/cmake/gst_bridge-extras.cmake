find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(gstreamer REQUIRED IMPORTED_TARGET gstreamer-1.0)
  pkg_check_modules(gstreamer-base REQUIRED IMPORTED_TARGET gstreamer-base-1.0)
  pkg_check_modules(gstreamer-app REQUIRED IMPORTED_TARGET gstreamer-app-1.0)
  pkg_check_modules(gstreamer-audio REQUIRED IMPORTED_TARGET gstreamer-audio-1.0)
  pkg_check_modules(gstreamer-video REQUIRED IMPORTED_TARGET gstreamer-video-1.0)

  # Propagate GStreamer include dirs through ament's exported variables so that
  # downstream packages using ament_target_dependencies(... gst_bridge) pick up
  # the sub-module include paths (audio, video, etc.) that gst_bridge headers need.
  list(APPEND gst_bridge_INCLUDE_DIRS
    ${gstreamer_INCLUDE_DIRS}
    ${gstreamer-base_INCLUDE_DIRS}
    ${gstreamer-app_INCLUDE_DIRS}
    ${gstreamer-audio_INCLUDE_DIRS}
    ${gstreamer-video_INCLUDE_DIRS}
  )
  list(REMOVE_DUPLICATES gst_bridge_INCLUDE_DIRS)
endif()
