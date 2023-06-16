# gst_pipeline_plugins
This package provides utility plugins for the gst_pipeline package.

Features are added using pluginlib; you have access to the node interfaces, and the pipeline, and you don't have to fork the node to do it.


## Features:

### gst-bridge connectors
  This plugin connects to any element from the gst-bridge package,
  and instructs it to attach its publishers and subscribers to the host node
  inside the composable node container.
  This gives you shared-pointer message transport with any other ROS nodes in
  the same composable node container.

  ###### Parameters:
  * element_name - String

### Play/Pause/Seek:
  Connect a Gazebo style play/pause service to the pipeline.
  Pause video files streaming into ROS the same way you pause physics in gazebo
  This plugin also allows you to seek through a video file with a ros service call specifying the nanoseconds from the beginning of the file


### Buffer Observer:
  Sends a ros message whenever a buffer passes out of a specified GStreamer element
  
  ###### parameters:
  * element_name - String



### Frame Gate:
  Retrieve single video frames from a live stream with a ROS service call.
  This plugin inserts a probe anywhere in a gstreamer pipeline, and causes the
  src pad to drop buffers unless the ros service was recently called.

  This can be used on low-bandwidth connections where you navigate on
  down-sampled potato-quality video, and occasionally transmit single frames
  of high definition on demand.
  ###### parameters:
  * element_name - String
  * pass_on_startup

### Dynamic pipelines: (Not yet built)
  This plugin adds and removes elements from the pipeline.
  Element discovery mechanisms in other plugins will need to use the bin_added signals to wait for their corresponding element to be created instead of assuming they will exist on init.

### Property to Parameter mappings: (Not yet built)
  This feature should iterate through all elements and expose their
  element properties as ros parameters where sensible type mappings exist.

### Diagnostics: (Not yet built)
  Run a diagnostic updater with information about the pipeline.

### Aravis: (Not yet built)
  Expose configuration of GiGE / Genicam cameras as ROS parameters

### BYO features:
  These features are based on pluginlib, and use an extremely simple base-class.


