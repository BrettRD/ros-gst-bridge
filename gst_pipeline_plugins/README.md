# gst_pipeline_plugins
This package provides utility plugins for the gst_pipeline package.

Features are added using pluginlib; you have access to the node interfaces, and the pipeline, and you don't have to fork the node to do it.

To make a new plugin, simply inherit gst_pipeline::plugin_base using pluginlib and override the initialise function.



## Features:

### gst-bridge connectors
  This plugin connects to any element from the gst-bridge package, and instructs it to attach its publishers and subscribers to the host node.
  This allows you to place the pipeline host node in a composable node container, and gives you shared-pointer image message transport with any other ROS nodes in the same container.

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
  Turn a video feed on or off, or fetch single frame from a live stream with a ROS service call.
  This plugin inserts a probe anywhere in a gstreamer pipeline, and causes the src pad to drop buffers.

  This can be used on low-bandwidth connections where you navigate on down-sampled potato-quality video, and occasionally transmit single high-resolution frames, or maybe you have a debug feed you want to leave off most of the time.
  ###### parameters:
  * element_name - String
  * pass_on_startup


### Clock Observer:
  Gstreamer timestamps and ROS timestamps are different things. sample both at nearly the same instant and track the relationship.
  This plugin is for people who are sticklers about timekeeping and want to accurately compare time measurements taken by one clock with measurements taken against another clock.
  If you use this to tackle Special Relativity, tell us about it!


### multifilesink_observer:
  The multifilesink element saves images (and other things) with files named with a sequential integer suffix.
  This plugin connects to the multifilesink, and listens to the messages it emits, carrying information about the file, including the timestamp of the buffer that it represents.
  Save this data to a rosbag, and you can reconstruct the timeline even if the sampling clock is non-monotonic


### rtp_header_hook:
  This plugin binds to the rtp\*pay and rtp\*depay elements and tacks on some metadata to the rtp header as a header extension, then retrieves it at the other end and publishes the metadata and the buffer timestamp to a ROS message.
  We use this to keep track of when a frame was captured on a remote machine and match it to a timestamp corresponding to a file from multifilesink_observer.

  This element currently only carries the buffer timestamp, but we're eager to hear suggestions for how to make this more flexible.


### metadata_hook: (Incomplete)
  Gstreamer Buffers can have arbitrary metadata attached to them.
  This hook shows how to attach and retrieve metadata in the pipeline. This currently has no specific purpose, but we see it becoming handy for annotations from machine vision applications in the future.


### net_clock: (Incomplete)
  The NetClock class allows you to run two pipelines off the same clock, provided by a network time source.
  This clock class greatly simplifies synchronised playback between multiple machines.


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


