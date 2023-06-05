# gst_pipeline
This package provides a ROS node that hosts a gstreamer pipeline inside a ROS composable node.
This is similar to the gscam2 package, but here we use the gst-bridge package for shared-memory IO,
allowing this package to focus on adding features to the pipeline itself.

Features are added using pluginlib; you have access to the node interfaces, and the pipeline,
and you don't have to fork the node to do it.


## Features:

### gst-bridge connectors
  This plugin connects to any element from the gst-bridge package,
  and instructs it to attach its publishers and subscribers to the host node
  inside the composable node container.
  This gives you shared-pointer message transport with any other ROS nodes in
  the same composable node container.
  
  ###### Parameters:
  * element_name - String

### Play/Pause:
  Connect a Gazebo style play/pause service to the pipeline, adding two
  services named `/play` and `/pause`. These enable pausing/playing video files
  streaming into ROS the same way you pause/play physics in gazebo.
###### Example:
```
ros2 service call /pause std_srvs/srv/Empty
```


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


### Property to Parameter mappings: (Not yet built)
  This feature should iterate through all elements and expose their
  element properties as ros parameters where sensible type mappings exist.

### Diagnostics: (Not yet built)
  Run a diagnostic updater with information about the pipeline.

### WebRTC: (Not yet built)
  A plugin that connects to the WebRTC signalling callbacks and provides
  transport for peer connection metadata.
  This plugin may need to add and remove elements from the pipeline,
  the addition and removal of nodes may need hooks that trigger plugin re-init

### Aravis: (Not yet built)
  Expose configuration of GiGE / Genicam cameras as ROS parameters


### BYO features:
  These features are based on pluginlib, and use an extremely simple base-class.



Many thanks to Clyde McQueen for his work on gscam2
