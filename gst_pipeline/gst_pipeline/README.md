# gst_pipeline

## gst_pipes:
This node replaces the gst_pipeline python node, it's faster, simpler,
and has a clearly defined plugin structure.

This node loads via ros_components, and exposes node interfaces to a
  variety of plugins.

Plugins load their config from the node parameters interface,
  connect to the pipeline, and typically search for relevant
  gst elements within the pipeline.

### Plugins allow features to be loaded as needed
Features:

Appsrc / Appsink (Not complete)
  These plugins connect ROS callbacks and GStreamer element callbacks
  These use the node base interface to connect into the ros-components
  foundation offering zero-copy transport from ROS.

  This plugin requires a few additional structures:
  * Polymorphism to cope with Video/Audio/Text publishers
  * CameraInfo publisher
  * ROS allocator to wrap gst-buffer destructor call extends zero-copy

Property to Parameter mappings: (Not yet built)
  This feature should iterate through all elements and expose their
  element properties as ros parameters where sensible type mappings exist

Diagnostics: (Not yet built)
  Run a diagnostic updater with information about the pipeline.

Play/Pause: (Not yet built)
  Connect a Gazebo style play/pause service to the pipeline

WebRTC: (Not yet built)
  A plugin that connects to the WebRTC signalling callbacks and provides
  transport for peer connection metadata.
  This plugin may need to add and remove elements from the pipeline,
  the addition and removal of nodes may need hooks that trigger plugin re-init

Aravis:
  Expose configuration of GiGE / Genicam cameras as ROS parameters




Many thanks to Clyde McQueen for his work on gscam2