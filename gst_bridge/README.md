# gst_bridge

This ROS package builds gstreamer elements that can be loaded into a gstreamer pipeline.
In this package, ROS is treated as a network-transport protocol for video, audio, and text.
This package does not build ROS nodes.

## Usage:
GStreamer needs to be able to find the plugin, so we need to tell it where to look.
In the example below, we use `--gst-plugin-path=install/lib/gst_bridge/` as an argument to `gst-launch-1.0`. Everything else, is the media pipeline.

This example sends dummy video data from a videotestsrc into ros from the command line.
`gst-launch-1.0 --gst-plugin-path=install/lib/gst_bridge/ videotestsrc ! videoconvert ! rosimagesink ros-topic="image"`

GStreamer makes it easy to change where the video comes from, like a webcam:
`gst-launch-1.0 --gst-plugin-path=install/lib/gst_bridge/ v4l2src ! videoconvert ! rosimagesink ros-topic="image"`

These elements can be loaded into any application that allows you to define a gstreamer pipeline, as long as you can add this ros package to the application's gst-plugin-path, perhaps using the `GST_PLUGIN_PATH` environment variable:
`export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/gst_bridge/lib/gst_bridge`

## More complex usage:

This package has some ROS libraries for interacting with the elements in the pipeline, See the related gst_pipeline package for how that works

## Future work:

The efficiency and utility of the ros-gst-bridge elements could be improved with a few additional features:
* rosimagesink needs a CameraInfo publisher
* gst_bridge.cpp needs an allocator class that allows a derived vector
  type to wrap a gst-buffer. This allows true zero-copy transport of all outbound
  gst buffer data
* gst_bridge.cpp needs a gst buffer unref callback that allows downstream
  elements to defer dereference of a ros shared message pointer
* the rosimagesink class needs a call to borrow memory from the shared-memory
  daemon. the memory should be borrowed as ros messages, and cast to a gst
  buffer pool which is passed to upstream gst elements.


