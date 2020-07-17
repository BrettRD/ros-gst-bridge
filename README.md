# ros-gst-bridge
A bidirectional, ROS to GStreamer bridge

ROS is great for running exquisitely weird processes on video streams.\
GStreamer is great for running complex pipelines with conversions between common formats.

It should be easy to pass data between GStreamer and ROS without loss of information.

## Features

### audio_msgs
A message class for transporting raw audio data with appropriate metadata for analysis
(this is likely to change)

### gst_pipeline
A collection of python scripts that handle gstreamer pipeline generation within a ROS node.
Simplebin takes a gst-launch style pipeline descriptor and assembles basic but extremely versatile pipelines.
This collection includes a modular WebRTC signalling system and pipeline handler making it easier to get data to a browser.

### gst_bridge
A package containing a GStreamer plugin, and simple format conversions (similar goal to cv-bridge).
The GStreamer plugin has source and sink elements that appear on the ROS graph as independent ROS nodes.
These nodes can be configured by passing parameters via the GStreamer pipeline, and can be assigned names, namespaces, and frame_ids.  These nodes can also be launched using gst-launch, or instantiated in pipelines inside other applications.


## Design goals:
* ROS Messages and GStreamer caps should not lose metadata like timestamps.
* ROS sim-time and pipeline clocks must be translatable. (accelerated simulations should drive accelerated pipelines)
* A new message format for Audio messages permitting accurate time stamps, flexible number formats, multiple channels, and flexible sample packing.
* bridge nodes should be gstreamer bins, not ROS nodes running appsink. (this reduces code complexity, improves pipeline efficiency, and allows ROS2 borrowed messages to be passed through the pipeline to facilitate zero-copy publishing)
* A ROS node should hold the pipeline and handle pipeline events, allowing use of ros launch and parameters.
* The pipeline node should be extensible to allow complex event handling like WebRTC signalling.


## Architecture:
GStreamer is able to run multiple streams in parallel and has an effective threading system. 
Managing pipeline events is not a resource intensive process, and can be comfortably handled by a Python script without significant loss of performance or generality. This pipeline manager could also be written in C, but the GLib C and ROS C++ conventions make it difficult to avoid memory leaks.
A gstreamer pipeline can comfortably manage multiple independent streams, so only one pipeline node should be required on any given host.

The sources and sinks should be implemented as gstreamer elements for a couple of reasons.\
GStreamer elements are intended to be compact and versatile, this encourages reduction of code complexity.
Ros elements would allow any GStreamer capable application to interact directly with ROS.  `gst-launch 'rosimagesrc topic="image_raw" ! gamma gamma=2.0 ! rosimagesink topic="image_gamma_corrected'` executed from the command line would apply a gamma correction to an image topic.
GStreamer is able to pre-allocate memory from down-stream elements and pass it upstream, enabling sink elements to benefit from the ROS zero-copy API.

It is possible to build this using the gstreamer appsrc/appsink API, but it requires re-implementation of the whole plugin architecture that GStreamer implements so well.
audio-common and gs-cam have good examples of the gstreamer appsrc/appsink API.  


## Remaining work
* The python node needs to call `Gst.Registry().scan_path(install/<package_name>/lib/<package_name>)` to import the elements, ideally this path should be derived from a call to ROS tools.
* pipeline elements should optionally provide a clock source to GStreamer, to allow the use of ROS sim-time.
* pipeline elements should optionally provide a clock source to ROS, to allow the use of pipeline time generated from an external hardware clock.
* image format equivalences need further testing, and basic format converters are needed.
