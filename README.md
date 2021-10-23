# ros-gst-bridge
A bidirectional, ROS to GStreamer bridge

[ROS](https://www.ros.org/) is great for running exquisitely weird processes on data streams.\
[GStreamer](https://gstreamer.freedesktop.org/documentation/) is great for running complex pipelines with conversions between common formats.

It should be easy to pass data between GStreamer and ROS without loss of information.

## Features

### gst_bridge
A ROS2 package containing a GStreamer plugin, and simple format conversions (similar goal to cv-bridge).
The GStreamer plugin has source and sink elements that appear on the ROS graph as independent ROS nodes.
These nodes can be configured by passing parameters via the GStreamer pipeline, and can be assigned names, namespaces, and frame_ids.  These nodes can also be launched using gst-launch, or instantiated in pipelines inside other applications.  
Currently implemented are `rosaudiosink`, `rosaudiosrc`, `rosimagesink`, `rosimagesrc`, `rostextsink`, and `rostextsrc`
Inspect them with `gst-inspect-1.0 --gst-plugin-path=install/gst_bridge/lib/gst_bridge/ rosaudiosink`

### gst_pipeline
A ROS2 package with a collection of python scripts that handle gstreamer pipeline generation within a ROS node.
Simplebin takes a gst-launch style pipeline descriptor and assembles basic but extremely versatile pipelines.
This collection includes a modular WebRTC signalling system and pipeline handler making it easier to get data to a browser.
gst_pipeline exposes properties of gstreamer elements as parameters of the ROS2 node hosting the pipeline, allowing you to tune your pipeline on the fly.

### audio_msgs
A message class for transporting raw audio data with metadata equivalent to sensor_msgs/image

## Design goals:
* ROS Messages and GStreamer caps should not lose metadata like timestamps.
* ROS sim-time and pipeline clocks must be translatable. (accelerated simulations should drive accelerated pipelines)
* A new message format for Audio messages permitting accurate time stamps, flexible formats, multiple channels, and flexible sample packing.
* Bridge nodes should be gstreamer bins, not ROS nodes running appsink.
* A ROS node should hold the pipeline and handle pipeline events, allowing use of ros launch and parameters.
* The pipeline node should be extensible to allow complex event handling like WebRTC signalling.
* ROS2 should be useable as an inter-process communication channel

## Architecture:
GStreamer is able to run multiple streams in parallel and has an effective threading system. 
Managing pipeline events is not a resource intensive process, and can be comfortably handled by a Python script without significant loss of performance or generality. This pipeline manager could also be written in C, but the GLib C and ROS C++ conventions make it difficult to avoid memory leaks.
A gstreamer pipeline can comfortably manage multiple independent streams, so only one pipeline node should be required on any given host.

The sources and sinks should be implemented as gstreamer elements for a couple of reasons.\
GStreamer elements are intended to be compact and versatile, this encourages reduction of code complexity.
Ros elements would allow any GStreamer capable application to interact directly with ROS.  `gst-launch --gst-plugin-path=install/gst_bridge/lib/gst_bridge/ rosimagesrc ros-topic="image_raw" ! gamma gamma=2.0 ! rosimagesink ros-topic="image_gamma_corrected"` executed from the command line would apply a gamma correction to an image topic.

It is possible to build this using the gstreamer appsrc/appsink API, but it requires re-implementation of the whole plugin architecture that GStreamer implements so well.
audio-common and gs-cam have good examples of the gstreamer appsrc/appsink API.  


## Remaining work
* Pipeline elements should optionally provide a clock source to GStreamer, to allow the use of ROS sim-time.
* Pipeline elements should optionally provide a clock source to ROS, to allow the use of pipeline time generated from an external hardware clock.
* exploration of allocator APIs so that gstreamer can preallocate buffers from the downstream DDS


## Examples
* render a ros std_msgs String message onto a ros sensor_msgs Image  
  `gst-launch-1.0 --gst-plugin-path=install/gst_bridge/lib/gst_bridge/ rostextsrc ! textrender ! videoconvert ! rosimagesink`
* apply a gamma correction to an image topic  
  `gst-launch --gst-plugin-path=install/gst_bridge/lib/gst_bridge/ rosimagesrc ros-topic="image_raw" ! gamma gamma=2.0 ! rosimagesink ros-topic="image_gamma_corrected"`
* stream a raspberry pi camera over UDP and expose the H264 bitrate as an adjustable ROS2 param
```
raspicam_udp.config.yaml
/**:
  ros__parameters:
    simple_bins:
      - 'raspicam_udp'
    raspicam_udp:
      descr: 'rpicamsrc bitrate=10000000 preview=0 ! video/x-h264,width=640,height=480,framereate=10/1,profile=high ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=host.local port=5000'
    gst_plugins_required:
      - 'rpicamsrc'
```
* speech recognition as ros string messages
  `gst-launch-1.0 --gst-plugin-path=install/gst_bridge/lib/gst_bridge/ autoaudiosrc ! audioconvert ! audioresample ! pocketsphinx ! queue ! rostextsink`
* *your applications here!*


## Building
Install ROS 2 ([instructions for Ubuntu](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html))

Make sure you have the colcon toolchain installed
* rosdep ([instructions for Ubuntu](http://wiki.ros.org/rosdep#Installing_rosdep))
* colcon (do not forget the extensions) ([instructions for Ubuntu](https://colcon.readthedocs.io/en/released/user/installation.html#using-debian-packages))

`sudo rosdep init` if it has not already been done on this host. (this is safe to do twice)

If you do not yet have a ROS2 workspace, create one ( e.g. `mkdir ~/galactic_ws/src` )

Clone this repo under the `src/` folder of your ROS2 workspace ( `git clone https://github.com/BrettRD/ros-gst-bridge.git ~/galactic_ws/src/ros-gst-bridge` )

The ros tools colcon and rosdep need to know about your system version of ros, so make sure your [environment](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html#environment-setup) is set up now: ( `source /opt/ros/galactic/setup.bash` )

This package requires software that might not already be installed on your host, so run the following in your ROS2 workspace.  rosdep will look for `package.xml` files in your workspace and install dependencies as needed using apt.
```
rosdep update
rosdep install --from-paths ~/galactic_ws/src/ --ignore-src -r -y
```

Then you should be able to build the packages with [`colcon build`](https://docs.ros.org/en/galactic/Tutorials/Colcon-Tutorial.html).

This build step creates new objects that need to be added to your environment.
Sourcing (not executing) the workspace setup.bash `source ~/galactic_ws/install/setup.bash` will add these new objects to your environment.

The gstreamer elements are all built into the same plugin under `install/gst_bridge/lib/gst_bridge/librosgstbridge.so` .
If you still have your ROS2 environment active (from `source ~/galactic_ws/install/setup.bash`) you should be able to `gst-inspect-1.0 install/gst_bridge/lib/gst_bridge/librosgstbridge.so` to get a list of the elements provided by the library.

You can get details about a specific element using a command like `source ~/galactic_ws/src/install/setup.bash; GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/gst_bridge/lib/gst_bridge gst-inspect-1.0 rosimagesrc`

If you inspect the plugin `gst-inspect-1.0` without your environment set up, it will fail missing shared objects.


## Licenses
* gst_pipeline: LGPLv3, depends on gstreamer (LGPLv2.1+)
* gst_bridge: LGPLv3, depends on gstreamer (LGPLv2.1+)
* audio_msgs: Apache 2
