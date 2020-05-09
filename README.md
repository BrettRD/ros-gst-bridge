# ros-gst-bridge
an attempt at a transparent, bidirectional, ros to gstreamer bridge

ROS audio-common is only able to shuffle mp3 data from a robot to a laptop.\
audio-common's architecture precludes any useful robotics application for audio data.\
ROS is not the best platform for handling audio and video, but it allows exquisitely weird processes.\
Ideally, gstreamer should handle most of the heavy lifting for multimedia streams, especially across networks.\
ROS should be at least capable of handling data from one gstreamer pipeline into another without loss of information.

## Requirements:
* ROS Messages and Gstreamer caps should not lose metadata like timestamps.
* At least Audio caps need to be handled, these will need a new message definition.
* bridge nodes should be gstreamer bins, not ROS nodes running appsink.
* a flexible intermediate node needs to exist to hold the pipeline and handle pipeline events.

## Issues:
* Invoking autotools from cmake
* Pulling dependencies (message .h files) from ROS into gstreamer
