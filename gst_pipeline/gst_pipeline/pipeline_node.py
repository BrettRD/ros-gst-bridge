#!/usr/bin/python3
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject

import rclpy
from rclpy.node import Node

import std_msgs
import diagnostic_updater
import diagnostic_msgs

from gst_pipeline.pipeline import Pipeline
from gst_pipeline.simplebin import Simplebin
from gst_pipeline.webrtc_pipes import webrtc_pipes
from gst_pipeline.webrtc_transport_ws import webrtc_transport_ws



import asyncio
import asyncio_glib
#import gbulb


# locate gstreamer plugins from other ROS packages
def ros_path(package):
  return 'install/' + package + '/lib/' + package

gst_plugin_paths = [ros_path('gst_bridge')]
gst_required_plugins = ["opus", "vpx", "nice", "webrtc", "dtls", "srtp", "rtp",
    "rtpmanager", "videotestsrc", "audiotestsrc", "rosgstbridge"]



def main(args=None):

  #integrate the GLib Mainloop with the asyncio event loop
  # XXX gbulb is deprecated for windows, asyncio_glib has issues with HUP and INT signals

  asyncio.set_event_loop_policy(asyncio_glib.GLibEventLoopPolicy())
  #gbulb.install()

  #init ROS, GObject mainloop, and Gstreamer
  rclpy.init()
  GObject.threads_init()
  Gst.init(None)


  loop = asyncio.get_event_loop()



  pipe_node = Pipeline('pipeline_node')
  pipe_node.add_plugin_paths(gst_plugin_paths)
  if not pipe_node.check_plugins(gst_required_plugins):
    exit(1)



  ## connect to the signalling server before trying to negotiate links
  #webrtc_transport = webrtc_transport_ws(pipe_node, loop)
  #webrtc_segment = webrtc_pipes(pipe_node, webrtc_transport, "webrtc_example_bin")
  #webrtc_transport.connect()
  #pipe_node.add_section(webrtc_segment)





  # basic pipelines
  # descr = 'audiotestsrc volume=0.3 is-live=true wave=red-noise ! tee name=t ! queue ! rosaudiosink provide-clock=False ros-name="audio_node" ros-topic="audio" ros-encoding="S16C2" t. ! queue ! audioconvert ! alsasink'
  # descr = 'videotestsrc is-live=true pattern=ball ! queue ! ximagesink'
  #descr = 'audiotestsrc volume=0.3 is-live=true wave=red-noise ! queue ! audioconvert ! alsasink'
  #descr = 'audiotestsrc volume=0.3 is-live=true wave=red-noise ! tee name=t ! queue ! audioconvert ! alsasink \
  #    t. ! queue !  rosaudiosink ros-name="audio_node" ros-topic="audio" ros-encoding="S16C2"'
  #descr = 'audiotestsrc volume=0.3 is-live=true wave=red-noise ! queue ! audioconvert ! rosaudiosink ros-name="audio_node" ros-topic="audio" ros-encoding="S16C2"'
  descr = 'alsasrc ! audioconvert ! rosaudiosink ros-name="audio_node" ros-topic="audio" ros-encoding="S16C2"'

  simple_segment = Simplebin(pipe_node, descr, 'simple_bridge_test_bin')
  pipe_node.add_section(simple_segment)

  pipe_node.start_pipeline()

  loop.run_until_complete(pipe_node.async_task())  # diagnostics



if __name__ == '__main__':
  main()