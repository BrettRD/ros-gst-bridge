#!/usr/bin/python3
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject

import rclpy
from rclpy.node import Node

import std_msgs
import diagnostic_updater
import diagnostic_msgs

from gst_bridge.pipeline import Pipeline
from gst_bridge.simplebin import Simplebin
from gst_bridge.webrtc_pipes import webrtc_pipes
from gst_bridge.webrtc_transport_ws import webrtc_transport_ws



import asyncio
#import asyncio_glib
import gbulb


# locate gstreamer plugins from other ROS packages
def ros_path(package):
  return 'install/' + package + '/lib/' + package

gst_plugin_paths = [ros_path('gst_plugins')]
gst_required_plugins = ["opus", "vpx", "nice", "webrtc", "dtls", "srtp", "rtp",
    "rtpmanager", "videotestsrc", "audiotestsrc", "rosaudiosink"]



def main(args=None):

  #integrate the GLib Mainloop with the asyncio event loop
  # XXX gbulb is deprecated for windows, asyncio_glib has issues with HUP and INT signals

  #asyncio.set_event_loop_policy(asyncio_glib.GLibEventLoopPolicy())
  gbulb.install()

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
  webrtc_transport = webrtc_transport_ws(pipe_node, loop)
  webrtc_segment = webrtc_pipes(pipe_node, webrtc_transport, "webrtc_example_bin")
  webrtc_transport.connect()
  pipe_node.add_section(webrtc_segment)



  # basic test case for a pipeline
  #simple_segment = Simplebin(pipe_node, 'videotestsrc is-live=true pattern=ball ! queue ! ximagesink', 'simple_test_bin')
  #pipe_node.add_section(simple_segment)



  # basic test case for a ros topic bridge
  #simple_segment = Simplebin(pipe_node, 'audiotestsrc is-live=true wave=red-noise ! rosaudiosink provide-clock=False ros-name="audio_node" ros-topic="audio" ros-encoding="S16C2" ', 'simple_bridge_test_bin')
  #pipe_node.add_section(simple_segment)


  pipe_node.start_pipeline()


  loop.run_until_complete(pipe_node.async_task())  # diagnostics

if __name__ == '__main__':
  main()