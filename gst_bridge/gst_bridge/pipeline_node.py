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
from gst_bridge.webrtc_sigchan import webrtc_sigchan
from gst_bridge.webrtc_transport_ws import webrtc_transport_ws

#import Pipeline
#import Simplebin
#import webrtc_pipes
#import webrtc_sigchan
#import webrtc_transport_ws



import asyncio
import asyncio_glib


def check_plugins():
  needed = ["opus", "vpx", "nice", "webrtc", "dtls", "srtp", "rtp",
    "rtpmanager", "videotestsrc", "audiotestsrc"]
  missing = list(filter(lambda p: Gst.Registry.get().find_plugin(p) is None, needed))
  if len(missing):
    print('Missing gstreamer plugins:', missing)
    return False
  else:
    print('gstreamer plugins accounted for')
  return True


def main(args=None):
  rclpy.init()
  GObject.threads_init()
  Gst.init(None)

  #asyncio.set_event_loop_policy(asyncio_glib.GLibEventLoopPolicy())
  # asyncio-glib get_event_loop() returns a GLib event loop 
  loop = asyncio.get_event_loop()

  if not check_plugins():
    exit(1)

  node = rclpy.create_node('pipeline_node')

  pipe_node = Pipeline(node)


  server = 'wss://webrtc.nirbheek.in:8443'
  node_id = 5032
  peer_id = 3717


  # transport for webrtc signalling
  node.get_logger().info("Creating transport")
  webrtc_transport = webrtc_transport_ws(node, node_id, peer_id, server)
  # connect to the server before trying to negotiate links
  loop.run_until_complete(webrtc_transport.connect())

  # signalling logic for webrtc bin
  node.get_logger().info("Creating signalling")
  webrtc_channel = webrtc_sigchan(node, webrtc_transport)
  
  # pipeline bin autoplugger
  node.get_logger().info("Creating webrtc pipes")
  webrtc_segment = webrtc_pipes(node, webrtc_channel, "webrtc_thingo")

  pipe_node.add_section(webrtc_segment)
  pipe_node.start_pipeline()

  # XXX surely each object can assign async tasks internally
  loop.create_task(webrtc_transport.loop())
  
  
  # basic test case for a pipeline
  #  simple_segment = Simplebin(node, 'videotestsrc is-live=true pattern=ball ! queue ! ximagesink', 'test_bin_thing')
  #  pipe_node.add_section(simple_segment)
  #  pipe_node.start_pipeline()
  #  loop.run_until_complete(pipe_node.async_task())  # diagnostics


  loop.run_until_complete(pipe_node.async_task())  # diagnostics

  
  
    #loop = GLib.MainLoop()
  #GLib.timeout_add(100, rosspin)
  #loop.run()

  node.get_logger().warn('fell off the bottom')
  
if __name__ == '__main__':
  main()