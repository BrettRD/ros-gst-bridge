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


def check_plugins(registry):
  needed = ["opus", "vpx", "nice", "webrtc", "dtls", "srtp", "rtp",
    "rtpmanager", "videotestsrc", "audiotestsrc", "rosaudiosink"]
  missing = list(filter(lambda p: registry.get().find_plugin(p) is None, needed))
  if len(missing):
    print('Missing gstreamer plugins:', missing)
    return False
  else:
    print('gstreamer plugins accounted for')
  return True




def plugin_added(registry, plugin):
  print('gstreamer plugin added')
  print('plugin name: "' + plugin.get_name() + '"')
  print('plugin filename: "' + plugin.get_filename() + '"')
  print('plugin description: "' + plugin.get_description() + '"')
  if None == plugin.load():
    print('plugin load error')




def main(args=None):
  #asyncio.set_event_loop_policy(asyncio_glib.GLibEventLoopPolicy())
  gbulb.install()

  loop = asyncio.get_event_loop()

  rclpy.init()
  GObject.threads_init()
  Gst.init(None)


  registry = Gst.Registry()
  registry.connect('plugin-added', plugin_added)

  if registry.scan_path('install/gst_plugins/lib/gst_plugins'):
    print('registry changed')
  else:
    print('nothing found?')

  
  if not check_plugins(registry):
    exit(1)


  pipe_node = Pipeline('pipeline_node')


  
  ## connect to the signalling server before trying to negotiate links
  webrtc_transport = webrtc_transport_ws(pipe_node, loop)
  loop.run_until_complete(webrtc_transport.connect())
  webrtc_segment = webrtc_pipes(pipe_node, webrtc_transport, "webrtc_thingo")
  pipe_node.add_section(webrtc_segment)
  loop.create_task(webrtc_transport.async_task())
  # XXX surely each object can assign async tasks internally



  # basic test case for a pipeline
  #simple_segment = Simplebin(pipe_node, 'videotestsrc is-live=true pattern=ball ! queue ! ximagesink', 'test_bin_thing')
  #pipe_node.add_section(simple_segment)



  # basic test case for a ros topic bridge
  #simple_segment = Simplebin(pipe_node, 'audiotestsrc is-live=true wave=red-noise ! rosaudiosink provide-clock=False ros-name="audio_node" ros-topic="audio" ros-encoding="S16C2" ', 'bridge_test_thing')
  #pipe_node.add_section(simple_segment)


  pipe_node.start_pipeline()

  loop.run_until_complete(pipe_node.async_task())  # diagnostics

  
  #GLib.timeout_add(100, rosspin)
  
  #loop = GLib.MainLoop()
  #loop.run()

  pipe_node.get_logger().warn('fell off the bottom')
  
if __name__ == '__main__':
  main()