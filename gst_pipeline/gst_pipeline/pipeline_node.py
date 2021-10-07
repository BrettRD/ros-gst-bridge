#!/usr/bin/python3
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject

import rclpy

from gst_pipeline.pipeline import Pipeline
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType

import os



import asyncio
import asyncio_glib
#import gbulb


# 
simple_bins_list = 'simple_bins'
webrtc_bins_list = 'webrtc_bins'



# XXX this is an awful hack to deal with broken param type mappings in galactic.
# https://github.com/ros2/rclpy/issues/829#issuecomment-937517881
rosdistro=os.environ['ROS_DISTRO']
if rosdistro == 'galactic':
  distro_dynamic_typing={'dynamic_typing':True}
else:
  distro_dynamic_typing={}



def main(args=None):

  #integrate the GLib Mainloop with the asyncio event loop
  # XXX gbulb is deprecated for windows,
  # XXX asyncio_glib has issues with HUP and INT signals,
  # XXX ROS integration would probably be a lot smoother if this lib leaned on regular futures and left the GLib mainloop to mind itself
  asyncio.set_event_loop_policy(asyncio_glib.GLibEventLoopPolicy())
  #gbulb.install()

  #init ROS, GObject mainloop, and Gstreamer
  rclpy.init()
  GObject.threads_init()
  Gst.init(None)

  loop = asyncio.get_event_loop()

  # create an empty pipeline and ros node
  pipe_node = Pipeline('pipeline_node')

  # go to the parameter server and create some stuff to put into the pipeline
  # your factory methods here, pull requests are encouraged.
  build_simple_bins_from_params(pipe_node)
  build_webrtc_bins(pipe_node, loop)

  #instruct gstreamer to begin preroll and playback
  pipe_node.start_pipeline()
  # loop over the diagnostic tasks of the pipeline and everything in it.
  loop.run_until_complete(pipe_node.async_task())  # diagnostics



# XXX standardise the factory method call, and move the call itself to the file that it corresponds to
#  def build_*_from_params(param_list, pipe_node, loop):
#    return [bin_list]



# factory method for simplebin
# pipe_node: instance of a class that implements ROS Node for parameters, and can accept a GST bin and a diagnostic_updater task
def build_simple_bins_from_params(pipe_node):
  pipe_node.declare_parameters(
    namespace='',
    parameters=[
      (simple_bins_list, [], ParameterDescriptor(
        name=simple_bins_list,
        type=ParameterType.PARAMETER_STRING_ARRAY,
        **distro_dynamic_typing
        )
      ),
    ]
  )
  simple_bins_names = pipe_node.get_parameter(simple_bins_list).value
  if simple_bins_names:
    from gst_pipeline.simplebin import Simplebin

    for name in simple_bins_names:
      simple_segment = Simplebin(pipe_node, name, param_prefix=name)
      pipe_node.add_section(simple_segment)




# factory method for webrtcbin
# pipe_node: instance of a class that implements ROS Node for parameters, and can accept a GST bin and a diagnostic_updater task
def build_webrtc_bins(pipe_node, loop):
  pipe_node.declare_parameters(
    namespace='',
    parameters=[
      (webrtc_bins_list, [], ParameterDescriptor(
        name=webrtc_bins_list,
        type=ParameterType.PARAMETER_STRING_ARRAY,
        **distro_dynamic_typing
        )
      ),
    ]
  )
  webrtc_bins_names = pipe_node.get_parameter(webrtc_bins_list).value
  if webrtc_bins_names:
    from gst_pipeline.webrtc_pipes import webrtc_pipes
    from gst_pipeline.webrtc_transport_ws import webrtc_transport_ws
    for name in webrtc_bins_names:
      pipe_node.declare_parameters(
        namespace='',
        parameters=[
          (name + '.' + 'signalling', None)
        ]
      )
      webrtc_transport = None
      signalling = pipe_node.get_parameter(name + '.' + 'signalling').value

      if 'webrtc_transport_ws' == signalling:
        pipe_node.get_logger().error("building websockets signalling client for " + name)
        webrtc_transport = webrtc_transport_ws(pipe_node, loop, param_prefix=name)

      elif 'webrtc_transport_ros' == signalling:
        #webrtc_transport = webrtc_transport_ros(pipe_node, loop, param_prefix=name)
        pipe_node.get_logger().error("the ros topic transport for webrtc hasn't been built yet")

      else:
        pipe_node.get_logger().error("webrtc requires a signalling system, none selected")

      ## connect to the signalling server before trying to negotiate links
      webrtc_segment = webrtc_pipes(pipe_node, name, webrtc_transport, param_prefix=name)
      webrtc_transport.connect()
      pipe_node.add_section(webrtc_segment)



if __name__ == '__main__':
  main()
