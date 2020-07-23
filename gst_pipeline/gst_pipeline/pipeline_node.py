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

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix


import asyncio
import asyncio_glib
#import gbulb


# locate gstreamer plugins from other ROS packages


simple_bins_list = 'simple_bins'
simple_bins_props = 'simple_bin_props'

webrtc_bins_list = 'webrtc_bins'
webrtc_bins_props = 'webrtc_bin_props'




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

  pipe_node = Pipeline('pipeline_node')

  # declare the lists of event handlers
  pipe_node.declare_parameters(
    namespace='',
    parameters=[
      (simple_bins_list, []),
      (simple_bins_props, None),
      (webrtc_bins_list, []),
      (webrtc_bins_props, None),
    ]
  )



  pipe_node.start_pipeline()
  loop.run_until_complete(pipe_node.async_task())  # diagnostics


def build_webrtc_bins(pipe_node, loop):
  webrtc_bins = pipe_node.get_parameter(webrtc_bins_list)

  if webrtc_bins != None:
    # the parameters used by this type of handler, and safe defaults
    props = [
      ('stun_server', None),
      ('bin_name', None),
      ('audio_src_bin_descr', None),
      ('video_src_bin_descr', None),
      ('video_sink_bin_descr', None),
      ('audio_sink_bin_descr', None),
      ('signalling', None)
      ]
    # expand the properties over the list of handlers
    param_list = [(name + '.' + prop[0], prop[1])  for prop in props  for name in webrtc_bins]
    # declare the parameters and defaults
    pipe_node.declare_parameters(namespace=webrtc_bins_props, parameters=param_list)


    for name in webrtc_bins:
      param_prefix = webrtc_bins_props + '.' + name + '.'
      
      stun_server =          pipe_node.get_parameter(param_prefix + 'stun_server').value
      bin_name =             pipe_node.get_parameter(param_prefix + 'bin_name').value
      audio_src_bin_descr =  pipe_node.get_parameter(param_prefix + 'audio_src_bin_descr').value
      video_src_bin_descr =  pipe_node.get_parameter(param_prefix + 'video_src_bin_descr').value
      video_sink_bin_descr = pipe_node.get_parameter(param_prefix + 'video_sink_bin_descr').value
      audio_sink_bin_descr = pipe_node.get_parameter(param_prefix + 'audio_sink_bin_descr').value
      signalling =           pipe_node.get_parameter(param_prefix + 'signalling').value
      
      webrtc_transport = None
      if 'webrtc_transport_ws' == signalling:
        trans_param_prefix = param_prefix + 'webrtc_transport_ws.'
        param_list = [
          (trans_param_prefix + 'signalling_server', ''),
          (trans_param_prefix + 'node_id', 0),
          (trans_param_prefix + 'peer_id', 0),
          (trans_param_prefix + 'offer', True),
          (trans_param_prefix + 'autodial', True)
          ]
        pipe_node.declare_parameters(namespace='', parameters=param_list)
        
        signalling_server = pipe_node.get_parameter(param_prefix + 'signalling_server').value
        node_id =   pipe_node.get_parameter(param_prefix + 'node_id').value
        peer_id =   pipe_node.get_parameter(param_prefix + 'peer_id').value
        offer =     pipe_node.get_parameter(param_prefix + 'offer').value
        autodail =  pipe_node.get_parameter(param_prefix + 'autodail').value

        webrtc_transport = webrtc_transport_ws(pipe_node, loop, 
            server = signalling_server,
            node_id = node_id,
            peer_id = peer_id,
            offer = offer,
            autodial = autodail
          )
      elif 'webrtc_transport_ros' == signalling:
        trans_param_prefix = param_prefix + 'webrtc_transport_ros.'

        param_list = [
          (trans_param_prefix + 'sdp_pub_topic', 'sdp_outgoing'),
          (trans_param_prefix + 'sdp_sub_topic', 'sdp_incoming'),
          (trans_param_prefix + 'ice_pub_topic', 'ice_outgoing'),
          (trans_param_prefix + 'ice_sub_topic', 'ice_incoming')
          ]
        pipe_node.declare_parameters(namespace='', parameters=param_list)
        
        sdp_pub_topic = pipe_node.get_parameter(trans_param_prefix + 'sdp_pub_topic').value
        sdp_sub_topic = pipe_node.get_parameter(trans_param_prefix + 'sdp_sub_topic').value
        ice_pub_topic = pipe_node.get_parameter(trans_param_prefix + 'ice_pub_topic').value
        ice_sub_topic = pipe_node.get_parameter(trans_param_prefix + 'ice_sub_topic').value
        #webrtc_transport = webrtc_transport_ros(pipe_node, loop, 
        #    sdp_pub_topic = sdp_pub_topic,
        #    sdp_sub_topic = sdp_sub_topic,
        #    ice_pub_topic = ice_pub_topic,
        #    ice_sub_topic = ice_sub_topic,
        #    offer = offer
        #  )
        pipe_node.get_logger().error("the ros topic transport for webrtc hasn't been built yet")
        webrtc_transport = None
      else:
        pipe_node.get_logger().error("webrtc requires a signalling system")
        webrtc_transport = None
      
      

      ## connect to the signalling server before trying to negotiate links
      webrtc_segment = webrtc_pipes(pipe_node, webrtc_transport, name,
        stun_server = stun_server,
        webrtc_element_name = bin_name,
        audio_src_bin_descr = audio_src_bin_descr,
        video_src_bin_descr = video_src_bin_descr,
        video_sink_bin_descr = video_sink_bin_descr,
        audio_sink_bin_descr = audio_sink_bin_descr,
      )
      webrtc_transport.connect()
      pipe_node.add_section(webrtc_segment)



def build_simple_bins(pipe_node):
  # get a list of the bins we want to create
  simple_bins = pipe_node.get_parameter(simple_bins_list).value
  if simple_bins != None:
    # the parameters used by this type of handler, and safe defaults
    props = [('descr', None)]  # probably worth including a ParameterDescriptor
    # expand the properties over the list of handlers
    param_list = [(name + '.' + prop[0], prop[1])  for prop in props  for name in simple_bins]
    # declare the parameters and defaults
    pipe_node.declare_parameters(namespace=simple_bins_props, parameters=param_list)

    for name in simple_bins:
      param_prefix = simple_bins_props + '.' + name + '.' # XXX looks like sub-params are moving from '.' to '/'
      param = param_prefix + 'descr' 
      descr = pipe_node.get_parameter(param).value
      if descr == None:
        pipe_node.get_logger().error('parameter '+ param + ' not found')
      else:
        pipe_node.get_logger().error('creating simple pipeline ' + name + ' with description "' + descr + '"')
        simple_segment = Simplebin(pipe_node, descr, name)
        pipe_node.add_section(simple_segment)



if __name__ == '__main__':
  main()