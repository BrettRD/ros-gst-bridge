import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject
gi.require_version('GstWebRTC', '1.0')
from gi.repository import GstWebRTC
gi.require_version('GstSdp', '1.0')
from gi.repository import GstSdp


import rclpy
from rclpy.node import Node

import std_msgs
import diagnostic_updater
import diagnostic_msgs

from gst_pipeline.webrtc_sigchan import webrtc_sigchan


#XXX collect some of these from ros parameter server
default_stun_server = 'stun://stun.l.google.com:19302'
default_audio_src_bin_descr = ' audiotestsrc volume=0.3 is-live=true wave=red-noise ! audioconvert ! audioresample ! queue ! opusenc ! rtpopuspay ! application/x-rtp,media=audio,encoding-name=OPUS,payload=96 ! queue '
default_video_src_bin_descr = ' videotestsrc is-live=true pattern=ball ! videoconvert ! queue ! vp8enc deadline=1 ! rtpvp8pay ! application/x-rtp,media=video,encoding-name=VP8,payload=97 ! queue '
default_video_sink_bin_descr = ' queue ! videoconvert ! ximagesink'
default_audio_sink_bin_descr = ' queue ! alsasink'


class webrtc_pipes:
  def __init__(self, node_, name_, transport,
      param_prefix=None,
      stun_server = default_stun_server,
      webrtc_element_name = 'webrtc_element',
      audio_src_bin_descr = default_audio_src_bin_descr,
      video_src_bin_descr = default_video_src_bin_descr,
      video_sink_bin_descr = default_video_sink_bin_descr,
      audio_sink_bin_descr = default_audio_sink_bin_descr,
      ):
    self.node = node_
    self.chan = webrtc_sigchan(node_, transport)
    self.name = name_

    self.stun_server = stun_server
    self.webrtc_element_name = webrtc_element_name
    self.audio_src_bin_descr = audio_src_bin_descr
    self.video_src_bin_descr = video_src_bin_descr
    self.video_sink_bin_descr = video_sink_bin_descr
    self.audio_sink_bin_descr = audio_sink_bin_descr

    if param_prefix != None:
      (self.stun_server,
      self.webrtc_element_name,
      self.audio_src_bin_descr,
      self.video_src_bin_descr,
      self.video_sink_bin_descr,
      self.audio_sink_bin_descr) = self.fetch_params(param_prefix)


    self.bin = self.build_initial_pipe()
    self.webrtc = self.bin.get_by_name(self.webrtc_element_name)
    self.chan.webrtc = self.webrtc
    self.connect_callbacks(self.webrtc)

    self.diagnostics = diagnostic_updater.CompositeDiagnosticTask(self.name + ' Status')
    self.diagnostics.addTask(diagnostic_updater.FunctionDiagnosticTask(self.name + ' Pipes', self.diagnostic_task))
    self.diagnostics.addTask(self.chan.diagnostics)

    # diagnostics info
    self.audio_src_built = False
    self.video_src_built = False
    self.audio_sink_built = False
    self.video_sink_built = False


  def fetch_params(self, param_prefix):
    if param_prefix != '':
      param_prefix = param_prefix + '.'
    self.node.declare_parameters(
      namespace='',
      parameters=[
        (param_prefix + 'stun_server', None),
        (param_prefix + 'element_name', None),
        (param_prefix + 'audio_src_bin_descr', None),
        (param_prefix + 'video_src_bin_descr', None),
        (param_prefix + 'video_sink_bin_descr', None),
        (param_prefix + 'audio_sink_bin_descr', None)
        #('signalling', None) # always pass the signalling protocol handler by argument
      ]
    )
    stun_server =          self.node.get_parameter(param_prefix + 'stun_server').value
    element_name =         self.node.get_parameter(param_prefix + 'element_name').value
    audio_src_bin_descr =  self.node.get_parameter(param_prefix + 'audio_src_bin_descr').value
    video_src_bin_descr =  self.node.get_parameter(param_prefix + 'video_src_bin_descr').value
    video_sink_bin_descr = self.node.get_parameter(param_prefix + 'video_sink_bin_descr').value
    audio_sink_bin_descr = self.node.get_parameter(param_prefix + 'audio_sink_bin_descr').value
    return (stun_server, element_name, audio_src_bin_descr, video_src_bin_descr, video_sink_bin_descr, audio_sink_bin_descr)


  def build_initial_pipe(self):

    webrtc_bin_descr = 'webrtcbin name=' + self.webrtc_element_name + ' bundle-policy=max-bundle stun-server=' + self.stun_server + ' '
    webrtc_descr = webrtc_bin_descr
    if self.video_src_bin_descr != '':
      webrtc_descr += self.video_src_bin_descr + ' ! ' + self.webrtc_element_name + '.'
    if self.audio_src_bin_descr != '':
      webrtc_descr += self.audio_src_bin_descr + ' ! ' + self.webrtc_element_name + '.'

    self.node.get_logger().debug("building initial webrtc pipes")
    self.node.get_logger().debug(webrtc_descr)

    webrtc_bin = Gst.parse_bin_from_description(webrtc_descr, True)
    webrtc_bin.name = self.name
    
    # XXX creating separate bins requires tracking down the the specific pads to link
    #audio_src_bin =  Gst.parse_bin_from_description(audio_src_bin_descr, False)
    #audio_src_bin.link(webrtc_bin)
    #video_src_bin =  Gst.parse_bin_from_description(video_src_bin_descr, False)
    #video_src_bin.link(webrtc_bin)

    self.audio_src_built = True
    self.video_src_built = True
    self.node.get_logger().debug("initial webrtc pipes built")

    return webrtc_bin



  def connect_callbacks(self, webrtc):
    webrtc.connect('pad-added', self.on_incoming_stream)
    try:
      webrtc.connect('on-data-channel', self.on_data_channel)
    except(TypeError):
      self.node.get_logger().error('data channels are not supported')
    self.chan.connect_callbacks(webrtc)


  def diagnostic_task(self, stat):
    stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'pipes')
    stat.add('audio src built',  str(self.audio_src_built))
    stat.add('video src built',  str(self.video_src_built))
    stat.add('audio sink built', str(self.audio_sink_built))
    stat.add('video sink built', str(self.video_sink_built))
    return stat



  def on_incoming_decodebin_stream(self, element, pad):
    self.node.get_logger().debug("incoming decodebin pad")
    if not pad.has_current_caps():
      print (pad, 'has no caps, ignoring')
      return
    caps = pad.get_current_caps()
    
    if caps.is_empty():
      self.node.get_logger().error('caps is empty')
    elif caps.is_fixed():
      self.node.get_logger().debug('incoming decodebin caps has only one format')
      self.node.get_logger().debug('format string: ' + caps.to_string())
    assert (not caps.is_empty())

    name = caps.to_string()
    if name.startswith('video'):
      sink_bin_descr = self.video_sink_bin_descr
      self.node.get_logger().debug('incoming video stream')
    elif name.startswith('audio'):
      sink_bin_descr = self.audio_sink_bin_descr
      self.node.get_logger().debug('incoming audio stream')

    # XXX using Ghost Pads has a small performace penalty, can we link the pads directly?
    sink_bin = Gst.parse_bin_from_description(sink_bin_descr, True)
    self.bin.add(sink_bin)
    element.link(sink_bin)
    sink_bin.sync_state_with_parent()

    if not sink_bin.sync_state_with_parent():
      self.node.get_logger().error('could not synchonise new sink')
    else:
      if name.startswith('video'):
        self.node.get_logger().debug('synchonised new video sink')
        self.video_sink_built = True
      elif name.startswith('audio'):
        self.node.get_logger().debug('synchonised new audio sink')
        self.audio_sink_built = True


  def on_incoming_stream(self, element, pad):
    self.node.get_logger().debug('incoming webrtc pad')
    if pad.direction != Gst.PadDirection.SRC:
      return
    decodebin = Gst.ElementFactory.make('decodebin')
    decodebin.connect('pad-added', self.on_incoming_decodebin_stream)
    self.bin.add(decodebin)
    element.link(decodebin)
    decodebin.sync_state_with_parent()

  def create_offer(self):
    self.chan.create_offer()

  def on_data_channel (self, element, data_channel):
    self.node.get_logger().debug("new data channel")
    data_channel.connect("on-error", self.data_channel_on_error)
    data_channel.connect("on-open", self.data_channel_on_open)
    data_channel.connect("on-close", self.data_channel_on_close)
    data_channel.connect("on-message-string", self.data_channel_on_message_string)





  def data_channel_on_error(self, dc):
    self.node.get_logger().error("data channel error?")

  def data_channel_on_open(self, dc):
    self.node.get_logger().info('data channel openned, label is "' + dc.label + '"')
    dc.emit('send-string', "Hi! from GStreamer and ROS")

  def data_channel_on_close(self, dc):
    self.node.get_logger().info("data channel closed")

  def data_channel_on_message_string(self, dc, msg):
    self.node.get_logger().info('data channel ' + dc.label + ' says "' + msg + '"')




