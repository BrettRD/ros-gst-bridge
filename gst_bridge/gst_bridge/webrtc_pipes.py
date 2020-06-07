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


#XXX collect some of these from ros parameter server
webrtc_name = 'sendrecv'
stun_server = 'stun://stun.l.google.com:19302'
audio_src_bin_descr = ' audiotestsrc is-live=true wave=red-noise ! audioconvert ! audioresample ! queue ! opusenc ! rtpopuspay ! application/x-rtp,media=audio,encoding-name=OPUS,payload=96 ! queue ! ' + webrtc_name + '. '
video_src_bin_descr = ' videotestsrc is-live=true pattern=ball ! videoconvert ! queue ! vp8enc deadline=1 ! rtpvp8pay ! application/x-rtp,media=video,encoding-name=VP8,payload=97 ! queue ! ' + webrtc_name + '. '
webrtc_descr = 'webrtcbin name=' + webrtc_name + ' bundle-policy=max-bundle stun-server=' + stun_server + ' '
webrtc_descr = webrtc_descr + '\n ' + video_src_bin_descr + '\n ' + audio_src_bin_descr

video_sink_bin_descr = ' queue ! videoconvert ! ximagesink'
audio_sink_bin_descr = ' queue ! audioconvert ! audioresample ! autoaudiosink'


class webrtc_pipes:
  def __init__(self, node_, chan_,  name_):
    self.node = node_
    self.chan = chan_
    self.name = name_
    self.bin = self.build_initial_pipe()
    self.webrtc = self.bin.get_by_name(webrtc_name)
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




  def build_initial_pipe(self):
    self.node.get_logger().info("building initial webrtc pipes")
    self.node.get_logger().info(webrtc_descr)

    webrtc_bin = Gst.parse_bin_from_description(webrtc_descr, True)
    webrtc_bin.name = self.name
    
    #audio_src_bin =  Gst.parse_bin_from_description(audio_src_bin_descr, False)
    #audio_src_bin.link(webrtc_bin)
    #self.audio_src_built = True
    #video_src_bin =  Gst.parse_bin_from_description(video_src_bin_descr, False)
    #video_src_bin.link(webrtc_bin)
    #self.video_src_built = True

    self.node.get_logger().info("initial webrtc pipes built")

    return webrtc_bin




  def connect_callbacks(self, webrtc):
    webrtc.connect('pad-added', self.on_incoming_stream)
    self.chan.connect_callbacks(webrtc)




  def diagnostic_task(self, stat):
    stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'eh?')
    stat.add('audio src built',  str(self.audio_src_built))
    stat.add('video src built',  str(self.video_src_built))
    stat.add('audio sink built', str(self.audio_sink_built))
    stat.add('video sink built', str(self.video_sink_built))

    # attach diagnostic on 'signaling-state' type: Web-rtcsignaling-state 
    # sig_state = self.webrtc.get_property('signaling-state')
    return stat



  def on_incoming_decodebin_stream(self, element, pad):
    self.node.get_logger().info("incoming decodebin pad")
    if not pad.has_current_caps():
      print (pad, 'has no caps, ignoring')
      return
    caps = pad.get_current_caps()
    
    if caps.is_empty():
      self.node.get_logger().error('caps is empty')
    elif caps.is_fixed():
      self.node.get_logger().info('incoming decodebin caps has only one format')
      self.node.get_logger().info('format string: ' + caps.to_string())
      


    assert (not caps.is_empty())


    name = caps.to_string()

    if name.startswith('video'):
      sink_bin_descr = video_sink_bin_descr
      self.node.get_logger().info("incoming video stream")
    elif name.startswith('audio'):
      sink_bin_descr = audio_sink_bin_descr
      self.node.get_logger().info("incoming audio stream")

    sink_bin = Gst.parse_bin_from_description(sink_bin_descr, False)
    self.bin.add(sink_bin)
    #sink_pad = sink_bin.find_unlinked_pad(Gst.PadDirection.SINK)
    #pad.link(sink_pad)
    element.link(sink_bin)

    if not sink_bin.sync_state_with_parent():
      self.node.get_logger().info("could not synchonise new sink")
    else:
      if name.startswith('video'):
        self.node.get_logger().info("synchonised new video sink")
        self.video_sink_built = True
      elif name.startswith('audio'):
        self.node.get_logger().info("synchonised new audio sink")
        self.audio_sink_built = True


  def on_incoming_stream(self, element, pad):
    self.node.get_logger().info("incoming webrtc pad")
    if pad.direction != Gst.PadDirection.SRC:
      return
    decodebin = Gst.ElementFactory.make('decodebin')
    decodebin.connect('pad-added', self.on_incoming_decodebin_stream)
    self.bin.add(decodebin)
    decodebin.sync_state_with_parent()
    element.link(decodebin)






