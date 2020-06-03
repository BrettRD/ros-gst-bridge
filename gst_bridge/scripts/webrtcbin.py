import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject
gi.require_version('GstWebRTC', '1.0')
from gi.repository import GstWebRTC
gi.require_version('GstSdp', '1.0')
from gi.repository import GstSdp


import roslib
roslib.load_manifest('diagnostic_updater')
import rospy
import diagnostic_updater
import diagnostic_msgs
import std_msgs


#XXX collect some of these from ros parameter server
webrtc_name = 'sendrecv'
stun_server = 'stun://stun.l.google.com:19302'
audio_src_bin_descr = 'audiotestsrc is-live=true wave=red-noise ! audioconvert ! audioresample ! queue ! opusenc ! rtpopuspay ! queue ! application/x-rtp,media=audio,encoding-name=OPUS,payload=96'
video_src_bin_descr = 'videotestsrc is-live=true pattern=ball ! videoconvert ! queue ! vp8enc deadline=1 ! rtpvp8pay ! queue ! application/x-rtp,media=video,encoding-name=VP8,payload=97'
audio_sink_bin_descr = 'queue ! audioconvert ! audioresample ! autoaudiosink'
video_sink_bin_descr = 'queue ! videoconvert ! autovideosink'
webrtc_descr = 'webrtcbin name=' + webrtc_name + ' bundle-policy=max-bundle stun-server=' + stun_server



class webtrtcbin:
  def __init__(self, _name, _signalling_channel):
    self.name = _name
    self.chan = _signalling_channel
    self.bin = self.build_initial_pipe()
    self.webrtc = self.bin.get_by_name(webrtc_name)
    self.chan.webrtc = self.webrtc
    self.connect_callbacks(self.webrtc)
    self.chan.connect_callbacks(self.webrtc)

    self.pipes_diagnostics = diagnostic_updater.FunctionDiagnosticTask(self.name + ' Pipes', self.diagnostic_task)
    self.diagnostics = diagnostic_updater.CompositeDiagnosticTask(self.name + ' Status')
    self.diagnostics.addTask(self.pipes_diagnostics)
    self.diagnostics.addTask(self.chan.diagnostics)

    # diagnostics info
    self.audio_src_built = False
    self.video_src_built = False
    self.audio_sink_built = False
    self.video_sink_built = False




  def build_initial_pipe(self):
    webrtc_bin = Gst.parse_bin_from_description(webrtc_descr, False)
    webrtc_bin.name = self.name
    
    audio_src_bin =  Gst.parse_bin_from_description(audio_src_bin_descr, False)
    audio_src_bin.link(webrtc_bin)
    self.audio_src_built = True

    video_src_bin =  Gst.parse_bin_from_description(video_src_bin_descr, False)
    video_src_bin.link(webrtc_bin)
    self.video_src_built = True
    return webrtc_bin

  def connect_callbacks(self, webrtc):
    self.webrtc.connect('pad-added', self.on_incoming_stream)




  def diagnostic_task(self, stat):
    stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'eh?')
    stat.add('audio src built',  self.audio_src_built)
    stat.add('video src built',  self.video_src_built)
    stat.add('audio sink built',  self.audio_sink_built)
    stat.add('video sink built',  self.video_sink_built)

    # attach diagnostic on 'signaling-state' type: Web-rtcsignaling-state 
    # sig_state = self.webrtc.get_property('signaling-state')
    return stat



  def on_incoming_decodebin_stream(self, element, pad):
    if not pad.has_current_caps():
      print (pad, 'has no caps, ignoring')
      return
    caps = pad.get_current_caps()
    assert (len(caps))
    s = caps[0]
    name = s.get_name()
    if name.startswith('video'):
      sink_bin_descr = video_sink_bin_descr
    elif name.startswith('audio'):
      sink_bin_descr = audio_sink_bin_descr

    sink_bin = Gst.parse_bin_from_description(sink_bin_descr, False)
    self.bin.add(sink_bin)
    #sink_pad = sink_bin.find_unlinked_pad(Gst.PadDirection.SINK)
    #pad.link(sink_pad)
    element.link(sink_bin)
    if not sink_bin.sync_state_with_parent():
      rospy.loginfo("could not synchonise new sink")
    else:
      if name.startswith('video'):
        self.video_sink_built = True
      elif name.startswith('audio'):
        self.audio_sink_built = True


  def on_incoming_stream(self, element, pad):
    if pad.direction != Gst.PadDirection.SRC:
      return
    decodebin = Gst.ElementFactory.make('decodebin')
    decodebin.connect('pad-added', self.on_incoming_decodebin_stream)
    self.bin.add(decodebin)
    decodebin.sync_state_with_parent()
    element.link(decodebin)






