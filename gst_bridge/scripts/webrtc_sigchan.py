import asyncio
import json
import ssl
import websockets

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
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



class webrtc_sigchan_ws:
  
  def __init__(self, id_, peer_id, server):
    self.server = server or 'wss://webrtc.nirbheek.in:8443'
    self.id_ = id_
    self.peer_id = peer_id
    self.conn = None
    self.webrtc = None #filled  by parent
    self.pipes_diagnostics = diagnostic_updater.FunctionDiagnosticTask('Signalling', self.diagnostic_task)



  def diagnostic_task(self, stat):
    stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'eh?')
    
    stat.add('webrtc element exists', (self.webrtc != None))
    stat.add('server connected', self.conn != None)
    stat.add('peer found', self.conn != None)

    # attach diagnostic on 'signaling-state' type: Web-rtcsignaling-state 
    # sig_state = self.webrtc.get_property('signaling-state')
    return stat

  # called by the webrtc element owner
  # (XXX gross)
  def connect_callbacks(self, webrtc):
    self.webrtc = webrtc
    self.webrtc.connect('on-negotiation-needed', self.on_negotiation_needed)
    self.webrtc.connect('on-ice-candidate', self.on_ice_candidate)





  async def connect(self):
    sslctx = ssl.create_default_context(purpose=ssl.Purpose.CLIENT_AUTH)
    self.conn = await websockets.connect(self.server, ssl=sslctx)
    await self.conn.send('HELLO %d' % self.id_)

  async def setup_call(self):
    await self.conn.send('SESSION {}'.format(self.peer_id))

  def send_sdp_offer(self, offer):
    text = offer.sdp.as_text()
    print ('Sending offer:\n%s' % text)
    msg = json.dumps({'sdp': {'type': 'offer', 'sdp': text}})
    loop = asyncio.new_event_loop()
    loop.run_until_complete(self.conn.send(msg))
    loop.close()


  def send_sdp_answer(self, answer):
    text = answer.sdp.as_text()
    print ('Sending answer:\n%s' % text)
    msg = json.dumps({'sdp': {'type': 'answer', 'sdp': text}})
    loop = asyncio.new_event_loop()
    loop.run_until_complete(self.conn.send(msg))
    loop.close()



  def handle_sdp(self, message):
    if self.webrtc == None:
      return
    msg = json.loads(message)
    if 'sdp' in msg:
      sdp = msg['sdp']
      sdp_type = None
      # XXX swap this out for the actual type / enum
      if sdp['type'] == 'answer':
        sdp_type = 'answer'
        #GstWebRTC.WebRTCSDPType.ANSWER
      elif sdp['type'] == 'offer':
        sdp_type = 'offer'
        #GstWebRTC.WebRTCSDPType.OFFER
      sdp = sdp['sdp']
      print ('Received answer:\n%s' % sdp)
      self.webrtc.remote_sends_sdp(sdp, sdp_type)
      #res, sdpmsg = GstSdp.SDPMessage.new()
      #GstSdp.sdp_message_parse_buffer(bytes(sdp.encode()), sdpmsg)
      #answer = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.ANSWER, sdpmsg)
      #promise = Gst.Promise.new()
      #self.webrtc.emit('set-remote-description', answer, promise)
      #promise.interrupt()
    elif 'ice' in msg:
      ice = msg['ice']
      candidate = ice['candidate']
      sdpmlineindex = ice['sdpMLineIndex']
      self.webrtc.remote_sends_ice(sdpmlineindex, candidate)

# this system can't yet accept an offer
# remove all webrtc signal handlers from this class










  def on_negotiation_needed(self, element, user_data):
    # XXX should we offer or wait?
    promise = Gst.Promise.new_with_change_func(self.on_offer_created, element, user_data)
    element.emit('create-offer', None, promise)

  
  # XXX mush offer and answer handlers together, the duplication is gross
  
  def on_offer_created(self, promise, element, user_data):
    self.offer_created = True
    promise.wait()
    reply = promise.get_reply()
    offer = reply['offer']
    # offer is of type GstWebRTC.WebRTCSessionDescription
    # offer has fields sdp  : GstSdp.SDPMessage
    #           and    type : GstWebRTC.WebRTCSDPType
    promise = Gst.Promise.new_with_change_func(self.local_description_set, user_data)
    element.emit('set-local-description', offer, promise)
    self.send_sdp_offer(offer)
    rospy.loginfo('created offer')


  def on_answer_created(self, promise, element, user_data):
    self.answer_created = True
    promise.wait()
    reply = promise.get_reply()
    answer = reply['answer']
    promise = Gst.Promise.new_with_change_func(self.local_description_set, user_data)
    element.emit('set-local-description', answer, promise)
    self.send_sdp_answer(answer)
    rospy.loginfo('created answer')


  def on_offer_set(self, user_data):
    self.remote_sdp_set = True
    rospy.loginfo('remote description was set')
    promise = Gst.Promise.new_with_change_func(self.on_answer_created, user_data)
    self.webrtc.emit("create-answer", None, promise, user_data)


  def on_answer_set(self, user_data):
    self.remote_sdp_set = True
    rospy.loginfo('remote description was set')


  def local_description_set(self, user_data):
    self.local_sdp_set = True
    rospy.loginfo('local description was set')


  def remote_sends_sdp(self, sdp, sdp_type):
    # XXX sanity check with the signal channel whether we should receive an offer or answer
    # XXX sanity check that we're in the right part of the process
    user_data = None
    res, sdpmsg = GstSdp.SDPMessage.new()
    GstSdp.sdp_message_parse_buffer(bytes(sdp.encode()), sdpmsg)
    remote_sdp = None
    promise = None
    if sdp_type == 'offer':
      remote_sdp = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.OFFER, sdpmsg)
      promise = Gst.Promise.new_with_change_func(self.on_offer_set, user_data)
      rospy.loginfo('received offer from remote')
    elif sdp_type == 'answer':
      remote_sdp = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.ANSWER, sdpmsg)
      promise = Gst.Promise.new_with_change_func(self.on_answer_set, user_data)
      rospy.loginfo('received answer from remote')
    else:
      rospy.logerr('remote sent a thing?')

    self.webrtc.emit('set-remote-description', remote_sdp, promise, user_data)

  def on_ice_candidate(self, element, mlineindex, candidate, user_data):
    rospy.logerr('sending ice candidate')
    icemsg = json.dumps({'ice': {'candidate': candidate, 'sdpMLineIndex': mlineindex}})
    loop = asyncio.new_event_loop()
    loop.run_until_complete(self.conn.send(icemsg))
    loop.close()


  def remote_sends_ice(self, mlineindex, candidate):
    # XXX sanity check with the signal channel whether we should receive an offer or answer
    # XXX sanity check that we're in the right part of the process
    user_data = None
    self.webrtc.emit('add-ice-candidate', mlineindex, candidate, user_data)
    self.ice_added = True
    rospy.loginfo('ice candidate added')


