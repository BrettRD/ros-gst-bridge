import asyncio

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
gi.require_version('GstWebRTC', '1.0')
from gi.repository import GstWebRTC
gi.require_version('GstSdp', '1.0')
from gi.repository import GstSdp

import rclpy
from rclpy.node import Node

import std_msgs
import diagnostic_updater
import diagnostic_msgs




class webrtc_sigchan:
  
  def __init__(self, node_, transport_):
    self.node = node_
    self.webrtc = None #filled  by parent
    self.transport = transport_
    self.diagnostics = diagnostic_updater.CompositeDiagnosticTask('Signalling & Transport')
    self.diagnostics.addTask(diagnostic_updater.FunctionDiagnosticTask('Signalling', self.diagnostic_task))
    self.diagnostics.addTask(self.transport.diagnostics)


  def diagnostic_task(self, stat):
    stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'sig_logic')
    stat.add('webrtc element exists', str(self.webrtc != None))
    sig_state = "Nope"
    if self.webrtc != None:
      sig_state = WebRTCSignalingState_to_str(self.webrtc.get_property('signaling-state'))
    stat.add('webrtc signalling state', sig_state)

    return stat




  # called by the webrtc element owner
  # (XXX gross)
  def connect_callbacks(self, element):
    element.connect('on-negotiation-needed', self.on_negotiation_needed)
    element.connect('on-ice-candidate', self.on_ice_candidate)
    self.webrtc = element
    # further sets transport variables (XXX also gross)
    self.transport.connect_callbacks(self.remote_sends_ice, self.remote_sends_sdp)





  def on_negotiation_needed(self, element):
    # XXX should we offer or wait?
    self.node.get_logger().info('on-negotiation-needed')
    promise = Gst.Promise.new_with_change_func(self.on_offer_created, element)
    # XXX we offer.
    element.emit('create-offer', None, promise)


  # XXX mush offer and answer handlers together, the duplication is gross
    # offer is of type GstWebRTC.WebRTCSessionDescription
    # offer has fields sdp  : GstSdp.SDPMessage
    #           and    type : GstWebRTC.WebRTCSDPType

  def on_offer_created(self, promise, element):
    self.node.get_logger().info('offer was created')
    promise.wait()
    reply = promise.get_reply()
    offer = reply.get_value('offer')
    promise = Gst.Promise.new_with_change_func(self.local_description_set, element)
    element.emit('set-local-description', offer, promise)
    self.transport.send_sdp_offer(offer)
    self.node.get_logger().info('sent offer')

  
  def on_answer_created(self, promise, element):
    self.node.get_logger().info('answer was created')
    promise.wait()
    reply = promise.get_reply()
    answer = reply.get_value('answer')
    if answer == None:
      self.node.get_logger().error('answer not found')

    promise = Gst.Promise.new_with_change_func(self.local_description_set, element)
    element.emit('set-local-description', answer, promise)
    self.transport.send_sdp_answer(answer)
    self.node.get_logger().info('sent answer')


  def on_offer_set(self, promise, element):
    self.node.get_logger().info('remote description was set')
    promise = Gst.Promise.new_with_change_func(self.on_answer_created, element)
    element.emit("create-answer", None, promise)


  def on_answer_set(self, promise, element):
    self.node.get_logger().info('remote description was set')


  # send ice candidate
  def on_ice_candidate(self, element, mlineindex, candidate):
    self.transport.send_ice_candidate(element, mlineindex, candidate)


  def local_description_set(self, promise, element):
    self.node.get_logger().info('local description was set')


  def remote_sends_sdp(self, remote_sdp):
    # XXX sanity check with the signal channel whether we should receive an offer or answer
    # XXX sanity check that we're in the right part of the process
    if self.webrtc == None:
      return
    promise = None
    element = self.webrtc
    if remote_sdp.type == GstWebRTC.WebRTCSDPType.OFFER:
      promise = Gst.Promise.new_with_change_func(self.on_offer_set, element)
      self.node.get_logger().info('received offer from remote')
    elif remote_sdp.type == GstWebRTC.WebRTCSDPType.ANSWER:
      promise = Gst.Promise.new_with_change_func(self.on_answer_set, element)
      self.node.get_logger().info('received answer from remote')
    else:
      self.node.get_logger().error('remote sent a thing? ' + GstWebRTC.WebRTCSDPType.to_string(remote_sdp.type))

    element.emit('set-remote-description', remote_sdp, promise)



  # remote sent ice candidate
  def remote_sends_ice(self, mlineindex, candidate):
    # XXX sanity check with the signal channel whether we should receive an offer or answer
    # XXX sanity check that we're in the right part of the process
    self.webrtc.emit('add-ice-candidate', mlineindex, candidate)
    self.ice_added = True
    self.node.get_logger().info('ice candidate added')




# XXX at least try to make case consistent

# missing string conversions from GstWebRTC
# XXX double check that these functions really don't exist in GstWebRTC
def WebRTCSignalingState_to_str(state):
  if state == GstWebRTC.WebRTCSignalingState.STABLE:
    state_str = 'stable'
  elif state == GstWebRTC.WebRTCSignalingState.CLOSED:
    state_str = 'closed'
  elif state == GstWebRTC.WebRTCSignalingState.HAVE_LOCAL_OFFER:
    state_str = 'have local offer'
  elif state == GstWebRTC.WebRTCSignalingState.HAVE_REMOTE_OFFER:
    state_str = 'have remote offer'
  elif state == GstWebRTC.WebRTCSignalingState.HAVE_LOCAL_PRANSWER:
    state_str = 'have local pranswer'
  elif state == GstWebRTC.WebRTCSignalingState.HAVE_REMOTE_PRANSWER:
    state_str = 'have remote pranswer'
  else:
    state_str = 'unknown'
  return state_str


# XXX excessively brittle?
def parse_WebRTCSDPType(sdp_type_str):
  if sdp_type_str == GstWebRTC.WebRTCSDPType.to_string(GstWebRTC.WebRTCSDPType.OFFER):
    sdp_type = GstWebRTC.WebRTCSDPType.OFFER
  elif sdp_type_str == GstWebRTC.WebRTCSDPType.to_string(GstWebRTC.WebRTCSDPType.PRANSWER):
    sdp_type = GstWebRTC.WebRTCSDPType.PRANSWER
  elif sdp_type_str == GstWebRTC.WebRTCSDPType.to_string(GstWebRTC.WebRTCSDPType.ANSWER):
    sdp_type = GstWebRTC.WebRTCSDPType.ANSWER
  elif sdp_type_str == GstWebRTC.WebRTCSDPType.to_string(GstWebRTC.WebRTCSDPType.ROLLBACK):
    sdp_type = GstWebRTC.WebRTCSDPType.ROLLBACK
  else:  #sdp_type_str == "unknown"
    sdp_type = None
  return sdp_type