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

import rclpy
from rclpy.node import Node

import std_msgs
import diagnostic_updater
import diagnostic_msgs

from rcl_interfaces.msg import ParameterDescriptor, ParameterType


from gst_bridge.webrtc_sigchan import parse_WebRTCSDPType

node_id_param = 'node_id'
peer_id_param = 'peer_id'
server_param = 'server'


class webrtc_transport_ws:
  
  def __init__(self, node, loop, node_id=None, peer_id=None, server=None, offer=True):
    self.node = node
    self.loop = loop
    self.conn = None
    self.diagnostics = diagnostic_updater.FunctionDiagnosticTask('Transport', self.diagnostic_task)
    self.remote_sends_ice_cb = None
    self.remote_sends_sdp_cb = None
    self.offer = offer
    self.node_id = node_id
    self.peer_id = peer_id
    self.server = server

    if self.node_id == None:
      self.node_id = self.node.declare_parameter(node_id_param).value
    if self.node_id == None:
      self.node.get_logger().error('parameter '+ node_id_param + ' not found')
    
    if self.peer_id == None:
      self.peer_id = self.node.declare_parameter(peer_id_param).value
    if self.peer_id == None:
      self.node.get_logger().error('parameter '+ peer_id_param + ' not found')

    if self.server == None:
      self.server = self.node.declare_parameter(server_param).value
    if self.server == None:
      self.server = 'wss://webrtc.nirbheek.in:8443'

    self.node.get_logger().info('using node_id "' + str(self.node_id) + '"')
    self.node.get_logger().info('using peer_id "' + str(self.peer_id) + '"')
    self.node.get_logger().info('using server "' + self.server + '"')





  def connect_callbacks(self, _remote_sends_ice_cb, _remote_sends_sdp_cb):
    self.remote_sends_ice_cb = _remote_sends_ice_cb
    self.remote_sends_sdp_cb = _remote_sends_sdp_cb
    self.node.get_logger().debug('transport connected callbacks')


  def diagnostic_task(self, stat):
    stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'sig_transport')
    stat.add('server connected', str(self.conn != None))
    stat.add('peer found',       str(self.conn != None))
    return stat


  async def async_task(self):
    assert self.conn
    async for message in self.conn:
      self.node.get_logger().debug('remote says "' + message + '"')
      if message == 'HELLO':
        await self.setup_call()
      elif message == 'SESSION_OK':
        #if offer is False, The peer will create the offer on request
        # XXX call this when the pipeline is ready
        if not self.offer:
          await self.conn.send('OFFER_REQUEST {}'.format(self.peer_id))
        
        #self.start_pipeline()
        self.node.get_logger().debug('session ok')
      elif message.startswith('ERROR'):
        self.node.get_logger().error('remote says "' + message + '"')
        #return 1
      else:
        self.handle_sdp(message)
    self.node.get_logger().error('transport closed socket?')
    #return 0

  async def stop(self):
    if self.conn:
      await self.conn.close()
    self.conn = None

  # XXX establish and maintain a link to the server, but don't initiate the call
  #     add a async_task() to the global mainloop
  async def connect(self):
    sslctx = ssl.create_default_context(purpose=ssl.Purpose.CLIENT_AUTH)
    self.conn = await websockets.connect(self.server, ssl=sslctx)
    self.node.get_logger().debug('saying HELLO to remote')
    await self.conn.send('HELLO %d' % self.node_id)
    async for message in self.conn:
      self.node.get_logger().debug('remote says "' + message + '"')
      if message == 'HELLO':
        await self.setup_call()
      elif message == 'SESSION_OK':
        self.node.get_logger().debug('session ok')
        return


  async def setup_call(self):
    self.node.get_logger().info('identifying with server')
    await self.conn.send('SESSION {}'.format(self.peer_id))


  # XXX mush offer and answer handlers together, the duplication is gross
  
  def send_sdp(self, local_sdp):
    # XXX local_sdp.sdp.as_text() strips the local_sdp.type (offer/answer enum)
    text = local_sdp.sdp.as_text()
    type_str = GstWebRTC.WebRTCSDPType.to_string(local_sdp.type)
    # XXX translate the type string for remote (case sensitive remote)

    msg = json.dumps({'sdp': {'type': type_str, 'sdp': text}})
    self.node.get_logger().debug('Sending ' + msg)
    self.loop.create_task(self.conn.send(msg))

  def send_sdp_offer(self, offer):
    self.send_sdp(offer)
    self.node.get_logger().debug('sent offer')

  def send_sdp_answer(self, answer):
    self.send_sdp(answer)
    self.node.get_logger().debug('sent answer')


  def send_ice_candidate(self, element, mlineindex, candidate):
    icemsg = json.dumps({'ice': {'candidate': candidate, 'sdpMLineIndex': mlineindex}})
    self.node.get_logger().debug('sending ice candidate: ' + icemsg)

    self.loop.create_task(self.conn.send(icemsg))


  def handle_sdp(self, message):
    msg = json.loads(message)
    if 'sdp' in msg:
      sdp = msg['sdp']

      sdp_type_str = sdp['type']
      sdp_msg_str = sdp['sdp']

      #parse sdp['sdp'] back to a GstSdp.SDPMessage
      _res, sdp_msg = GstSdp.SDPMessage.new()
      GstSdp.sdp_message_parse_buffer(bytes(sdp_msg_str.encode()), sdp_msg)
      # XXX check success

      #parse sdp['type'] back to a GstWebRTC.WebRTCSDPType
      sdp_type = parse_WebRTCSDPType(sdp_type_str)
      if sdp_type == None:
        self.node.get_logger().error('invalid sdp type')
        return
        
      # combine the parsed sdp into a GstWebRTC.WebRTCSessionDescription
      remote_sdp = GstWebRTC.WebRTCSessionDescription.new(sdp_type, sdp_msg)

      if self.remote_sends_sdp_cb != None:
        self.remote_sends_sdp_cb(remote_sdp)

    elif 'ice' in msg:
      ice = msg['ice']
      candidate = ice['candidate']
      sdpmlineindex = ice['sdpMLineIndex']
      if self.remote_sends_ice_cb != None:
        self.remote_sends_ice_cb(sdpmlineindex, candidate)

