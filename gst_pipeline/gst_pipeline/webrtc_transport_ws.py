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

from gst_pipeline.webrtc_sigchan import parse_WebRTCSDPType

node_id_param = 'node_id'
peer_id_param = 'peer_id'
server_param = 'server'


class webrtc_transport_ws:
  
  def __init__(self, node, loop,
    param_prefix=None,
    server=None,
    node_id=None,
    peer_id=None,
    offer=True,
    autodial=True
  ):

    self.node = node
    self.loop = loop

    self.server = server
    self.node_id = node_id
    self.peer_id = peer_id
    self.offer = offer
    self.autodial = autodial
    if param_prefix != None:
      (self.server,
      self.node_id,
      self.peer_id,
      self.offer,
      self.autodial) = self.fetch_params(param_prefix)

    self.conn = None  # websockets connection
    self.diagnostics = diagnostic_updater.FunctionDiagnosticTask('Transport', self.diagnostic_task)
    self.remote_sends_ice_cb = None
    self.remote_sends_sdp_cb = None
    self.create_offer_cb = None
    self.webrtc_ready = False
    self.session_ready = False


    self.node.get_logger().info('using node_id "' + str(self.node_id) + '"')
    self.node.get_logger().info('using peer_id "' + str(self.peer_id) + '"')
    self.node.get_logger().info('using server "' + self.server + '"')




  def fetch_params(self, param_prefix):
    if param_prefix != '':
      param_prefix = param_prefix + '.'
    self.node.declare_parameters(
      namespace='',
      parameters=[
        (param_prefix + 'signalling_server', ''),
        (param_prefix + 'node_id', 0),
        (param_prefix + 'peer_id', 0),
        (param_prefix + 'offer', True),
        (param_prefix + 'autodial', True)
      ]
    )
    server =    self.node.get_parameter(param_prefix + 'signalling_server').value
    node_id =   self.node.get_parameter(param_prefix + 'node_id').value
    peer_id =   self.node.get_parameter(param_prefix + 'peer_id').value
    offer =     self.node.get_parameter(param_prefix + 'offer').value
    autodial =  self.node.get_parameter(param_prefix + 'autodial').value
    return (server, node_id, peer_id, offer, autodial)


  def connect_callbacks(self, _remote_sends_ice_cb, _remote_sends_sdp_cb, _create_offer_cb=None):
    self.remote_sends_ice_cb = _remote_sends_ice_cb
    self.remote_sends_sdp_cb = _remote_sends_sdp_cb
    self.create_offer_cb = _create_offer_cb #used for autodialling
    self.node.get_logger().debug('transport connected callbacks')


  def diagnostic_task(self, stat):
    stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'sig_transport')
    stat.add('server connected', str(self.conn != None))
    return stat


  def connect(self):
    self.loop.create_task(self.async_task())

  async def async_task(self):
    sslctx = ssl.create_default_context(purpose=ssl.Purpose.CLIENT_AUTH)
    self.node.get_logger().debug('connecting to server')
    self.conn = await websockets.connect(self.server, ssl=sslctx)
    assert self.conn
    self.node.get_logger().debug('registering with server')
    await self.conn.send('HELLO {}'.format(self.node_id))
    async for message in self.conn:
      self.node.get_logger().debug('remote says "' + message + '"')
      if message == 'HELLO':
        await self.find_peer()
      elif message == 'SESSION_OK':
        self.node.get_logger().debug('session ok')
        self.session_ready = True
        self.start_call()
      elif message.startswith('ERROR'):
        self.node.get_logger().error('remote says "' + message + '"')
      else:
        self.handle_sdp(message)
        self.session_ready = False
    self.node.get_logger().error('transport closed socket')


  async def find_peer(self):
    if self.peer_id != None:
      self.node.get_logger().debug('locating peer')
      await self.conn.send('SESSION {}'.format(self.peer_id))


  def pipeline_ready(self, ready=True):
    self.webrtc_ready = ready
    self.start_call()


  def start_call(self):
    if self.autodial and self.webrtc_ready and self.session_ready:
      if self.offer:
        if self.create_offer_cb:
          self.create_offer_cb()
      else:
        self.request_offer()


  def request_offer(self):
    if self.peer_id != None:
      self.loop.create_task(self.conn.send('OFFER_REQUEST {}'.format(self.peer_id)))


  async def stop(self):
    if self.conn:
      await self.conn.close()
    self.conn = None





  def send_sdp(self, local_sdp):
    sdp_msg_str = local_sdp.sdp.as_text()
    sdp_type_str = GstWebRTC.WebRTCSDPType.to_string(local_sdp.type)

    msg = json.dumps({'sdp': {'type': sdp_type_str, 'sdp': sdp_msg_str}})
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

