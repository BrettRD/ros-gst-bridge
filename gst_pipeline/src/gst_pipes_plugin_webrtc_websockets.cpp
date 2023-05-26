#include <gst_pipes_plugin_webrtc_websockets.h>

namespace gst_pipes
{


// ############ override virtual methods, signal with websockets ############

void gst_pipes_webrtc_websockets::init_signalling_server_client()
{

  // XXX collect from parameter server:
  //  server url and socket
  //  turn server address
  //  local peer ID
  //  remote peer ID

}


void gst_pipes_webrtc_websockets::send_sdp(
  GstWebRTCSessionDescription * desc
){
  // sanity check the state of the server
  // package and send the sdp payload
}

void send_ice_candidate(
guint mline_index,
gchararray candidate)
{
  // sanity check the state of the server
  // package and send the ice payload
}


// XXX websocket traffic callback





}  // namespace gst_pipes

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipes::gst_pipes_webrtc_websockets, gst_pipes::gst_pipes_plugin)
