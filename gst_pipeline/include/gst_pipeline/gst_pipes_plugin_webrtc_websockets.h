#ifndef GST_PIPELINE__GST_PIPES_PLUGIN_WEBRTC_WEBSOCKETS_H_
#define GST_PIPELINE__GST_PIPES_PLUGIN_WEBRTC_WEBSOCKETS_H_

#include <gst_pipes_webrtc_base.h>

#include <std_msgs/msg/string.hpp>
#include <gst_msgs/msg/webrtc_ice.hpp>

#include "rclcpp/rclcpp.hpp"

namespace gst_pipes
{
/*
  This plugin specialises gst_pipes_webrtc_base to use ros topics as the peer discovery mechanism


*/
class gst_pipes_webrtc_websockets : public gst_pipes_webrtc_base
{
public:


  // ############ overide methods for signalling transport ############

  void init_signalling_server_client() override

  void begin_negotiate() override;

  // send a sdp description to the remote server
  void send_sdp(GstWebRTCSessionDescription * desc) override;

  // send an ice candidat to the remote server
  void send_ice_candidate(guint mline_index, gchararray candidate) override;


private:




  enum AppState
  {
    APP_STATE_UNKNOWN = 0,
    APP_STATE_ERROR = 1,          /* generic error */
    SERVER_CONNECTING = 1000,
    SERVER_CONNECTION_ERROR,
    SERVER_CONNECTED,             /* Ready to register */
    SERVER_REGISTERING = 2000,
    SERVER_REGISTRATION_ERROR,
    SERVER_REGISTERED,            /* Ready to call a peer */
    SERVER_CLOSED,                /* server connection closed by us or the server */
    PEER_CONNECTING = 3000,
    PEER_CONNECTION_ERROR,
    PEER_CONNECTED,
    PEER_CALL_NEGOTIATING = 4000,
    PEER_CALL_STARTED,
    PEER_CALL_STOPPING,
    PEER_CALL_STOPPED,
    PEER_CALL_ERROR,
  };


  



};

}  // namespace gst_pipes

#endif  //GST_PIPELINE__GST_PIPES_PLUGIN_WEBRTC_WEBSOCKETS_H_
