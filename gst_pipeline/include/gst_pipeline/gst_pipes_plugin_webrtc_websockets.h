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

  void gst_pipes_webrtc_websockets::init_signalling_server_client() override

  // send a sdp description to the remote server
  void send_sdp(GstWebRTCSessionDescription * desc) override;

  // send an ice candidat to the remote server
  void send_ice_candidate(guint mline_index, gchararray candidate) override;


private:



};

}  // namespace gst_pipes

#endif  //GST_PIPELINE__GST_PIPES_PLUGIN_WEBRTC_WEBSOCKETS_H_
