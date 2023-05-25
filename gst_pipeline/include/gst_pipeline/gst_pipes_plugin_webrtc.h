#ifndef GST_PIPELINE__GST_PIPES_PLUGIN_WEBRTC_H_
#define GST_PIPELINE__GST_PIPES_PLUGIN_WEBRTC_H_

#include <gst_bridge/gst_bridge.h>
#include <gst_pipes_plugin_base.h>

#include <std_msgs/msg/empty.hpp>

#include "rclcpp/rclcpp.hpp"

namespace gst_pipes
{
/*
  This plugin inserts a callback into the sink pad of the target elenemt,
  conditionally dropping frames according to some traffic on a ros subscription
*/
class gst_pipes_webrtc : public gst_pipes_plugin
{
public:
  // during init, we need to
  //  fetch a param from the node for the element name of the target element
  //  search the pipeline for the target element
  //  insert a probe into the pipeline on the sink pad of the target element
  void initialise(
    std::string name,  // the config name of the plugin
    std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline);


  // This callback is used to disconnect dummy sources from the webrtc bin
  static GstPadProbeReturn
  gst_pad_probe_cb(
    GstPad * pad,
    GstPadProbeInfo * info,
    gpointer user_data
  );


  static void
  on_negotiation_needed_cb(
    GstElement * object,
    gpointer udata
  );

  static void
  on_ice_candidate_cb(
    GstElement * object,
    guint mline_index,
    gchararray candidate,
    gpointer udata
  );

  static void
  on_notify_ice_gathering_state_cb(
    GstElement *webrtcbin,
    GParamSpec *pspec,
    gpointer user_data
  );

  static void
  pad_added_cb(
    GstElement *webrtc,
    GstPad *pad,
    gpointer user_data
  );

  static void
  on_data_channel_cb(
    GstElement * object,
    GstWebRTCDataChannel * channel,
    gpointer user_data
  );

  //called when the remote peer has a SDP answer
  // thin wrapper over g_signal_emit_by_name(webrtc_, "set-remote-description", desc, promise);
  void
  sdp_received(
    // GstElement * object,                //the webrtcbin
    GstWebRTCSessionDescription * desc,   //a GstWebRTCSessionDescription description
    GstPromise * promise                  //a GstPromise to be notified when it's set 
  );

  // thin wrapper over g_signal_emit_by_name(webrtc_, "add-ice-candidate", mline_index, ice_candidate);
  void
  ice_received(
    // GstElement * object,        // the webrtcbin
    guint mline_index,          // the index of the media description in the SDP
    gchararray ice_candidate,   // an ice candidate or NULL/"" to mark that no more candidates will arrive
  );



  static void
  data_channel_on_message_data_cb(
    GstWebRTCDataChannel * self,
    GBytes * data,
    gpointer user_data
  );

  static void
  data_channel_on_open_cb(
    GstWebRTCDataChannel * self,
    gpointer user_data
  );

  static void
  data_channel_on_error_cb(
    GstWebRTCDataChannel * self,
    GError * error,
    gpointer user_data
  );

  static void
  data_channel_on_close_cb(
    GstWebRTCDataChannel * self,
    gpointer user_data
  );




private:

  // the name of the target element in the pipeline
  std::string elem_name_;

  // a pointer to the bridge elements in the pipeline
  GstElement * webrtc_;

  GstWebRTCDataChannel * data_channel_rx_;
  GstWebRTCDataChannel * data_channel_tx_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sdp_rx;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ice_rx;

  // A data type we can pass as pointer into the callback
  struct cb_user_data cb_data;
};

}  // namespace gst_pipes

#endif  //GST_PIPELINE__GST_PIPES_PLUGIN_WEBRTC_H_
