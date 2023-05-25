#include <gst_pipes_plugin_webrtc.h>

namespace gst_pipes
{
void gst_pipes_webrtc::initialise(
  std::string name,  // the config name of the plugin
  std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline)
{
  name_ = name;
  node_if_ = node_if;
  pipeline_ = pipeline;
  cb_data.this_ptr = this;
  cb_data.trigger = false;

  elem_name_ = node_if->parameters
                 ->declare_parameter(
                   name + ".element_name", rclcpp::ParameterValue("mysrc"),
                   descr("the name of the source element inside the pipeline", true))
                 .get<std::string>();

  if (GST_IS_BIN(pipeline_)) {
    GstElement * bin = gst_bin_get_by_name(GST_BIN_CAST(pipeline_), elem_name_.c_str());
    if (bin) {
      RCLCPP_INFO(
        node_if->logging->get_logger(), "plugin gst_pipes_webrtc '%s' found '%s'",
        name_.c_str(), elem_name_.c_str());

      // XXX test if bin_ is a webrtcbin
      webrtc_ = bin;


      // webrtc peer discovery server comms:
      // webrtc_on_negotiation_needed is called when webrtc goes to playing
      g_signal_connect(webrtc_, "on-negotiation-needed", G_CALLBACK(on_negotiation_needed_cb), this);

      // gstreamer will compute and emit ICE candidates, these need to be sent to the remote server
      g_signal_connect(webrtc_, "on-ice-candidate", G_CALLBACK(on_ice_candidate_cb), this);

      // the remote server will send messages back, these need to be forwarded to webrtcbin


      // this is a monitoring call for quality of life, it tells us where in the ICE candidate step we're up to
      g_signal_connect(webrtc_, "notify::ice-gathering-state", G_CALLBACK(on_notify_ice_gathering_state_cb), this);


      // connecting calls:
      // webrtc will add new pads for audio or video, these need decoding, it's easy to just plug a decodebin into here
      g_signal_connect(webrtc_, "pad-added", G_CALLBACK(pad_added_cb), this);

      /* data channels requires gstreamer 1.18 or higher
      g_signal_connect(webrtc_, "on-data-channel", G_CALLBACK(on_data_channel_cb), this);

      
      // data channels can be used to create tunnels for ROS serialised transports

      // the remote peer is offering a low latency data channel 
      // we can offer a low latency data channel back
      g_signal_emit_by_name(webrtc_, "create-data-channel", "channel", NULL, &data_channel_tx);



      */


    }

    else {
      RCLCPP_ERROR(
        node_if->logging->get_logger(),
        "plugin gst_pipes_webrtc '%s' failed to locate a gstreamer element called '%s'",
        name_.c_str(), elem_name_.c_str());
    }
  } else {
    RCLCPP_ERROR(
      node_if->logging->get_logger(),
      "plugin gst_pipes_webrtc '%s' received invalid pipeline in initialisation",
      name_.c_str());
  }
}


// XXX needs logic to disconnect pads, delete elements, and connect new pads.
// this is important for cases where you want to loop the webrtc output back to the input
GstPadProbeReturn
gst_pipes_webrtc::gst_pad_probe_cb(
  GstPad * pad,
  GstPadProbeInfo * info,
  gpointer user_data
){
  gst_pipes_webrtc* this_ptr = (gst_pipes_webrtc*) user_data;
  // XXX remove the source bin,
  // XXX attach the appropriate new src bin
  GstPadProbeReturn ret;
  ret = GST_PAD_PROBE_OK;
  return ret;
}

// 
void
gst_pipes_webrtc::on_negotiation_needed_cb(
  GstElement * object,
  gpointer udata
){
  gst_pipes_webrtc* this_ptr = (gst_pipes_webrtc*) user_data;
}

void
gst_pipes_webrtc::on_ice_candidate_cb(
  GstElement * object,
  guint mline_index,
  gchararray candidate,
  gpointer udata
){
  gst_pipes_webrtc* this_ptr = (gst_pipes_webrtc*) user_data;
  // our webrtcbin is ready to send an ice candidate to the remote peer,
  // package and publish the string
}

void
gst_pipes_webrtc::on_notify_ice_gathering_state_cb(
  GstElement *webrtcbin,
  GParamSpec *pspec,
  gpointer user_data
){
  gst_pipes_webrtc* this_ptr = (gst_pipes_webrtc*) user_data;
  // this is purely a monitoring call, just print to debug, and maybe send a ros diagnostics update
}

void
gst_pipes_webrtc::pad_added_cb(
  GstElement *webrtc,
  GstPad *pad,
  gpointer user_data
){
  gst_pipes_webrtc* this_ptr = (gst_pipes_webrtc*) user_data;
  // XXX test if the pad is audio or video, then build and attach the appropriate sink bin
  // optionally shim a decodebin in for convenience
}



// called when the remote peer has a SDP answer,
// this will be called by a ros topic message in the default plugin
void
gst_pipes_webrtc::sdp_received(
  GstWebRTCSessionDescription * desc,   //a GstWebRTCSessionDescription description
){

  GstPromise * promise = gst_promise_new_with_change_func(set_remote_description_prom, this, NULL);

  g_signal_emit_by_name(webrtc_, "set-remote-description", desc, promise);
  // XXX
  // promise calls set_remote_description_prom
  // which emits signal g_signal_emit_by_name(webrtc_, "create-answer", NULL, promise);
  // whose promise calls create_answer_prom
  // which emits signal g_signal_emit_by_name(webrtc_, "set-local-description", answer, promise);
  // whose promise can finally terminate the call tree

}

void 
gst_pipes_webrtc::set_remote_description_prom(
  GstPromise *promise,
  gpointer user_data
){
  gst_pipes_webrtc* this_ptr = (gst_pipes_webrtc*) user_data;
  gst_promise_unref(promise);


  promise = gst_promise_new_with_change_func(create_answer_prom, (gpointer) this_ptr, NULL);
  g_signal_emit_by_name(webrtc_, "create-answer", NULL, promise);

}
void 
gst_pipes_webrtc::create_answer_prom(
  GstPromise *promise,
  gpointer user_data
){
  gst_pipes_webrtc* this_ptr = (gst_pipes_webrtc*) user_data;

  GstWebRTCSessionDescription *answer = NULL;
  const GstStructure *reply;

  // XXX sanity check the state of the promise

  // extract answer from finished promise
  reply = gst_promise_get_reply(promise);
  gst_structure_get(reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &answer, NULL);
  gst_promise_unref(promise);

  // Is this needed?
  g_signal_emit_by_name(webrtc_, "set-local-description", answer, NULL);

  // send answer to remote peer,
  this_ptr->publish_sdp_answer(answer);

}



// called when the remote peer sends an ice candidate,
// this will be called by a ros topic message in the default plugin
void
gst_pipes_webrtc::ice_received(
  guint mline_index,          // the index of the media description in the SDP
  gchararray ice_candidate,   // an ice candidate or NULL/"" to mark that no more candidates will arrive
){
  g_signal_emit_by_name(webrtc_, "add-ice-candidate", mline_index, ice_candidate);
}


// remote peer has offered a data channel to send us data
void
gst_pipes_webrtc::on_data_channel_cb(
  GstElement * object,
  GstWebRTCDataChannel * channel,
  gpointer user_data
){
  gst_pipes_webrtc* this_ptr = (gst_pipes_webrtc*) user_data;
  this_ptr->data_channel_rx_ = channel; // XXX use a setter method

  // events associated with data channels:
  g_signal_connect(channel, "on-error", G_CALLBACK(data_channel_on_error_cb), this_ptr);
  g_signal_connect(channel, "on-open", G_CALLBACK(data_channel_on_open_cb), this_ptr);
  g_signal_connect(channel, "on-close", G_CALLBACK(data_channel_on_close_cb), this_ptr);
  g_signal_connect(channel, "on-message-data ", G_CALLBACK(data_channel_on_message_data_cb), this_ptr);
}

void
gst_pipes_webrtc::data_channel_on_message_data_cb(
  GstWebRTCDataChannel * self,
  GBytes * data,
  gpointer user_data
){
  gst_pipes_webrtc* this_ptr = (gst_pipes_webrtc*) user_data;
}

void
gst_pipes_webrtc::data_channel_on_open_cb(
  GstWebRTCDataChannel * self,
  gpointer user_data
){
  gst_pipes_webrtc* this_ptr = (gst_pipes_webrtc*) user_data;
}

void
gst_pipes_webrtc::data_channel_on_error_cb(
  GstWebRTCDataChannel * self,
  GError * error,
  gpointer user_data
){
  gst_pipes_webrtc* this_ptr = (gst_pipes_webrtc*) user_data;
}

void
gst_pipes_webrtc::data_channel_on_close_cb(
  GstWebRTCDataChannel * self,
  gpointer user_data
){
  gst_pipes_webrtc* this_ptr = (gst_pipes_webrtc*) user_data;
}
















}  // namespace gst_pipes

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipes::gst_pipes_webrtc, gst_pipes::gst_pipes_plugin)
