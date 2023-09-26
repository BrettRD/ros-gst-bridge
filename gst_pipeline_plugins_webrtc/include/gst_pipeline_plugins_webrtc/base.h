#ifndef GST_PIPELINE_PLUGINS_WEBRTC__BASE_H_
#define GST_PIPELINE_PLUGINS_WEBRTC__BASE_H_

#include <gst_bridge/gst_bridge.h>
#include <gst_pipeline/plugin_base.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"  // XXX data channel

#include <gst/gst.h>
#include <gst/sdp/sdp.h>
#define GST_USE_UNSTABLE_API
#include <gst/webrtc/webrtc.h>

namespace gst_pipeline_plugins_webrtc
{
/*
  This plugin manages the connection between a webrtcbin and its peer discovery mechanism


*/
class datachannel_handler;

class base : public gst_pipeline::plugin_base
{

// allow data_channel derived types to interact with node_if_ and pipeline_
friend class datachannel_handler;

public:
  // during init, we need to

  //  fetch parameters describing the src and sink bins/pads to connect when webrtc goes online
  //    audio_sink_descr
  //    video_sink_descr
  
  //  fetch parameters indicating if we should:
  //    dialout automatically
  //    pick up automatically

  //  fetch a param from the node for the element name of our webrtcbin
  //  search the pipeline for the webrtcbin
  //  connect callbacks to the webrtcbin
  protected:

  // XXX lacking the imagination to do this elegantly
  struct pad_swap_args_t{
    base* this_ptr;
    GstPad* new_src;
    GstPad* sink_pad;
    GstPad* old_src;
  };


  void initialise(
    std::string name,  // the config name of the plugin
    std::shared_ptr<gst_bridge::node_interface_collection> node_if,
    GstPipeline * pipeline
  );


  // ############ virtual methods for different signalling servers ############

  // connect to your signalling server
  virtual void init_signalling_server_client();

  // we're ready to start a call, or receive incoming calls
  virtual void begin_negotiate();

  // create a sdp offer, and begin the call stack to send it to the remote peer
  virtual void create_offer();

  // called when the webrtcbin is instructed to send a sdp offer
  // default calls  send_sdp(offer)
  virtual void send_sdp_offer(GstWebRTCSessionDescription * offer);

  // called when the webrtcbin wants to send a SDP answer
  // default calls  send_sdp(answer)
  virtual void send_sdp_answer(GstWebRTCSessionDescription * answer);

  // send a sdp description to the remote server
  virtual void send_sdp(GstWebRTCSessionDescription * desc) = 0;

  // send an ice candidat to the remote server
  virtual void send_ice_candidate(guint mline_index, gchararray candidate) = 0;


  // ############# handlers for remote peer events #############

  // to be called by the implementation when the remote peer has a SDP answer
  
  // thin wrapper over g_signal_emit_by_name(webrtc_, "set-remote-description", desc, promise);
  void sdp_offer_received (GstSDPMessage * sdp);
  void sdp_offer_received(GstWebRTCSessionDescription * offer);

  // thin wrapper over g_signal_emit_by_name(webrtc_, "set-remote-description", desc, promise);
  void sdp_answer_received (GstSDPMessage * sdp);
  void sdp_answer_received(GstWebRTCSessionDescription * answer);

  // thin wrapper over g_signal_emit_by_name(webrtc_, "add-ice-candidate", mline_index, ice_candidate);
  void ice_candidate_received(
    guint mline_index,          // the index of the media description in the SDP
    const gchar* ice_candidate   // an ice candidate or NULL/"" to mark that no more candidates will arrive
  );



  // ############# Callbacks for establishing a webrtc connection #############

  // called when webrtcbin wants to start connecting with the remote peer.
  static void
  on_negotiation_needed_cb(
    GstElement * object,
    gpointer udata
  );

  // called when the webrtcbin has generated an ICE candidate to send to the remote peer
  static void
  on_ice_candidate_cb(
    GstElement * object,
    guint mline_index,
    gchararray candidate,
    gpointer udata
  );

  // diagnostic only,
  // connected to a notify event that tells us how we're progressing to a NAT punching config
  static void
  on_notify_ice_gathering_state_cb(
    GstElement *webrtcbin,
    GParamSpec *pspec,
    gpointer user_data
  );



  // ########## static callbacks for waiting on functions in the webrtcbin ##########

  // gst promise that is called after set_remote_description completes.
  // this asks the bin to create a SDP answer,
  // the answer will be passed to create_answer_prom
  static void 
  set_remote_description_prom(
    GstPromise *promise,
    gpointer user_data
  );

  // gst promise that is called after the bin creates a SDP offer,
  static void
  create_offer_prom(
    GstPromise * promise,
    gpointer user_data
  );


  // gst promise that is called after the bin creates a SDP answer,
  static void 
  create_answer_prom(
    GstPromise *promise,
    gpointer user_data
  );

  // This callback is used to add sinks to the webrtcbin
  static void
  on_incoming_decodebin_stream(
    GstElement * decodebin,
    GstPad * pad,
    gpointer user_data
  );

  // This callback is used to add auto decoders to the webrtcbin
  static void
  pad_added_cb(
    GstElement * webrtc,
    GstPad * pad,
    gpointer user_data
  );

  // This callback is used to dynamically alter the pipeline after pad_added_cb
  static GstPadProbeReturn
  gst_pad_swap_cb(
    GstPad * pad,
    GstPadProbeInfo * info,
    gpointer user_data
  );


  // ###################### Callbacks for data channels ######################

  // either local or remote has opened a data channel, 
  static void
  on_data_channel_cb(
    GstElement * object,
    GstWebRTCDataChannel * channel,
    gpointer user_data
  );


  // ###################### Callbacks for housekeeping ######################

  static gboolean
  gst_bus_cb(
    GstBus* bus,
    GstMessage* message,
    gpointer user_data
  );



protected:

  // the name of the target element in the pipeline
  std::string elem_name_;
  std::string audio_sink_descr_;  // the audio sink to create on pickup
  std::string video_sink_descr_;  // the video sink to create on pickup
  std::string audio_loop_sink_;   // the element to feed audio back into
  std::string video_loop_sink_;   // the element to feed video back into
  bool generate_debug_files_;   // generate dotfiles and write sdp messages to file

  // a pointer to the bridge elements in the pipeline
  GstBin * webrtc_;

  std::vector<std::shared_ptr<datachannel_handler> > data_channels_;


};

}  // namespace gst_pipeline_plugins_webrtc

#endif  //GST_PIPELINE_PLUGINS_WEBRTC__BASE_H_
