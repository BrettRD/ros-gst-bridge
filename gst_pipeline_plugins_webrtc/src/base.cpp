#include <base.h>
#include <datachannel.h>

#include <fstream>  // writing sdp messages as debug output

namespace gst_pipeline_plugins_webrtc
{
void base::initialise(
  std::string name,  // the config name of the plugin
  std::shared_ptr<gst_bridge::node_interface_collection> node_if,
  GstPipeline * pipeline)
{
  name_ = name;
  node_if_ = node_if;
  pipeline_ = pipeline;

  elem_name_ = node_if->parameters->declare_parameter(
    name_ + ".element_name",
    rclcpp::ParameterValue("mysrc"),
    descr(
      "the name of the source element inside the pipeline",
      true)
    ).get<std::string>();

  audio_sink_descr_ = node_if->parameters->declare_parameter(
    name_ + ".audio_sink_descr",
    rclcpp::ParameterValue("audioconvert ! audioresample ! autoaudiosink"),
    descr(
      "the audio sink to process webrtc output",
      true
    )
  ).get<std::string>();

  video_sink_descr_ = node_if->parameters->declare_parameter(
    name_ + ".video_sink_descr",
    rclcpp::ParameterValue("videoconvert ! autovideosink"),
    descr(
      "the video sink to process webrtc output",
      true
    )
  ).get<std::string>();


  audio_loop_sink_ = node_if->parameters->declare_parameter(
    name_ + ".audio_loop_sink",
    rclcpp::ParameterValue(""),
    descr(
      "used if the audio_sink_descr feeds back to the peer, this selects the element where it is fed back",
      true
    )
  ).get<std::string>();

  video_loop_sink_ = node_if->parameters->declare_parameter(
    name_ + ".video_loop_sink",
    rclcpp::ParameterValue(""),
    descr(
      "used if the video_sink_descr feeds back to the peer, this selects the element where it is fed back",
      true
    )
  ).get<std::string>();

  generate_debug_files_ = node_if->parameters->declare_parameter(
    name_ + ".generate_debug_files",
    rclcpp::ParameterValue(false),
    descr(
      "generate dotfiles when the pipeline changes, and write sdp messages to file",
      true
    )
  ).get<bool>();



  if (GST_IS_BIN(pipeline_)) {
    GstElement * bin = gst_bin_get_by_name(GST_BIN_CAST(pipeline_), elem_name_.c_str());
    if (bin) {
      RCLCPP_INFO(
        node_if->logging->get_logger(), "plugin gst_pipes_webrtc '%s' found '%s'",
        name_.c_str(), elem_name_.c_str());

      // XXX test if bin_ is a webrtcbin
      //if(GST_IS_WEBRTCBIN(bin)){
        webrtc_ = GST_BIN_CAST(bin);
      //}


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


      // hook to the async "message" signal emitted by the bus
      GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE (pipeline_));
      g_signal_connect (bus, "message", (GCallback) base::gst_bus_cb, static_cast<gpointer>(this));
      gst_object_unref(bus);

      g_signal_connect(webrtc_, "on-data-channel", G_CALLBACK(on_data_channel_cb), this);

      init_signalling_server_client();

    }

    else {
      RCLCPP_ERROR(
        node_if_->logging->get_logger(),
        "plugin gst_pipes_webrtc '%s' failed to locate a gstreamer element called '%s'",
        name_.c_str(),
        elem_name_.c_str()
      );
    }
  } else {
    RCLCPP_ERROR(
      node_if_->logging->get_logger(),
      "plugin gst_pipes_webrtc '%s' received invalid pipeline in initialisation",
      name_.c_str()
    );
  }
}





// ############ virtual methods for different signalling servers ############

// connect to your signalling server
void base::init_signalling_server_client(){
  RCLCPP_ERROR(
    node_if_->logging->get_logger(),
    "init_signalling_server_client called on base class"
  );
}

// default implementation
void base::begin_negotiate(){
  RCLCPP_ERROR(
    node_if_->logging->get_logger(),
    "begin_negotiate called on base class"
  );
}


// called when the webrtcbin wants to send a SDP answer
// default calls  send_sdp(descr)
void base::send_sdp_answer(
  GstWebRTCSessionDescription * answer
){
  RCLCPP_INFO(
    node_if_->logging->get_logger(),
    "Sending sdp answer"
  );

  if(generate_debug_files_){
    std::ofstream sdp_file;
    sdp_file.open ("sdp_sent_answer.txt", std::ofstream::app);
    sdp_file << "type: \n";
    sdp_file << gst_webrtc_sdp_type_to_string(answer->type);
    sdp_file << "\n";
    sdp_file << "message: \n";
    sdp_file << gst_sdp_message_as_text(answer->sdp);
    sdp_file << "\n";
    sdp_file.close();
  }

  send_sdp(answer);
}

// called when the webrtcbin is instructed to send a sdp offer
// default calls  send_sdp(descr)
void base::send_sdp_offer(
  GstWebRTCSessionDescription * offer
){
  RCLCPP_INFO(
    node_if_->logging->get_logger(),
    "Sending sdp offer"
  );

  if(generate_debug_files_){
    std::ofstream sdp_file;
    sdp_file.open ("sdp_sent_offer.txt", std::ofstream::app);
    sdp_file << "type: \n";
    sdp_file << gst_webrtc_sdp_type_to_string(offer->type);
    sdp_file << "\n";
    sdp_file << "message: \n";
    sdp_file << gst_sdp_message_as_text(offer->sdp);
    sdp_file << "\n";
    sdp_file.close();
  }

  send_sdp(offer);
}

// send a sdp description to the remote server
void base::send_sdp(
  GstWebRTCSessionDescription * desc
){
  (void) desc;
}

// send an ice candidat to the remote server
void base::send_ice_candidate(
  guint mline_index,
  gchararray candidate
){
  (void) mline_index;
  (void) candidate;
}





// ############### C style callbacks for the webrtc async background logic ###############

void
base::on_negotiation_needed_cb(
  GstElement * object,
  gpointer user_data
){
  (void) object;
  base* this_ptr = (base*) user_data;

  RCLCPP_INFO(
    this_ptr->node_if_->logging->get_logger(),
    "Starting negotiate"
  );


  // XXX conditionally open the outbound data channel

  // the data channel needs to be part of the SDP sent to the browser
  //  and needs to be created after the pipeline has reached a ready state

  // data channels requires gstreamer 1.18 or higher
  GstWebRTCDataChannel * dc = NULL; // dc will be stored in on_data_channel_cb
  g_signal_emit_by_name(this_ptr->webrtc_, "create-data-channel",
    this_ptr->name_.c_str(),  // label
    NULL, // options
    &dc // return value
  );
  if (dc){
    RCLCPP_ERROR(
      this_ptr->node_if_->logging->get_logger(),
      "local created data channel"
    );
    std::shared_ptr<datachannel_handler> handler = std::make_shared<datachannel_handler_string_topic>();  // default construct
    handler->initialise(this_ptr, dc);
    this_ptr->data_channels_.push_back(std::move(handler));
  }
  else
  {
    RCLCPP_ERROR(
      this_ptr->node_if_->logging->get_logger(),
      "Could not create tx data channel, is usrsctp available?"
    );
  }


  this_ptr->begin_negotiate();
}


void
base::on_ice_candidate_cb(
  GstElement * object,
  guint mline_index,
  gchararray candidate,
  gpointer user_data
){
  (void) object;
  base* this_ptr = (base*) user_data;

  //RCLCPP_INFO(
  //  this_ptr->node_if_->logging->get_logger(),
  //  "Created ice candidate"
  //);

  this_ptr->send_ice_candidate(mline_index, candidate);
}


void
base::on_notify_ice_gathering_state_cb(
  GstElement *webrtcbin,
  GParamSpec *pspec,
  gpointer user_data
){
  (void) webrtcbin;
  (void) pspec;
  
  base* this_ptr = (base*) user_data;

  GstWebRTCICEGatheringState ice_gather_state;
  const gchar *new_state = "unknown";

  g_object_get (this_ptr->webrtc_, "ice-gathering-state", &ice_gather_state, NULL);
  switch (ice_gather_state) {
    case GST_WEBRTC_ICE_GATHERING_STATE_NEW:
      new_state = "new";
      break;
    case GST_WEBRTC_ICE_GATHERING_STATE_GATHERING:
      new_state = "gathering";
      break;
    case GST_WEBRTC_ICE_GATHERING_STATE_COMPLETE:
      new_state = "complete";
      break;
  }



  RCLCPP_INFO(
    this_ptr->node_if_->logging->get_logger(),
    "Ice gathering state: %s",
    new_state
  );
}



void
base::on_incoming_decodebin_stream(
  GstElement * decodebin,
  GstPad * pad,
  gpointer user_data
){
  (void) decodebin; // unused
  base* this_ptr = (base*) user_data;

  GstCaps *caps;
  const gchar *caps_name;
  GstElement* sink_bin = NULL;
  std::string sink_bin_descr;
  bool synced = true;
  caps = gst_pad_get_current_caps (pad);
  caps_name = gst_structure_get_name (gst_caps_get_structure (caps, 0));
  std::string loop_sink_name;

  if (g_str_has_prefix (caps_name, "video"))
  {
    sink_bin_descr = this_ptr->video_sink_descr_;
    loop_sink_name = this_ptr->video_loop_sink_;

  }
  else if (g_str_has_prefix (caps_name, "audio"))
  {
    sink_bin_descr = this_ptr->audio_sink_descr_;
    loop_sink_name = this_ptr->audio_loop_sink_;
  }
  else
  {
    RCLCPP_ERROR(
      this_ptr->node_if_->logging->get_logger(),
      "on_incoming_decodebin_stream: Unknown pad %s, ignoring, caps_name = '%s'",
      GST_PAD_NAME (pad),
      caps_name
    );
    return;
  }

  RCLCPP_INFO(
    this_ptr->node_if_->logging->get_logger(),
    "Adding output '%s'",
    sink_bin_descr.c_str()
  );
  sink_bin = gst_parse_bin_from_description(sink_bin_descr.c_str(), true, NULL);
  gst_bin_add(GST_BIN_CAST(this_ptr->pipeline_), sink_bin);
  synced &= gst_element_sync_state_with_parent(sink_bin);

  //gst_element_link(decodebin, sink_bin);
  GstPad * sinkpad = gst_element_get_static_pad (sink_bin, "sink");
  gst_pad_link (pad, sinkpad);
  gst_object_unref (sinkpad);

  // Optionally reconfigure the pipeline to form a loop
  // test if the newly created sink bin has a dangling pad
  GstPad *src_pad = gst_element_get_static_pad (sink_bin, "src");
  if(src_pad){
    // locate the destination element
    GstElement* loop_bin = gst_bin_get_by_name_recurse_up(
      this_ptr->webrtc_, loop_sink_name.c_str());
    // set a callback to connect the loop, and destruct the upstream elements

    if(loop_bin){
      // XXX C style memory management is ugly here
      pad_swap_args_t* pad_swap_args = (pad_swap_args_t*) malloc(sizeof(pad_swap_args_t));
      pad_swap_args->this_ptr = this_ptr;
      pad_swap_args->new_src = src_pad;
      pad_swap_args->sink_pad = gst_element_get_static_pad(loop_bin, "sink");
      pad_swap_args->old_src = gst_pad_get_peer(pad_swap_args->sink_pad);


      gst_pad_add_probe(
        src_pad,  // catch the pad immediately
        GST_PAD_PROBE_TYPE_IDLE,
        (GstPadProbeCallback)base::gst_pad_swap_cb,
        static_cast<gpointer>(pad_swap_args),
        NULL
      );
    }
    else
    {
      RCLCPP_ERROR(
        this_ptr->node_if_->logging->get_logger(),
        "on_incoming_decodebin_stream: sink pipeline has dangling pads, but loop element '%s' was not found",
        loop_sink_name.c_str()
      );
    }

  }

  // synced &= gst_bin_sync_children_states(sink_bin);
  if(!synced) {
    RCLCPP_ERROR(
      this_ptr->node_if_->logging->get_logger(),
      "on_incoming_decodebin_stream: could not synchronise '%s' with the webrtcbin",
      sink_bin_descr.c_str()
    );
  }
}

GstPadProbeReturn
base::gst_pad_swap_cb(
  GstPad * pad,
  GstPadProbeInfo * info,
  gpointer user_data
){
  (void) pad; //unused
  (void) info; //unused

  GstPadProbeReturn ret;
  base* this_ptr = (static_cast<pad_swap_args_t*>(user_data))->this_ptr;
  GstPad* new_src = (static_cast<pad_swap_args_t*>(user_data))->new_src;
  GstPad* sink_pad = (static_cast<pad_swap_args_t*>(user_data))->sink_pad;
  GstPad* old_src = (static_cast<pad_swap_args_t*>(user_data))->old_src;
  free(user_data);


  // XXX we probably need to use a blocking probe on sink_pad before we make the switch

  // swap the streams over
  bool success = true;
  success &= gst_pad_unlink(old_src, sink_pad);
  success &= gst_pad_link (new_src, sink_pad);

  // XXX The shutdown and removal should happen async (glib idle task)
  // shutdown and remove the old bin
  GstElement* old_bin = gst_pad_get_parent_element(old_src);
  gst_element_set_state(old_bin, GST_STATE_NULL);
  gst_bin_remove(GST_BIN_CAST(this_ptr->pipeline_), old_bin);

  ret = GST_PAD_PROBE_REMOVE;
  return ret;
}

void
base::pad_added_cb(
  GstElement *webrtc,
  GstPad *pad,
  gpointer user_data
){
  (void) webrtc;
  base* this_ptr = (base*) user_data;

  GstElement *decodebin;
  GstPad *sinkpad;

  if (GST_PAD_DIRECTION (pad) != GST_PAD_SRC)
    return;

  RCLCPP_INFO(
    this_ptr->node_if_->logging->get_logger(),
    "Adding decodebin"
  );

  decodebin = gst_element_factory_make ("decodebin", NULL);
  g_signal_connect (decodebin, "pad-added",
      G_CALLBACK (on_incoming_decodebin_stream), this_ptr);
  gst_bin_add (GST_BIN (this_ptr->pipeline_), decodebin);
  gst_element_sync_state_with_parent (decodebin);

  sinkpad = gst_element_get_static_pad (decodebin, "sink");
  gst_pad_link (pad, sinkpad);
  gst_object_unref (sinkpad);


}


// ############### async logic - local sends a SDP offer ###############

// signal the webrtcbin to create an offer,
// take the offer from the the webrtcbin and send it to the remote peer via the signalling layer
// accept a sdp answer back 



// entry point to establishing a call
void
base::create_offer()
{
  RCLCPP_INFO(
    node_if_->logging->get_logger(),
    "Creating sdp offer"
  );
  GstPromise *promise =
    gst_promise_new_with_change_func (create_offer_prom, this, NULL);
  g_signal_emit_by_name (webrtc_, "create-offer", NULL, promise);
}


// on_offer_created
// unpack the promise and retrieve the sdp offer, calling sdp_received with the result
void
base::create_offer_prom (GstPromise * promise, gpointer user_data)
{
  base* this_ptr = (base*) user_data;

  RCLCPP_INFO(
    this_ptr->node_if_->logging->get_logger(),
    "Created sdp offer"
  );

  GstWebRTCSessionDescription *offer = NULL;
  const GstStructure *reply;

  // retrieve the offer from the promise
  g_assert_cmphex (gst_promise_wait (promise), ==, GST_PROMISE_RESULT_REPLIED);
  reply = gst_promise_get_reply (promise);
  gst_structure_get (reply, "offer",
      GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &offer, NULL);
  gst_promise_unref (promise);

  // set the local description
  // XXX this is probably not needed, or needs to be broken out for overloading
  promise = gst_promise_new ();
  g_signal_emit_by_name (this_ptr->webrtc_, "set-local-description", offer, promise);
  gst_promise_interrupt (promise);
  gst_promise_unref (promise);

  // Send offer to peer
  this_ptr->send_sdp_offer (offer);
  gst_webrtc_session_description_free (offer);

}

// after sending the offer, we expect the client link to return a sdp answer
// convenience overload to unpack the GstSDPMessage to a GstWebRTCSessionDescription
void
base::sdp_answer_received (GstSDPMessage * sdp)
{
  GstWebRTCSessionDescription *answer = NULL;

  answer = gst_webrtc_session_description_new (GST_WEBRTC_SDP_TYPE_ANSWER, sdp);
  sdp_answer_received(answer);
}

void
base::sdp_answer_received (GstWebRTCSessionDescription * answer)
{
  g_assert_nonnull (answer);
  
  RCLCPP_INFO(
    node_if_->logging->get_logger(),
    "Recieved sdp answer"
  );

  if(generate_debug_files_){
    std::ofstream sdp_file;
    sdp_file.open ("sdp_received_answer.txt", std::ofstream::app);
    sdp_file << "type: \n";
    sdp_file << gst_webrtc_sdp_type_to_string(answer->type);
    sdp_file << "\n";
    sdp_file << "message: \n";
    sdp_file << gst_sdp_message_as_text(answer->sdp);
    sdp_file << "\n";
    sdp_file.close();
  }

  GstPromise *promise = gst_promise_new ();
  g_signal_emit_by_name (webrtc_, "set-remote-description", answer,
    promise);
  gst_promise_interrupt (promise);
  gst_promise_unref (promise);

}

// ############### async logic - remote sent a SDP offer ###############


  // sdp_offer_received sends signal "set-remote-description"
  // whose promise calls set_remote_description_prom
  // which emits signal g_signal_emit_by_name(webrtc_, "create-answer", NULL, promise);
  // whose promise calls create_answer_prom
  // which emits signal g_signal_emit_by_name(webrtc_, "set-local-description", answer, promise);
  // whose promise can finally terminate the call tree

// called when the remote peer sends a SDP offer,
// this will be called by a ros topic message in the default plugin
// convenience overload to unpack the GstSDPMessage to a GstWebRTCSessionDescription

void
base::sdp_offer_received (GstSDPMessage * sdp)
{
  GstWebRTCSessionDescription *offer = NULL;

  offer = gst_webrtc_session_description_new (GST_WEBRTC_SDP_TYPE_OFFER, sdp);

  /* Set remote description on our pipeline */
  sdp_offer_received(offer);
  gst_webrtc_session_description_free (offer);
}



// called when the remote peer sends a SDP offer,
// this will be called by a ros topic message in the default plugin
void
base::sdp_offer_received(
  GstWebRTCSessionDescription * offer
){
  g_assert_nonnull (offer);

  RCLCPP_INFO(
    node_if_->logging->get_logger(),
    "Recieved sdp offer"
  );

  if(generate_debug_files_){
    std::ofstream sdp_file;
    sdp_file.open ("sdp_received_offer.txt", std::ofstream::app);
    sdp_file << "type: \n";
    sdp_file << gst_webrtc_sdp_type_to_string(offer->type);
    sdp_file << "\n";
    sdp_file << "message: \n";
    sdp_file << gst_sdp_message_as_text(offer->sdp);
    sdp_file << "\n";
    sdp_file.close();
  }

  GstPromise * promise = gst_promise_new_with_change_func(set_remote_description_prom, this, NULL);

  g_signal_emit_by_name(webrtc_, "set-remote-description", offer, promise);

}

void 
base::set_remote_description_prom(
  GstPromise *promise,
  gpointer user_data
){
  base* this_ptr = (base*) user_data;
  gst_promise_unref(promise);

  RCLCPP_INFO(
    this_ptr->node_if_->logging->get_logger(),
    "Remote desciption set"
  );

  promise = gst_promise_new_with_change_func(create_answer_prom, (gpointer) this_ptr, NULL);
  g_signal_emit_by_name(this_ptr->webrtc_, "create-answer", NULL, promise);

}


void 
base::create_answer_prom(
  GstPromise *promise,
  gpointer user_data
){
  base* this_ptr = (base*) user_data;

  GstWebRTCSessionDescription *answer = NULL;
  const GstStructure *reply;

  RCLCPP_INFO(
    this_ptr->node_if_->logging->get_logger(),
    "Created sdp answer"
  );

  // XXX sanity check the state of the promise

  // extract answer from finished promise
  reply = gst_promise_get_reply(promise);
  gst_structure_get(reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &answer, NULL);
  gst_promise_unref(promise);

  // Is this needed?
  g_signal_emit_by_name(this_ptr->webrtc_, "set-local-description", answer, NULL);

  // XXX send answer to remote peer,
  this_ptr->send_sdp_answer(answer);
}

// ############### async background: ice candidates ###############


// called when the remote peer sends an ice candidate,
// this will be called by a ros topic message in the default plugin
void
base::ice_candidate_received(
  guint mline_index,          // the index of the media description in the SDP
  const gchar* ice_candidate    // an ice candidate or NULL/"" to mark that no more candidates will arrive
){

  //RCLCPP_INFO(
  //  node_if_->logging->get_logger(),
  //  "Received ice candidate"// '%s'",
  //  //ice_candidate
  //);
  g_signal_emit_by_name(webrtc_, "add-ice-candidate", mline_index, ice_candidate);
}


// ############### data channel callbacks ###############

// remote peer has offered a data channel to send us data
void
base::on_data_channel_cb(
  GstElement * object,
  GstWebRTCDataChannel * channel,
  gpointer user_data
){
  (void) object;
  base* this_ptr = (base*) user_data;

  GValue dc_label_val = {0,0};
  g_object_get_property((GObject*)channel, "label", &dc_label_val);
  std::string label(g_value_get_string(&dc_label_val));
  RCLCPP_INFO(
    this_ptr->node_if_->logging->get_logger(),
    "webrtcbin created a data channel with label '%s'",
    label.c_str()
  );

  // XXX choose the type of handler based on label and config
  std::shared_ptr<datachannel_handler> handler = std::make_shared<datachannel_handler_string_topic>();  // default construct

  handler->initialise(this_ptr, channel);
  this_ptr->data_channels_.push_back(std::move(handler));
}


gboolean base::gst_bus_cb(GstBus* bus, GstMessage* message, gpointer user_data)
{
  (void)bus;
  auto* this_ptr = static_cast<base*>(user_data);


  // XXX check that the message comes from our webrtc element
  //const GstStructure* s;
  //s = gst_message_get_structure(message);
  //if (0 == g_strcmp0(gst_structure_get_name(s), "GstWebrtcbin")) {  // XXX it's not called that
  //if (0 == g_strcmp0(GST_OBJECT_NAME (message->src), this_ptr->elem_name_.c_str())) {


  switch (GST_MESSAGE_TYPE (message)) {
    case GST_MESSAGE_ASYNC_DONE:
    {
      RCLCPP_INFO(
        this_ptr->node_if_->logging->get_logger(),
        "Async done"
      );
      GST_DEBUG_BIN_TO_DOT_FILE_WITH_TS (GST_BIN (this_ptr->pipeline_),
          GST_DEBUG_GRAPH_SHOW_ALL, "webrtc-sendrecv.async-done");
      break;
    }
    case GST_MESSAGE_ERROR:
    {
      GError *error = NULL;
      gchar *debug = NULL;

      GST_DEBUG_BIN_TO_DOT_FILE_WITH_TS (GST_BIN (this_ptr->pipeline_),
          GST_DEBUG_GRAPH_SHOW_ALL, "webrtc-sendrecv.error");

      gst_message_parse_error (message, &error, &debug);
      //cleanup_and_quit_loop ("ERROR: Error on bus", APP_STATE_ERROR);
      //g_warning ("Error on bus: %s (debug: %s)", error->message, debug);
      RCLCPP_ERROR(
        this_ptr->node_if_->logging->get_logger(),
        "Error on bus: %s (debug: %s)",
        error->message,
        debug
      );
      g_error_free (error);
      g_free (debug);
      break;
    }
    case GST_MESSAGE_WARNING:
    {
      GError *error = NULL;
      gchar *debug = NULL;

      gst_message_parse_warning (message, &error, &debug);
      RCLCPP_WARN(
        this_ptr->node_if_->logging->get_logger(),
        "Warning on bus: %s (debug: %s)",
        error->message,
        debug
      );
      g_error_free (error);
      g_free (debug);
      break;
    }
    case GST_MESSAGE_LATENCY:
      gst_bin_recalculate_latency (GST_BIN (this_ptr->pipeline_));
      RCLCPP_INFO(
        this_ptr->node_if_->logging->get_logger(),
        "Latency message"
      );
      break;
    default:
      break;
  }

  return G_SOURCE_CONTINUE;
}


}  // namespace gst_pipeline_plugins_webrtc

