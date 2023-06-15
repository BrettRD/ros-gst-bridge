#include <gst_pipes_plugin_webrtc_websockets.h>


/*
  this is a carbon copy of the gstreamer webrtc example by nirbheek for testing purposes
  https://gitlab.freedesktop.org/gstreamer/gstreamer/-/blob/main/subprojects/gst-examples/webrtc/sendrecv/gst/webrtc-sendrecv.c

*/

// For signalling 
#include <libsoup/soup.h>
#include <json-glib/json-glib.h>


/*



The original example plays fast and loose with it's state transition table
the main flags that control the flow are
  our_id==0 : the application knows the remote peer ID at startup and will find them
  our_id!=0 : the application registers its ID and will wait for any remote peer
  remote_is_offerer :  controls which peer actually sends the sdp offer once the pipeline starts


Establish a link to the server:
  SERVER_CONNECTING is the entry point of the state machine
  This set of state transitions only happens one way,
  there's no branching except for error trapping,
  nothing else can happen until we reach SERVER_REGISTERED

  SERVER_CONNECTING
  on_server_connected
  SERVER_CONNECTED
  register_with_server  "HELLO 1234"  announce local id
  SERVER_REGISTERING
  on_server_message "HELLO"
  SERVER_REGISTERED

Peer Discovery:
  Once we've registered ourselves with the server we need to find our peer



SERVER_REGISTERED && our_id!=0
  setup_call - socket send "SESSION 4321" requesting connection to known peer
  PEER_CONNECTING
  on_server_message "SESSION_OK"
  PEER_CONNECTED
  start pipeline - remote_is_offerer==true
    socket send "OFFER_REQUEST"
  start pipeline - remote_is_offerer==false
    webrtcbin "create_offer"

SERVER_REGISTERED && our_id==0
  on_server_message "OFFER_REQUEST"
  start pipeline - remote_is_offerer=false
    webrtcbin "create_offer"
    socket send sdp offer

on_server_message recieve sdp offer
PEER_CALL_STARTED


on_server_message recieve sdp answer
PEER_CALL_NEGOTIATING
set-remote-description
PEER_CALL_STARTED






//#################

In this version, we only need to deal with the remote server,
choose to offer, answer, or request offer,
and pass the sdp and ice events to the plugin base class.

The first create-offer needs two entry-points:
  when the peer session connects and the webrtcbin is waiting (begin_negotiate has been called)
  when begin_negotiate is called and the peer session is already PEER_CONNECTED

when the socket client recieves a sdp offer, it should check if begin_negotiate has been called
if begin_negotiate has not been called, this plugin must save the offer for begin_negotiate
The begin_negotiate call should first check if we've recieved a SDP offer


all of the rest of the event logic is handled by the plugin base class.



*/

namespace gst_pipes
{


// ############ override virtual methods, signal with websockets ############

void gst_pipes_webrtc_websockets::init_signalling_server_client()
{

  // XXX collect from parameter server:
  //  server url and socket
  //    server_url = "wss://webrtc.gstreamer.net:8443";
  //  turn server address
  //  local peer ID
  //  remote peer ID



  // start a web socket to the signalling server
  // announce our local peer id
  // conditionally request a session with the remote peer


}


// called when webrtcbin is up
void begin_negotiate()
{
  if (remote_is_offerer) {
    soup_websocket_connection_send_text (ws_conn, "OFFER_REQUEST");
  } else if (create_offer) {
    GstPromise *promise =
      gst_promise_new_with_change_func (on_offer_created, NULL, NULL);
    g_signal_emit_by_name (webrtc1, "create-offer", NULL, promise);

  }
  // implicit default case: wait for a sdp offer from the remote peer

}


void gst_pipes_webrtc_websockets::send_sdp(
  GstWebRTCSessionDescription * desc
){
  gchar *text;
  JsonObject *msg, *sdp;

  // sanity check the state of the server
  if (app_state < PEER_CALL_NEGOTIATING) {
    cleanup_and_quit_loop ("Can't send SDP to peer, not in call",
        APP_STATE_ERROR);
    return;
  }


  // package and send the sdp payload
  text = gst_sdp_message_as_text (desc->sdp);
  sdp = json_object_new ();

  if (desc->type == GST_WEBRTC_SDP_TYPE_OFFER) {
    json_object_set_string_member (sdp, "type", "offer");
  } else if (desc->type == GST_WEBRTC_SDP_TYPE_ANSWER) {
    json_object_set_string_member (sdp, "type", "answer");
  } else {
    g_assert_not_reached ();
  }

  json_object_set_string_member (sdp, "sdp", text);
  g_free (text);

  msg = json_object_new ();
  json_object_set_object_member (msg, "sdp", sdp);
  text = get_string_from_json_object (msg);
  json_object_unref (msg);

  soup_websocket_connection_send_text (ws_conn, text);
  g_free (text);

}

void gst_pipes_webrtc_websockets::send_ice_candidate(
  guint mline_index,
  gchararray candidate
){
  gchar *text;
  JsonObject *ice, *msg;

  // sanity check the state of the server
  if (app_state < PEER_CALL_NEGOTIATING) {
    cleanup_and_quit_loop ("Can't send ICE, not in call", APP_STATE_ERROR);
    return;
  }

  // package and send the ice payload
  ice = json_object_new ();
  json_object_set_string_member (ice, "candidate", candidate);
  json_object_set_int_member (ice, "sdpMLineIndex", mlineindex);
  msg = json_object_new ();
  json_object_set_object_member (msg, "ice", ice);
  text = get_string_from_json_object (msg);
  json_object_unref (msg);

  soup_websocket_connection_send_text (ws_conn, text);
  g_free (text);
}


// XXX websocket traffic callback


void
on_server_message(
  SoupWebsocketConnection * conn,
  SoupWebsocketDataType type,
  GBytes * message,
  gpointer user_data
){
  gchar *text;

  switch (type) {
    case SOUP_WEBSOCKET_DATA_BINARY:
      gst_printerr ("Received unknown binary message, ignoring\n");
      return;
    case SOUP_WEBSOCKET_DATA_TEXT:{
      gsize size;
      const gchar *data = g_bytes_get_data (message, &size);
      /* Convert to NULL-terminated string */
      text = g_strndup (data, size);
      break;
    }
    default:
      g_assert_not_reached ();
  }

  if (g_strcmp0 (text, "HELLO") == 0) {
    /* Server has accepted our registration, we are ready to send commands */

    if (app_state != SERVER_REGISTERING) {
      cleanup_and_quit_loop ("ERROR: Received HELLO when not registering",
          APP_STATE_ERROR);
      goto out;
    }
    app_state = SERVER_REGISTERED;
    gst_print ("Registered with server\n");
    if (!our_id) {
      /* Ask signalling server to connect us with a specific peer */
      if (!setup_call ()) {
        cleanup_and_quit_loop ("ERROR: Failed to setup call", PEER_CALL_ERROR);
        goto out;
      }
    } else {
      gst_println ("Waiting for connection from peer (our-id: %s)", our_id);
    }


  } else if (g_strcmp0 (text, "SESSION_OK") == 0) {
    /* The call initiated by us has been setup by the server; now we can start
     * negotiation */
    if (app_state != PEER_CONNECTING) {
      cleanup_and_quit_loop ("ERROR: Received SESSION_OK when not calling",
          PEER_CONNECTION_ERROR);
      goto out;
    }

    app_state = PEER_CONNECTED;
    /* Start negotiation (exchange SDP and ICE candidates) */
    if (!start_pipeline (TRUE, RTP_OPUS_DEFAULT_PT, RTP_VP8_DEFAULT_PT))
      cleanup_and_quit_loop ("ERROR: failed to start pipeline",
          PEER_CALL_ERROR);



  } else if (g_strcmp0 (text, "OFFER_REQUEST") == 0) {
    if (app_state != SERVER_REGISTERED) {
      gst_printerr ("Received OFFER_REQUEST at a strange time, ignoring\n");
      goto out;
    }
    gst_print ("Received OFFER_REQUEST, sending offer\n");
    /* Peer wants us to start negotiation (exchange SDP and ICE candidates) */
    if (!start_pipeline (TRUE, RTP_OPUS_DEFAULT_PT, RTP_VP8_DEFAULT_PT))
      cleanup_and_quit_loop ("ERROR: failed to start pipeline",
          PEER_CALL_ERROR);


  } else if (g_str_has_prefix (text, "ERROR")) {
    /* Handle errors */
    switch (app_state) {
      case SERVER_CONNECTING:
        app_state = SERVER_CONNECTION_ERROR;
        break;
      case SERVER_REGISTERING:
        app_state = SERVER_REGISTRATION_ERROR;
        break;
      case PEER_CONNECTING:
        app_state = PEER_CONNECTION_ERROR;
        break;
      case PEER_CONNECTED:
      case PEER_CALL_NEGOTIATING:
        app_state = PEER_CALL_ERROR;
        break;
      default:
        app_state = APP_STATE_ERROR;
    }
    cleanup_and_quit_loop (text, 0);
  } else {
    /* Look for JSON messages containing SDP and ICE candidates */
    JsonNode *root;
    JsonObject *object, *child;
    JsonParser *parser = json_parser_new ();
    if (!json_parser_load_from_data (parser, text, -1, NULL)) {
      gst_printerr ("Unknown message '%s', ignoring\n", text);
      g_object_unref (parser);
      goto out;
    }

    root = json_parser_get_root (parser);
    if (!JSON_NODE_HOLDS_OBJECT (root)) {
      gst_printerr ("Unknown json message '%s', ignoring\n", text);
      g_object_unref (parser);
      goto out;
    }

    object = json_node_get_object (root);
    /* Check type of JSON message */
    if (json_object_has_member (object, "sdp")) {
      int ret;
      GstSDPMessage *sdp;
      const gchar *text, *sdptype;
      GstWebRTCSessionDescription *answer;

      app_state = PEER_CALL_NEGOTIATING;

      child = json_object_get_object_member (object, "sdp");

      if (!json_object_has_member (child, "type")) {
        cleanup_and_quit_loop ("ERROR: received SDP without 'type'",
            PEER_CALL_ERROR);
        goto out;
      }

      sdptype = json_object_get_string_member (child, "type");
      /* In this example, we create the offer and receive one answer by default,
       * but it's possible to comment out the offer creation and wait for an offer
       * instead, so we handle either here.
       *
       * See tests/examples/webrtcbidirectional.c in gst-plugins-bad for another
       * example how to handle offers from peers and reply with answers using webrtcbin. */
      text = json_object_get_string_member (child, "sdp");
      ret = gst_sdp_message_new (&sdp);
      g_assert_cmphex (ret, ==, GST_SDP_OK);
      ret = gst_sdp_message_parse_buffer ((guint8 *) text, strlen (text), sdp);
      g_assert_cmphex (ret, ==, GST_SDP_OK);

      if (g_str_equal (sdptype, "answer")) {
        gst_print ("Received answer:\n%s\n", text);
        answer = gst_webrtc_session_description_new (GST_WEBRTC_SDP_TYPE_ANSWER,
            sdp);
        g_assert_nonnull (answer);

        /* Set remote description on our pipeline */
        {
          GstPromise *promise = gst_promise_new ();
          g_signal_emit_by_name (webrtc1, "set-remote-description", answer,
              promise);
          gst_promise_interrupt (promise);
          gst_promise_unref (promise);
        }
        app_state = PEER_CALL_STARTED;
      } else {
        gst_print ("Received offer:\n%s\n", text);
        on_offer_received (sdp);
      }

    } else if (json_object_has_member (object, "ice")) {
      const gchar *candidate;
      gint sdpmlineindex;

      child = json_object_get_object_member (object, "ice");
      candidate = json_object_get_string_member (child, "candidate");
      sdpmlineindex = json_object_get_int_member (child, "sdpMLineIndex");

      /* Add ice candidate sent by remote peer */
      g_signal_emit_by_name (webrtc1, "add-ice-candidate", sdpmlineindex,
          candidate);
    } else {
      gst_printerr ("Ignoring unknown JSON message:\n%s\n", text);
    }
    g_object_unref (parser);
  }

out:
  g_free (text);
}







}  // namespace gst_pipes

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipes::gst_pipes_webrtc_websockets, gst_pipes::gst_pipes_plugin)
