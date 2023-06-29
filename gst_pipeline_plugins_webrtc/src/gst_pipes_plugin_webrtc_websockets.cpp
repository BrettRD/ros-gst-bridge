#include <gst_pipes_plugin_webrtc_websockets.h>


/*
  this is a carbon copy of the gstreamer webrtc example by nirbheek for testing purposes
  https://gitlab.freedesktop.org/gstreamer/gstreamer/-/blob/main/subprojects/gst-examples/webrtc/sendrecv/gst/webrtc-sendrecv.c

*/




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



static gchar *
get_string_from_json_object (JsonObject * object)
{
  JsonNode *root;
  JsonGenerator *generator;
  gchar *text;

  /* Make it the root node */
  root = json_node_init_object (json_node_alloc (), object);
  generator = json_generator_new ();
  json_generator_set_root (generator, root);
  text = json_generator_to_data (generator, NULL);

  /* Release everything */
  g_object_unref (generator);
  json_node_free (root);
  return text;
}



void
gst_pipes_webrtc_websockets::init_signalling_server_client()
{

  // XXX collect from parameter server:
  //  server url and socket
  //    server_url = "wss://webrtc.gstreamer.net:8443";
  //  turn server address
  //  local peer ID
  //  remote peer ID


  // start a web socket to the signalling server
  connect_to_websocket_server_async();
  // announce our local peer id
  // conditionally request a session with the remote peer


}



void gst_pipes_webrtc_websockets::connect_to_websocket_server_async (void)
{
  SoupLogger *logger;
  SoupMessage *message;
  SoupSession *session;
  const char *https_aliases[] = { "wss", NULL };
  session = soup_session_new_with_options(
    SOUP_SESSION_SSL_STRICT,
    !disable_ssl,
    SOUP_SESSION_SSL_USE_SYSTEM_CA_FILE,
    TRUE,
    //SOUP_SESSION_SSL_CA_FILE, "/etc/ssl/certs/ca-bundle.crt",
    SOUP_SESSION_HTTPS_ALIASES,
    https_aliases,
    NULL
  );

  logger = soup_logger_new (SOUP_LOGGER_LOG_BODY, -1);
  soup_session_add_feature (session, SOUP_SESSION_FEATURE (logger));
  g_object_unref (logger);
  message = soup_message_new (SOUP_METHOD_GET, server_url);
  gst_print ("Connecting to server...\n");
  /* Once connected, we will register */
  soup_session_websocket_connect_async (
    session,
    message,
    NULL,
    NULL,
    NULL,
    (GAsyncReadyCallback) on_server_connected,
    message
  );

  app_state = SERVER_CONNECTING;
}


void gst_pipes_webrtc_websockets::on_server_connected(
  SoupSession * session,
  GAsyncResult * res,
  gpointer user_data )
{
  gst_pipes_webrtc_websockets* this_ptr = (gst_pipes_webrtc_websockets*) user_data;
  GError *error = NULL;
  
  this_ptr->ws_conn = soup_session_websocket_connect_finish (session, res, &error);
  if (error) {
    this_ptr->cleanup_and_quit_loop (error->message, SERVER_CONNECTION_ERROR);
    g_error_free (error);
    return;
  }
  g_assert_nonnull (this_ptr->ws_conn);
  this_ptr->app_state = SERVER_CONNECTED;
  gst_print ("Connected to signalling server\n");
  g_signal_connect (this_ptr->ws_conn, "closed", G_CALLBACK (on_server_closed), this_ptr);
  g_signal_connect (this_ptr->ws_conn, "message", G_CALLBACK (on_server_message), this_ptr);
  /* Register with the server so it knows about us and can accept commands */
  this_ptr->register_with_server ();
}


gboolean gst_pipes_webrtc_websockets::register_with_server ()
{
  gchar *hello;
  if (soup_websocket_connection_get_state (ws_conn) !=
      SOUP_WEBSOCKET_STATE_OPEN)
    return FALSE;

  gst_print ("Registering id %s with server\n", our_id);
  hello = g_strdup_printf ("HELLO %s", our_id);

  app_state = SERVER_REGISTERING;

  soup_websocket_connection_send_text (ws_conn, hello);
  g_free (hello);
  return TRUE;
}


gboolean gst_pipes_webrtc_websockets::setup_call()
{
  gchar *msg;

  if (soup_websocket_connection_get_state (ws_conn) !=
      SOUP_WEBSOCKET_STATE_OPEN)
    return FALSE;

  gst_print ("Setting up signalling server call with %s\n", peer_id);
  app_state = PEER_CONNECTING;
  msg = g_strdup_printf ("SESSION %s", peer_id);
  soup_websocket_connection_send_text (ws_conn, msg);
  g_free (msg);
  return TRUE;
}


void gst_pipes_webrtc_websockets::on_server_closed(
  SoupWebsocketConnection * conn,
  gpointer user_data)
{
  (void) conn;
  gst_pipes_webrtc_websockets* this_ptr = (gst_pipes_webrtc_websockets*) user_data;

  this_ptr->app_state = SERVER_CLOSED;
  this_ptr->cleanup_and_quit_loop ("Server connection closed", 0);
}


// called when webrtcbin is up
void gst_pipes_webrtc_websockets::begin_negotiate()
{
  if (remote_is_offerer) {
    soup_websocket_connection_send_text (ws_conn, "OFFER_REQUEST");
  } else if (local_is_offerer) {
    create_offer();
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

void
gst_pipes_webrtc_websockets::send_ice_candidate(
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
  json_object_set_int_member (ice, "sdpMLineIndex", mline_index);
  msg = json_object_new ();
  json_object_set_object_member (msg, "ice", ice);
  text = get_string_from_json_object (msg);
  json_object_unref (msg);

  soup_websocket_connection_send_text (ws_conn, text);
  g_free (text);
}


// XXX websocket traffic callback


void
gst_pipes_webrtc_websockets::on_server_message(
  SoupWebsocketConnection * conn,
  SoupWebsocketDataType type,
  GBytes * message,
  gpointer user_data
){
  (void) conn;
  gst_pipes_webrtc_websockets* this_ptr = (gst_pipes_webrtc_websockets*) user_data;
  gchar *text;

  switch (type) {
    case SOUP_WEBSOCKET_DATA_BINARY:
      gst_printerr ("Received unknown binary message, ignoring\n");
      return;
    case SOUP_WEBSOCKET_DATA_TEXT:{
      gsize size;
      const gchar* data = (const char*) g_bytes_get_data (message, &size);
      /* Convert to NULL-terminated string */
      text = g_strndup (data, size);
      break;
    }
    default:
      g_assert_not_reached ();
  }

  if (g_strcmp0 (text, "HELLO") == 0) {
    /* Server has accepted our registration, we are ready to send commands */

    if (this_ptr->app_state != SERVER_REGISTERING) {
      this_ptr->cleanup_and_quit_loop ("ERROR: Received HELLO when not registering",
          APP_STATE_ERROR);
      goto out;
    }
    this_ptr->app_state = SERVER_REGISTERED;
    gst_print ("Registered with server\n");
    if (this_ptr->peer_id) {
      // Ask signalling server to connect us with a specific peer
      if (!this_ptr->setup_call ()) {
        this_ptr->cleanup_and_quit_loop ("ERROR: Failed to setup call", PEER_CALL_ERROR);
        goto out;
      }
    } else {
      gst_println ("Waiting for connection from peer (our-id: %s)", this_ptr->our_id);
    }


  } else if (g_strcmp0 (text, "SESSION_OK") == 0) {
    /* The call initiated by us has been setup by the server; now we can start
     * negotiation */
    if (this_ptr->app_state != PEER_CONNECTING) {
      this_ptr->cleanup_and_quit_loop ("ERROR: Received SESSION_OK when not calling",
          PEER_CONNECTION_ERROR);
      goto out;
    }

    this_ptr->app_state = PEER_CONNECTED;
    /* Start negotiation (exchange SDP and ICE candidates) */
    this_ptr->create_offer();



  } else if (g_strcmp0 (text, "OFFER_REQUEST") == 0) {
    if (this_ptr->app_state != SERVER_REGISTERED) {
      gst_printerr ("Received OFFER_REQUEST at a strange time, ignoring\n");
      goto out;
    }
    gst_print ("Received OFFER_REQUEST, sending offer\n");
    this_ptr->create_offer();


  } else if (g_str_has_prefix (text, "ERROR")) {
    /* Handle errors */
    switch (this_ptr->app_state) {
      case SERVER_CONNECTING:
        this_ptr->app_state = SERVER_CONNECTION_ERROR;
        break;
      case SERVER_REGISTERING:
        this_ptr->app_state = SERVER_REGISTRATION_ERROR;
        break;
      case PEER_CONNECTING:
        this_ptr->app_state = PEER_CONNECTION_ERROR;
        break;
      case PEER_CONNECTED:
      case PEER_CALL_NEGOTIATING:
        this_ptr->app_state = PEER_CALL_ERROR;
        break;
      default:
        this_ptr->app_state = APP_STATE_ERROR;
    }
    this_ptr->cleanup_and_quit_loop (text, 0);
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

      this_ptr->app_state = PEER_CALL_NEGOTIATING;

      child = json_object_get_object_member (object, "sdp");

      if (!json_object_has_member (child, "type")) {
        this_ptr->cleanup_and_quit_loop ("ERROR: received SDP without 'type'",
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

        this_ptr->sdp_answer_received(sdp);

        this_ptr->app_state = PEER_CALL_STARTED;
      } else {
        gst_print ("Received offer:\n%s\n", text);
        this_ptr->sdp_offer_received (sdp);
      }

    } else if (json_object_has_member (object, "ice")) {
      const gchar *candidate;
      gint sdp_mline_index;

      child = json_object_get_object_member (object, "ice");
      candidate = json_object_get_string_member (child, "candidate");
      sdp_mline_index = json_object_get_int_member (child, "sdpMLineIndex");

      /* Add ice candidate sent by remote peer */
      this_ptr->ice_candidate_received(sdp_mline_index, candidate);

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