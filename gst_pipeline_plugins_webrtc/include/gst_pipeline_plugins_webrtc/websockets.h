#ifndef GST_PIPELINE_PLUGINS_WEBRTC__WEBSOCKETS_H_
#define GST_PIPELINE_PLUGINS_WEBRTC__WEBSOCKETS_H_

#include <base.h>


// For signalling 
#include <libsoup/soup.h>  // gir1.2-soup-2.4 libsoup2.4-dev
#include <json-glib/json-glib.h>

#include <std_msgs/msg/string.hpp>
#include <gst_msgs/msg/webrtc_ice.hpp>
#include "rclcpp/rclcpp.hpp"

namespace gst_pipeline_plugins_webrtc
{
/*
  This plugin specialises base to use ros topics as the peer discovery mechanism


*/
class websockets : public base
{
public:


  // ############ overide methods for signalling transport ############

  void init_signalling_server_client() override;

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


void connect_to_websocket_server_async();

static void on_server_connected(
  SoupSession * session,
  GAsyncResult * res,
  gpointer user_data);

gboolean register_with_server();

gboolean setup_call();

static void reset_connection (
  websockets * this_ptr,
  const gchar * msg,
  enum AppState state
);


static void on_server_closed(
  SoupWebsocketConnection * conn,
  gpointer user_data);

static void on_server_message(
  SoupWebsocketConnection * conn,
  SoupWebsocketDataType type,
  GBytes * message,
  gpointer user_data
);


  gchar* server_url;
  bool disable_ssl;
  
  gchar* our_id;
  gchar* peer_id;
  AppState app_state;
  bool remote_is_offerer;
  bool local_is_offerer;
  static SoupWebsocketConnection *ws_conn;


};

}  // namespace gst_pipeline_plugins_webrtc

#endif  //GST_PIPELINE_PLUGINS_WEBRTC__WEBSOCKETS_H_
