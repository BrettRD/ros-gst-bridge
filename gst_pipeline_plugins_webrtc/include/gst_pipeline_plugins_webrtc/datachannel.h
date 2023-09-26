#ifndef GST_PIPELINE_PLUGINS_WEBRTC__DATACHANNEL_H_
#define GST_PIPELINE_PLUGINS_WEBRTC__DATACHANNEL_H_

#include <base.h>

namespace gst_pipeline_plugins_webrtc
{

class datachannel_handler
{
public:
  datachannel_handler():base_(NULL),channel_(NULL){}

  void
  init(
    base* base,
    GstWebRTCDataChannel * channel
  );


private:

  // XXX example subscriber, move this to a derived class
  void data_channel_sub_cb(
    const std_msgs::msg::String::SharedPtr msg
  );

  // the data channel has received new data
  static void
  data_channel_on_message_data_cb(
    GstWebRTCDataChannel * self,
    GBytes * data,
    gpointer user_data
  );

  static void
  data_channel_on_message_string_cb(
    GstWebRTCDataChannel * self,
    gchar * str,
    gpointer user_data
  );

  static void
  data_channel_on_error_cb(
    GstWebRTCDataChannel * self,
    GError * error,
    gpointer user_data
  );

  static void
  data_channel_on_open_cb(
    GstWebRTCDataChannel * self,
    gpointer user_data
  );

  static void
  data_channel_on_close_cb(
    GstWebRTCDataChannel * self,
    gpointer user_data
  );

  base* base_;  // the class that controls the webrtcbin and accesss the node and pipeline
  GstWebRTCDataChannel* channel_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr data_channel_sub_;

};


}  // namespace gst_pipeline_plugins_webrtc

#endif  //GST_PIPELINE_PLUGINS_WEBRTC__DATACHANNEL_H_









