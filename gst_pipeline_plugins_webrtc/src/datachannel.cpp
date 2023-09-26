


/*
https://gstreamer.freedesktop.org/documentation/webrtclib/gstwebrtc-datachannel.html?gi-language=c


The data channel for WebRTC transports unstructured data at low latency.
This module should be sub-classed, maybe even pluggable.


It would be nice to use the data channel to move ros messages to web apps,
It would be nice to re-use any ser/des logic on either end of any gstreamer tunnel


rosbridge and foxglove bridge both use a strongly assymetric pattern designed
  to act as an agent, allowing the remote to request subscribers and publishers from local.

mqtt bridge seems to preserve symmetry 
https://github.com/ika-rwth-aachen/mqtt_client/blob/main/mqtt_client/src/MqttClient.ros2.cpp

*/

#include <datachannel.h>

namespace gst_pipeline_plugins_webrtc
{

void
datachannel_handler::data_channel_sub_cb(const std_msgs::msg::String::SharedPtr msg)
{
  g_signal_emit_by_name (channel_, "send-string", msg->data.c_str());
}

void
datachannel_handler::init (
  base* base,
  GstWebRTCDataChannel * data_channel
){
  base_ = base;
  channel_ = data_channel;

  GValue dc_label_val = {0,0};
  g_object_get_property((GObject*)channel_, "label", &dc_label_val);
  std::string label(g_value_get_string(&dc_label_val));

  RCLCPP_INFO(
    base->node_if_->logging->get_logger(),
    "Connecting signals for data channel '%s'",
    label.c_str()
  );

  g_signal_connect (channel_, "on-error", G_CALLBACK (data_channel_on_error_cb), this);
  g_signal_connect (channel_, "on-open", G_CALLBACK (data_channel_on_open_cb), this);
  g_signal_connect (channel_, "on-close", G_CALLBACK (data_channel_on_close_cb), this);
  g_signal_connect (channel_, "on-message-data", G_CALLBACK (data_channel_on_message_data_cb), this);
  g_signal_connect (channel_, "on-message-string", G_CALLBACK (data_channel_on_message_string_cb), this);
}


void
datachannel_handler::data_channel_on_message_data_cb(
  GstWebRTCDataChannel * channel,
  GBytes * data,
  gpointer user_data
){
  datachannel_handler* this_ptr = (datachannel_handler*) user_data;

  gsize msg_size = 0;
  const char* msg_data = (const char*) g_bytes_get_data (data, &msg_size);
  std::string msg_str = std::string(msg_data, msg_size);


  GValue dc_label = {0,0};
  g_object_get_property((GObject*)channel, "label", &dc_label);

  RCLCPP_INFO(
    this_ptr->base_->node_if_->logging->get_logger(),
    "Data channel %s received a data message %s",
    g_value_get_string(&dc_label),
    msg_str.c_str()
  );
}


void
datachannel_handler::data_channel_on_message_string_cb(
  GstWebRTCDataChannel * channel,
  gchar * str,
  gpointer user_data
){
  datachannel_handler* this_ptr = (datachannel_handler*) user_data;

  GValue dc_label = {0,0};
  g_object_get_property((GObject*)channel, "label", &dc_label);

  RCLCPP_INFO(
    this_ptr->base_->node_if_->logging->get_logger(),
    "A data channel %s received a string message %s",
    g_value_get_string(&dc_label),
    str
  );
}


void
datachannel_handler::data_channel_on_open_cb(
  GstWebRTCDataChannel * channel,
  gpointer user_data
){
  datachannel_handler* this_ptr = (datachannel_handler*) user_data;

  GValue dc_label_val = {0,0};
  g_object_get_property((GObject*)channel, "label", &dc_label_val);
  std::string label(g_value_get_string(&dc_label_val));

  RCLCPP_INFO(
    this_ptr->base_->node_if_->logging->get_logger(),
    "A data channel opened with label '%s'",
    label.c_str()
  );


  this_ptr->data_channel_sub_ = rclcpp::create_subscription<std_msgs::msg::String> (
    this_ptr->base_->node_if_->parameters,
    this_ptr->base_->node_if_->topics,
    "~/"+this_ptr->base_->name_+"/"+label,
    10,
    std::bind(
      &datachannel_handler::data_channel_sub_cb,
      this_ptr,
      std::placeholders::_1
    )
  );

  g_signal_emit_by_name (channel, "send-string", "Hi! from GStreamer");

}


void
datachannel_handler::data_channel_on_error_cb(
  GstWebRTCDataChannel * channel,
  GError * error,
  gpointer user_data
){
  (void) error;
  datachannel_handler* this_ptr = (datachannel_handler*) user_data;

  GValue dc_label = {0,0};
  g_object_get_property((GObject*)channel, "label", &dc_label);

  RCLCPP_INFO(
    this_ptr->base_->node_if_->logging->get_logger(),
    "Error on data channel '%s'",
    g_value_get_string(&dc_label)
  );
}


void
datachannel_handler::data_channel_on_close_cb(
  GstWebRTCDataChannel * channel,
  gpointer user_data
){
  (void) channel;
  (void) user_data;
  // datachannel_handler* this_ptr = (datachannel_handler*) user_data;
}




}  // namespace gst_pipeline_plugins_webrtc

