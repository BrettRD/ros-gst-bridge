


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

#include <base.h>
#include <datachannel.h>

namespace gst_pipeline_plugins_webrtc
{

void
datachannel_handler::initialise (base* base, GstWebRTCDataChannel * data_channel)
{
  base_ = base;
  channel_ = data_channel;

  RCLCPP_INFO(
    base->node_if_->logging->get_logger(),
    "Connecting signals for data channel '%s'",
    get_label(channel_).c_str()
  );

  g_signal_connect (channel_, "on-error", G_CALLBACK (data_channel_on_error_cb), this);
  g_signal_connect (channel_, "on-open", G_CALLBACK (data_channel_on_open_cb), this);
  g_signal_connect (channel_, "on-close", G_CALLBACK (data_channel_on_close_cb), this);
  g_signal_connect (channel_, "on-message-data", G_CALLBACK (data_channel_on_message_data_cb), this);
  g_signal_connect (channel_, "on-message-string", G_CALLBACK (data_channel_on_message_string_cb), this);
}


// ############### overridable example callbacks for datachannel events ###############


void
datachannel_handler::on_message_data_cb(GstWebRTCDataChannel * channel, GBytes * data)
{
  gsize msg_size = 0;
  const char* msg_data = (const char*) g_bytes_get_data (data, &msg_size);
  std::string msg_str = std::string(msg_data, msg_size);

  RCLCPP_INFO(
    base_->node_if_->logging->get_logger(),
    "Data channel %s received a data message %s",
    get_label(channel).c_str(),
    msg_str.c_str()
  );
}


void
datachannel_handler::on_message_string_cb(GstWebRTCDataChannel * channel, gchar * str)
{
  RCLCPP_INFO(
    base_->node_if_->logging->get_logger(),
    "A data channel %s received a string message %s",
    get_label(channel).c_str(),
    str
  );
}


void
datachannel_handler::on_open_cb(GstWebRTCDataChannel * channel)
{
  RCLCPP_INFO(
    base_->node_if_->logging->get_logger(),
    "A data channel opened with label '%s'",
    get_label(channel).c_str()
  );
  g_signal_emit_by_name (channel, "send-string", "Hi! from ros-gst-bridge");
}


void
datachannel_handler::on_error_cb(GstWebRTCDataChannel * channel, GError * error)
{
  (void) error;

  RCLCPP_INFO(
    base_->node_if_->logging->get_logger(),
    "Error on data channel '%s'",
    get_label(channel).c_str()
  );
}


void
datachannel_handler::on_close_cb(GstWebRTCDataChannel * channel)
{
  (void) channel;
}



// ############### C style static callbacks for datachannel logic ###############


// the data channel has received new data
void
datachannel_handler::data_channel_on_message_data_cb(
  GstWebRTCDataChannel * dc, GBytes * data, gpointer user_data)
{
  ((datachannel_handler*) user_data)->on_message_data_cb(dc, data);
}

void
datachannel_handler::data_channel_on_message_string_cb(
  GstWebRTCDataChannel * dc, gchar * str, gpointer user_data)
{
  ((datachannel_handler*) user_data)->on_message_string_cb(dc, str);
}

void
datachannel_handler::data_channel_on_error_cb(
  GstWebRTCDataChannel * dc, GError * error, gpointer user_data)
{
  ((datachannel_handler*) user_data)->on_error_cb(dc, error);
}

void
datachannel_handler::data_channel_on_open_cb(GstWebRTCDataChannel * dc, gpointer user_data)
{
  ((datachannel_handler*) user_data)->on_open_cb(dc);
}

void
datachannel_handler::data_channel_on_close_cb(
  GstWebRTCDataChannel * dc, gpointer user_data)
{
  ((datachannel_handler*) user_data)->on_close_cb(dc);
}

// ############### convenience functions ###############
std::string datachannel_handler::get_label(GstWebRTCDataChannel * dc)
{
  GValue val = {0,0};
  g_object_get_property((GObject*)dc, "label", &val);
  return std::string (g_value_get_string(&val));
}

std::shared_ptr<gst_bridge::node_interface_collection>
datachannel_handler::get_base_node_if(base* base)
{
  return base->node_if_;
}
GstPipeline* datachannel_handler::get_base_pipeline(base* base)
{
  return base->pipeline_;
}
std::string datachannel_handler::get_base_name(base* base)
{
  return base->name_;
}








// ############### derived class ###############

void
datachannel_handler_string_topic::sub_cb(
  const std_msgs::msg::String::SharedPtr msg)
{
  g_signal_emit_by_name (channel_, "send-string", msg->data.c_str());
}


void
datachannel_handler_string_topic::on_message_string_cb(
  GstWebRTCDataChannel * channel, gchar * str)
{
  (void)channel;  // unused
  std_msgs::msg::String msg;
  msg.data = str;
  pub_->publish(msg);
}


void
datachannel_handler_string_topic::on_open_cb(GstWebRTCDataChannel * channel)
{
  sub_ = rclcpp::create_subscription<std_msgs::msg::String> (
    get_base_node_if(base_)->parameters,
    get_base_node_if(base_)->topics,
    "~/"+get_base_name(base_)+"/"+get_label(channel)+"_tx",
    10,
    std::bind(
      &datachannel_handler_string_topic::sub_cb,
      this,
      std::placeholders::_1
    )
  );

  pub_ = rclcpp::create_publisher<std_msgs::msg::String>(
    get_base_node_if(base_)->parameters,
    get_base_node_if(base_)->topics,
    "~/"+get_base_name(base_)+"/"+get_label(channel)+"_rx",
    10
  );

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipeline_plugins_webrtc::datachannel_handler_string_topic, gst_pipeline_plugins_webrtc::datachannel_handler)

}  // namespace gst_pipeline_plugins_webrtc

