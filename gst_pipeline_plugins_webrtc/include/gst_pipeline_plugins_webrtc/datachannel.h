#ifndef GST_PIPELINE_PLUGINS_WEBRTC__DATACHANNEL_H_
#define GST_PIPELINE_PLUGINS_WEBRTC__DATACHANNEL_H_

#include "std_msgs/msg/string.hpp"  // XXX derived class


#include <gst/gst.h>
#include <gst/sdp/sdp.h>
#define GST_USE_UNSTABLE_API
#include <gst/webrtc/webrtc.h>

namespace gst_pipeline_plugins_webrtc
{
class base; // forward declaration avoids a dependency

class datachannel_handler
{
public:
  // default-only constructor to allow pluginlib
  virtual ~datachannel_handler(){}

  virtual void initialise(base* base, GstWebRTCDataChannel * channel);

protected:
  virtual void on_message_data_cb(GstWebRTCDataChannel * self, GBytes * data);
  virtual void on_message_string_cb(GstWebRTCDataChannel * self, gchar * str);
  virtual void on_error_cb(GstWebRTCDataChannel * self, GError * error);
  virtual void on_open_cb(GstWebRTCDataChannel * self);
  virtual void on_close_cb(GstWebRTCDataChannel * self);

  // ############### C style static callbacks for datachannel logic ###############

  // the data channel has received new data
  static void
    data_channel_on_message_data_cb(
    GstWebRTCDataChannel * dc, GBytes * data, gpointer user_data);
  static void
    data_channel_on_message_string_cb(
    GstWebRTCDataChannel * dc, gchar * str, gpointer user_data);
  static void
    data_channel_on_error_cb(
    GstWebRTCDataChannel * dc, GError * error, gpointer user_data);
  static void
    data_channel_on_open_cb(
    GstWebRTCDataChannel * dc, gpointer user_data);
  static void
    data_channel_on_close_cb(
    GstWebRTCDataChannel * dc, gpointer user_data);

  // ############### convenience functions ###############
  static std::string get_label(GstWebRTCDataChannel * dc);
  // protected getters to extend the friend-class to derived classes
  static std::shared_ptr<gst_bridge::node_interface_collection>
    get_base_node_if(base* base);
  static GstPipeline* get_base_pipeline(base* base);
  static std::string get_base_name(base* base);



  // ############### base class variables ###############
  base* base_;  // the class that controls the webrtcbin and accesss the node and pipeline
  GstWebRTCDataChannel* channel_;

};


// example subscriber
class datachannel_handler_string_topic : public datachannel_handler
{
protected:
  void on_open_cb(GstWebRTCDataChannel * channel) override;
  void on_message_string_cb(GstWebRTCDataChannel * self, gchar * str);

private:
  void sub_cb(const std_msgs::msg::String::SharedPtr msg);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

};

}  // namespace gst_pipeline_plugins_webrtc

#endif  //GST_PIPELINE_PLUGINS_WEBRTC__DATACHANNEL_H_









