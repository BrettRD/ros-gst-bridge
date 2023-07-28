#ifndef GST_PIPELINE__GST_PIPES_PLUGIN_WEBRTC_ROS_H_
#define GST_PIPELINE__GST_PIPES_PLUGIN_WEBRTC_ROS_H_

#include <base.h>

#include <std_msgs/msg/string.hpp>
#include <gst_msgs/msg/webrtc_ice.hpp>

#include "rclcpp/rclcpp.hpp"

namespace gst_pipes
{
/*
  This plugin specialises gst_pipes_webrtc_base to use ros topics as the peer discovery mechanism


*/
class gst_pipes_webrtc_ros : public gst_pipes_webrtc_base
{
public:


  // ############ overide methods for signalling transport ############

  void gst_pipes_webrtc_ros::init_signalling_server_client()

  // send a sdp description to the remote server
  void send_sdp(GstWebRTCSessionDescription * desc) override;

  // send an ice candidat to the remote server
  void send_ice_candidate(guint mline_index, gchararray candidate) override;


private:
  // ############ subscription callbacks for incoming signalling events ############

  void
  gst_pipes_webrtc_ros::sdp_rx_topic_cb(
    const std_msgs::msg::String::SharedPtr msg);

  void
  gst_pipes_webrtc_ros::ice_rx_topic_cb(
    const std_msgs::msg::String::SharedPtr msg);


  // publishers and subscribers for peer discovery

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sdp_rx;
  rclcpp::Subscription<gst_msgs::msg::WebrtcIce>::SharedPtr ice_rx;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sdp_tx;
  rclcpp::Publisher<gst_msgs::msg::WebrtcIce>::SharedPtr ice_tx;


};

}  // namespace gst_pipes

#endif  //GST_PIPELINE__GST_PIPES_PLUGIN_WEBRTC_ROS_H_
