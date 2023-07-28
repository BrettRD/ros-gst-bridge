#include <gst_pipes_plugin_webrtc_ros.h>

namespace gst_pipes
{


// ############ override virtual methods, signal with ros topics ############

void gst_pipes_webrtc_ros::init_signalling_server_client()
{

  // XXX collect from parameter server:
  //  sdp_tx_topic, ice_tx_topic, sdp_rx_topic, ice_rx_topic


  rclcpp::QoS qos = rclcpp::SensorDataQoS().reliable();


  sdp_tx = rclcpp::create_publisher<std_msgs::msg::String>(
    node_if->parameters, node_if->topics, sdp_tx_topic, qos);
  ice_tx = rclcpp::create_publisher<gst_msgs::msg::WebrtcIce>(
    node_if->parameters, node_if->topics, ice_tx_topic, qos);


  auto cb = [src](std_msgs::msg::String::ConstSharedPtr msg) { rostextsrc_sub_cb(src, msg); };

  sdp_rx = rclcpp::create_subscription<std_msgs::msg::String>(
    node_if->parameters, node_if->topics, sdp_rx_topic, qos, 
    std::bind(&gst_pipes_webrtc_ros::sdp_rx_topic_cb, this, _1));

}


void gst_pipes_webrtc_ros::send_sdp(
  GstWebRTCSessionDescription * desc
){
  std_msgs::msg::String msg;
  msg.data = descr;
  sdp_tx->publish(msg);
}

void send_ice_candidate(
guint mline_index,
gchararray candidate)
{
  gst_msgs::msg::WebrtcIce msg;
  msg.mline_index = mline_index;
  msg.candidate = candidate;
  ice_tx->publish(msg);
}


void gst_pipes_webrtc_ros::sdp_rx_topic_cb(
  const std_msgs::msg::String::SharedPtr msg
){
  RCLCPP_INFO(this->get_logger(), "recievd sdp");

  GstWebRTCSessionDescription * desc
  desc = msg.data();  // XXX packaging

  sdp_received(desc);
}

void gst_pipes_webrtc_ros::ice_rx_topic_cb(
  const std_msgs::msg::String::SharedPtr msg
){
  RCLCPP_INFO(this->get_logger(), "received ice candidate");
  guint mline_index;
  gchararray ice_candidate;

  mline_index = msg->mline_index;
  ice_candidate = msg->candidate;

  ice_candidate_received(mline_index, ice_candidate);
}






}  // namespace gst_pipes

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipes::gst_pipes_webrtc_ros, gst_pipes::gst_pipes_plugin)
