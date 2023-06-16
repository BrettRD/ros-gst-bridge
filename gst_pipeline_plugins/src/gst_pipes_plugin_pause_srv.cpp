#include <gst_pipes_plugin_pause_srv.h>

/*

Play/Pause:
  provide a ROS service with a Gazebo style play/pause API

*/

namespace gst_pipes
{
void gst_pipes_pause_srv::initialise(
  std::string name,  // the config name of the plugin
  std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline)
{
  name_ = name;
  node_if_ = node_if;
  pipeline_ = pipeline;

  if (GST_IS_BIN(pipeline_)) {
    // automatically start the pipeline
  }

  // Something like the gazebo_ros_pkgs gazebo_ros_init plugin
  pause_service_ = rclcpp::create_service<std_srvs::srv::Empty>(
    node_if_->base, node_if_->services, "pause",
    std::bind(
      &gst_pipes_pause_srv::pause_srv_cb, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), nullptr);

  play_service_ = rclcpp::create_service<std_srvs::srv::Empty>(
    node_if_->base, node_if_->services, "play",
    std::bind(
      &gst_pipes_pause_srv::play_srv_cb, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), nullptr);

  seek_service_ = rclcpp::create_service<gst_msgs::srv::Seek>(
    node_if_->base, node_if_->services, "seek",
    std::bind(
      &gst_pipes_pause_srv::seek_srv_cb, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), nullptr);
}

void gst_pipes_pause_srv::pause_srv_cb(
  std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr)
{
  GstStateChangeReturn state_return = gst_element_set_state(pipeline_, GST_STATE_PAUSED);
  if (GST_STATE_CHANGE_FAILURE == state_return) {
    RCLCPP_ERROR(node_if_->logging->get_logger(), "Failed to pause stream");
  }
}

void gst_pipes_pause_srv::play_srv_cb(
  std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr)
{
  GstStateChangeReturn state_return = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (GST_STATE_CHANGE_FAILURE == state_return) {
    RCLCPP_ERROR(node_if_->logging->get_logger(), "Failed to play stream");
  }
}

void gst_pipes_pause_srv::seek_srv_cb(
  gst_msgs::srv::Seek::Request::SharedPtr req, gst_msgs::srv::Seek::Response::SharedPtr res)
{
  gint64 seek_pos_ns = req->seek_time;

  gboolean seek_return;
  seek_return = gst_element_seek_simple(
    pipeline_, GST_FORMAT_TIME, (GstSeekFlags)(GST_SEEK_FLAG_FLUSH | GST_SEEK_FLAG_KEY_UNIT),
    seek_pos_ns);

  if (false == seek_return) {
    RCLCPP_ERROR(
      node_if_->logging->get_logger(), "Failed to seek the pipeline, are we running live?");
  }

  res->success = seek_return;
  res->message = "";
}

}  // namespace gst_pipes

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipes::gst_pipes_pause_srv, gst_pipes::gst_pipes_plugin)
