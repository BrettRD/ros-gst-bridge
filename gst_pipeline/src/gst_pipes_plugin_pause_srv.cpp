#include <gst_pipes_plugin_pause_srv.h>

/*

Play/Pause:
  provide a ROS service with a Gazebo style play/pause API

*/

namespace gst_pipes
{
void gst_pipes_pause_srv::initialise(
  std::string name,  // the config name of the plugin
  std::shared<gst_bridge::node_interface_collection> node_if, GstElement * pipeline)
{
  name_ = name;
  node_if_ = node_if;
  pipeline_ = pipeline;

  if (GST_IS_BIN(pipeline_)) {
    // automatically start the pipeline
  }
}

void gst_pipes_pause_srv::service_cb(/* gazebo's service type*/) {}

}  // namespace gst_pipes

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipes::gst_pipes_pause_srv, gst_pipes::gst_pipes_plugin)
