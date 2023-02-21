
#include <gst_pipes.h>
#include <pluginlib/class_loader.hpp>
#include <gst_pipes_plugins.h>


/*
gst_pipes
This node replaces the gst_pipeline python node


This node loads via ros_components, and exposes node interfaces to a
  variety of plugins.

appsrc and appsink elements are handled by classes that bind callbacks to
  both the gstreamer elements, and the ros node interface.
aravissrc will benefit from a plugin that exposes gig-e values
webrtc may need a complex plugin that dynamically re-configures the pipeline.

*/



namespace gst_pipes {



gst_pipes::gst_pipes(const rclcpp::NodeOptions & options):
  Node("gst_pipes", options)
{
  // validate and load the gstreamer plugin paths
  // read the pipeline parameter

  // read the pluginlib parameters (to handle elements)
  // instantiate the pluginlib classloader
  // load each pluginlib plugin

  // instantiate the pipeline
  // instantiate the pluginlib plugins
  // connect the callbacks
  
  // optionally wait for play/pause event (probably a subscription or service call)
  
  load_handler();
}


void gst_pipes::load_handler()
{
  pluginlib::ClassLoader<gst_pipes_plugin> loader("gst_pipeline", "gst_pipes::gst_pipes_plugin");

  std::shared_ptr<gst_pipes_plugin> element_handler = loader.createSharedInstance("gst_pipes::gst_pipes_appsink");

  
}


} // namespace gst_pipes


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gst_pipes::gst_pipes)
