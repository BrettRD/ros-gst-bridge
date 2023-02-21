#include <gst_pipes_plugins.h>

/*

appsrc and appsink elements are handled by classes that bind callbacks to
  both the gstreamer elements, and the ros node interface.

aravissrc will benefit from a plugin that exposes gig-e values

webrtc may need a complex plugin that dynamically re-configures the pipeline.


*/



namespace gst_pipes
{


  gst_pipes_plugin::gst_pipes_plugin(){}

  void gst_pipes_plugin::initialise(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_if,
    GstElement * pipeline,
    GstElement * elem)
  {}

  void gst_pipes_appsink::initialise(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_if,
    GstElement * pipeline,
    GstElement * elem)
  {}

  void gst_pipes_appsink::frame_cb(/* ideally the gstreamer buffer*/)
  {}



  //void gst_pipes_appsrc::initialise(
  //  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_if,
  //  GstElement * pipeline,
  //  GstElement * elem)
  //{}

  void gst_pipes_appsrc::frame_cb(/* the ros image message / audio message */)
  {}


} // namespace gst_pipes



#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipes::gst_pipes_appsink, gst_pipes::gst_pipes_plugin)
PLUGINLIB_EXPORT_CLASS(gst_pipes::gst_pipes_appsrc, gst_pipes::gst_pipes_plugin)
