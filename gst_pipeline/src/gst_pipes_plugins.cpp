#include <gst_pipes_plugins.h>

/*

appsrc and appsink elements are handled by classes that bind callbacks to
  both the gstreamer elements, and the ros node interface.

aravissrc will benefit from a plugin that exposes gig-e values

webrtc may need a complex plugin that dynamically re-configures the pipeline.


*/



namespace gst_pipes
{


  void gst_pipes_appsink::initialise(
    std::string name, // the config name of the plugin
    node_interface_collection node_if,
    GstElement * pipeline)
  {
    name_ = name;
    node_if_ = node_if;
    pipeline_= pipeline;

    appsink_name_ = node_if.param-> declare_parameter(
      name+".appsink_name",
      rclcpp::ParameterType::PARAMETER_STRING,
      descr("the name of the appsink element inside the pipeline", true)
    ).get<std::string>();

    RCLCPP_INFO(
      node_if.log->get_logger(),
      "Binding gst_pipes_appsink '%s' to gstreamer element '%s'",
      name_.c_str(),
      appsink_name_.c_str()
    );

    // XXX search for the element in pipeline_

  }

  void gst_pipes_appsink::frame_cb(/* ideally the gstreamer buffer*/)
  {}






  void gst_pipes_appsrc::initialise(
    std::string name, // the config name of the plugin
    node_interface_collection node_if,
    GstElement * pipeline)
  {
    name_ = name;
    node_if_ = node_if;
    pipeline_= pipeline;

    appsrc_name_ = node_if.param-> declare_parameter(
      name+".appsrc_name",
      rclcpp::ParameterType::PARAMETER_STRING,
      descr("the name of the appsrc element inside the pipeline", true)
    ).get<std::string>();

    RCLCPP_INFO(
      node_if.log->get_logger(),
      "Binding gst_pipes_appsrc '%s' to gstreamer element '%s'",
      name_.c_str(),
      appsrc_name_.c_str()
    );

    // XXX search for the element in pipeline_

  }

  void gst_pipes_appsrc::frame_cb(/* the ros image message / audio message */)
  {}


} // namespace gst_pipes



#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipes::gst_pipes_appsink, gst_pipes::gst_pipes_plugin)
PLUGINLIB_EXPORT_CLASS(gst_pipes::gst_pipes_appsrc, gst_pipes::gst_pipes_plugin)
