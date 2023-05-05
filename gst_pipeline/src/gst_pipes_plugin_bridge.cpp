#include <gst_bridge/rosbasesink.h>
#include <gst_bridge/rosbasesrc.h>
#include <gst_pipes_plugin_bridge.h>

namespace gst_pipes
{
void gst_pipes_bridge::initialise(
  std::string name,  // the config name of the plugin
  std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline)
{
  name_ = name;
  node_if_ = node_if;
  pipeline_ = pipeline;

  elem_name_ = node_if->parameters->declare_parameter(
      name + ".element_name", rclcpp::ParameterType::PARAMETER_STRING,
      descr("the name of the appsink element inside the pipeline", true))
    .get<std::string>();

  if (GST_IS_BIN(pipeline_)) {
    bin_ = gst_bin_get_by_name(GST_BIN_CAST(pipeline_), elem_name_.c_str());
    if (bin_) {
      RCLCPP_INFO(
        node_if->logging->get_logger(), "plugin gst_pipes_bridge '%s' found '%s'", name_.c_str(),
        elem_name_.c_str());

      if (GST_IS_ROS_BASE_SINK(bin_)) {
        RCLCPP_INFO(
          node_if->logging->get_logger(),
          "plugin gst_pipes_bridge - '%s' is a sink from the gst-bridge package, connecting interfaces",
          elem_name_.c_str());
          
        RosBaseSink* ros_sink = GST_ROS_BASE_SINK_CAST(bin_);
        ros_sink->node_if = node_if_;
      }
      else if (GST_IS_ROS_BASE_SRC(bin_)) {
        RCLCPP_INFO(
          node_if->logging->get_logger(),
          "plugin gst_pipes_bridge - '%s' is a src from the gst-bridge package, connecting interfaces",
          elem_name_.c_str());
          
        RosBaseSrc* ros_src = GST_ROS_BASE_SRC_CAST(bin_);
        ros_src->node_if = node_if_;
      }
      else {
        RCLCPP_ERROR(
          node_if->logging->get_logger(),
          "plugin gst_pipes_bridge could not recognise the type of '%s'",
          elem_name_.c_str());
      }
    }

    else {
      RCLCPP_ERROR(
        node_if->logging->get_logger(),
        "plugin gst_pipes_bridge '%s' failed to locate a gstreamer element called '%s'",
        name_.c_str(), elem_name_.c_str());
    }
  } else {
    RCLCPP_ERROR(
      node_if->logging->get_logger(),
      "plugin gst_pipes_bridge '%s' received invalid pipeline in initialisation", name_.c_str());
  }
}

}  // namespace gst_pipes

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipes::gst_pipes_bridge, gst_pipes::gst_pipes_plugin)
