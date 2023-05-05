#include <gst_pipes_plugin_app_elements.h>

namespace gst_pipes
{
void gst_pipes_appsink::initialise(
  std::string name,  // the config name of the plugin
  std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline)
{
  name_ = name;
  node_if_ = node_if;
  pipeline_ = pipeline;

  appsink_name_ = node_if->parameters
                    ->declare_parameter(
                      name + ".appsink_name", rclcpp::ParameterType::PARAMETER_STRING,
                      descr("the name of the appsink element inside the pipeline", true))
                    .get<std::string>();

  if (GST_IS_BIN(pipeline_)) {
    sink_ = gst_bin_get_by_name(GST_BIN_CAST(pipeline_), appsink_name_.c_str());
    if (sink_) {
      RCLCPP_INFO(
        node_if->logging->get_logger(), "plugin gst_pipes_appsink '%s' found '%s'", name_.c_str(),
        appsink_name_.c_str());
    }

    else {
      RCLCPP_ERROR(
        node_if->logging->get_logger(),
        "plugin gst_pipes_appsink '%s' failed to locate a gstreamer element called '%s'",
        name_.c_str(), appsink_name_.c_str());
    }
  } else {
    RCLCPP_ERROR(
      node_if->logging->get_logger(),
      "plugin gst_pipes_appsink '%s' received invalid pipeline in initialisation", name_.c_str());
  }
}

void gst_pipes_appsink::frame_cb(/* ideally the gstreamer buffer*/) {}

void gst_pipes_appsrc::initialise(
  std::string name,  // the config name of the plugin
  std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline)
{
  name_ = name;
  node_if_ = node_if;
  pipeline_ = pipeline;

  appsrc_name_ = node_if->parameters
                   ->declare_parameter(
                     name + ".appsrc_name", rclcpp::ParameterType::PARAMETER_STRING,
                     descr("the name of the appsrc element inside the pipeline", true))
                   .get<std::string>();

  if (GST_IS_BIN(pipeline_)) {
    src_ = gst_bin_get_by_name(GST_BIN_CAST(pipeline_), appsrc_name_.c_str());
    if (src_) {
      RCLCPP_INFO(
        node_if->logging->get_logger(), "plugin gst_pipes_appsrc '%s' found '%s'", name_.c_str(),
        appsrc_name_.c_str());
    }

    else {
      RCLCPP_ERROR(
        node_if->logging->get_logger(),
        "plugin gst_pipes_appsrc '%s' failed to locate a gstreamer element called '%s'",
        name_.c_str(), appsrc_name_.c_str());
    }
  } else {
    RCLCPP_ERROR(
      node_if->logging->get_logger(),
      "plugin gst_pipes_appsrc '%s' received invalid pipeline in initialisation", name_.c_str());
  }
}

void gst_pipes_appsrc::frame_cb(/* the ros image message / audio message */) {}

}  // namespace gst_pipes

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipes::gst_pipes_appsink, gst_pipes::gst_pipes_plugin)
PLUGINLIB_EXPORT_CLASS(gst_pipes::gst_pipes_appsrc, gst_pipes::gst_pipes_plugin)
