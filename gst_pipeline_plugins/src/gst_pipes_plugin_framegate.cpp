#include <gst_pipes_plugin_framegate.h>

namespace gst_pipes
{
void gst_pipes_framegate::initialise(
  std::string name,  // the config name of the plugin
  std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline)
{
  name_ = name;
  node_if_ = node_if;
  pipeline_ = pipeline;
  gate_mode_ = gst_msgs::msg::FrameGate::DROP_ALL;

  elem_name_ = node_if->parameters
                 ->declare_parameter(
                   name + ".element_name", rclcpp::ParameterValue("mysrc"),
                   descr("the name of the source element inside the pipeline", true))
                 .get<std::string>();

  gate_topic_ = node_if->parameters
                     ->declare_parameter(
                       name + ".gate_topic", rclcpp::ParameterValue("snap"),
                       descr("the topic name to listen for gate events", true))
                     .get<std::string>();

  bool startup_passing = node_if->parameters
    ->declare_parameter(
			name + ".pass_on_startup", rclcpp::ParameterValue(false),
			descr("pass frames on startup", true))
    .get<bool>();

  if (startup_passing) {
    gate_mode_ = gst_msgs::msg::FrameGate::PASS_ALL;
  } else {
    //gate_mode_ = gst_msgs::msg::FrameGate::DROP_ALL;
    gate_mode_ = gst_msgs::msg::FrameGate::PASS_ONE;	// dirty hack to get through preroll
  }

  rclcpp::QoS qos = rclcpp::SensorDataQoS();

  gate_sub_ = rclcpp::create_subscription<gst_msgs::msg::FrameGate>(
    node_if_->parameters, node_if_->topics, gate_topic_, qos,
    std::bind(&gst_pipes_framegate::gate_sub_cb, this, std::placeholders::_1));

  if (GST_IS_BIN(pipeline_)) {
    bin_ = gst_bin_get_by_name(GST_BIN_CAST(pipeline_), elem_name_.c_str());
    if (bin_) {
      RCLCPP_INFO(
        node_if->logging->get_logger(), "plugin gst_pipes_framegate '%s' found '%s'",
        name_.c_str(), elem_name_.c_str());

      // find the src pad of the element
      GstPad * pad = gst_element_get_static_pad(bin_, "src");
      // attach our callback to whenever a buffer crosses the pad
      gst_pad_add_probe(
        pad, GST_PAD_PROBE_TYPE_BUFFER,
        (GstPadProbeCallback)gst_pipes_framegate::gst_pad_probe_cb, &gate_mode_, NULL);

    }

    else {
      RCLCPP_ERROR(
        node_if->logging->get_logger(),
        "plugin gst_pipes_framegate '%s' failed to locate a gstreamer element called '%s'",
        name_.c_str(), elem_name_.c_str());
    }
  } else {
    RCLCPP_ERROR(
      node_if->logging->get_logger(),
      "plugin gst_pipes_framegate '%s' received invalid pipeline in initialisation",
      name_.c_str());
  }
}

void gst_pipes_framegate::gate_sub_cb(const gst_msgs::msg::FrameGate::SharedPtr msg)
{
  gate_mode_ = msg->mode;
}

GstPadProbeReturn gst_pipes_framegate::gst_pad_probe_cb(
  GstPad * pad, GstPadProbeInfo * info, gpointer user_data)
{
  (void)pad;
  (void)info;
  auto* mode = static_cast<decltype(gate_mode_)*>(user_data);
  
  switch(*mode) {
  case(gst_msgs::msg::FrameGate::DROP_ALL):
    return GST_PAD_PROBE_DROP;
  case(gst_msgs::msg::FrameGate::PASS_ALL):
    return GST_PAD_PROBE_OK;
  case(gst_msgs::msg::FrameGate::PASS_ONE):
    // drop the next one
    *mode = gst_msgs::msg::FrameGate::DROP_ALL;
    // pass this one
    return GST_PAD_PROBE_OK;
  default:
    return GST_PAD_PROBE_DROP;
  }

  return GST_PAD_PROBE_DROP;
  
}

}  // namespace gst_pipes

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipes::gst_pipes_framegate, gst_pipes::gst_pipes_plugin)
