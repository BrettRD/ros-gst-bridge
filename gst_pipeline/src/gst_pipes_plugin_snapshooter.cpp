#include <gst_pipes_plugin_snapshooter.h>

namespace gst_pipes
{
void gst_pipes_snapshooter::initialise(
  std::string name,  // the config name of the plugin
  std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline)
{
  name_ = name;
  node_if_ = node_if;
  pipeline_ = pipeline;
  cb_data.this_ptr = this;
  cb_data.trigger = false;

  elem_name_ = node_if->parameters
                 ->declare_parameter(
                   name + ".element_name", rclcpp::ParameterValue("mysrc"),
                   descr("the name of the source element inside the pipeline", true))
                 .get<std::string>();

  trigger_topic_ = node_if->parameters
                     ->declare_parameter(
                       name + ".trigger_topic", rclcpp::ParameterValue("snap"),
                       descr("the topic name to listen for trigger events", true))
                     .get<std::string>();

  rclcpp::QoS qos = rclcpp::SensorDataQoS();

  trigger_sub_ = rclcpp::create_subscription<std_msgs::msg::Empty>(
    node_if_->parameters, node_if_->topics, trigger_topic_, qos,
    std::bind(&gst_pipes_snapshooter::trigger_sub_cb, this, std::placeholders::_1));

  if (GST_IS_BIN(pipeline_)) {
    bin_ = gst_bin_get_by_name(GST_BIN_CAST(pipeline_), elem_name_.c_str());
    if (bin_) {
      RCLCPP_INFO(
        node_if->logging->get_logger(), "plugin gst_pipes_snapshooter '%s' found '%s'",
        name_.c_str(), elem_name_.c_str());

      // find the src pad of the element
      GstPad * pad = gst_element_get_static_pad(bin_, "src");
      // attach our callback to whenever a buffer crosses the pad
      gst_pad_add_probe(
        pad, GST_PAD_PROBE_TYPE_BUFFER,
        (GstPadProbeCallback)gst_pipes_snapshooter::gst_pad_probe_cb, &cb_data, NULL);

    }

    else {
      RCLCPP_ERROR(
        node_if->logging->get_logger(),
        "plugin gst_pipes_snapshooter '%s' failed to locate a gstreamer element called '%s'",
        name_.c_str(), elem_name_.c_str());
    }
  } else {
    RCLCPP_ERROR(
      node_if->logging->get_logger(),
      "plugin gst_pipes_snapshooter '%s' received invalid pipeline in initialisation",
      name_.c_str());
  }
}

void gst_pipes_snapshooter::trigger_sub_cb(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  cb_data.trigger = true;
}

GstPadProbeReturn gst_pipes_snapshooter::gst_pad_probe_cb(
  GstPad * pad, GstPadProbeInfo * info, gpointer user_data)
{
  GstPadProbeReturn ret;

  (void)pad;
  (void)info;

  struct cb_user_data * cb_data = (struct cb_user_data *)user_data;

  if (cb_data->trigger == true) {
    ret = GST_PAD_PROBE_OK;
  } else {
    ret = GST_PAD_PROBE_DROP;
  }

  cb_data->trigger = false;
  return ret;
}

}  // namespace gst_pipes

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipes::gst_pipes_snapshooter, gst_pipes::gst_pipes_plugin)
