#include <gst_pipes_plugin_buffer_observer.h>

namespace gst_pipes
{
void gst_pipes_buffer_observer::initialise(
  std::string name,  // the config name of the plugin
  std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline)
{
  name_ = name;
  node_if_ = node_if;
  pipeline_ = pipeline;

  elem_name_ = node_if->parameters
                 ->declare_parameter(
                   name_ + ".element_name", rclcpp::ParameterValue("mysrc"),
                   descr("the name of the source element inside the pipeline", true))
                 .get<std::string>();

  auto topic_name_ = node_if->parameters
                ->declare_parameter(
                  name_ + ".event_topic", rclcpp::ParameterValue("/" + elem_name_ + "/gst_pts"),
                  descr("the topic name to post events from the source", true))
                .get<std::string>();

  rclcpp::QoS qos = rclcpp::SensorDataQoS();

  event_pub_ = rclcpp::create_publisher<gst_msgs::msg::BufferEvent>(
              node_if->parameters, node_if->topics, topic_name_, qos);


  if (GST_IS_BIN(pipeline_)) {
    bin_ = gst_bin_get_by_name(GST_BIN_CAST(pipeline_), elem_name_.c_str());
    if (bin_) {
      RCLCPP_INFO(
        node_if->logging->get_logger(), "plugin gst_pipes_buffer_observer '%s' found '%s'",
        name_.c_str(), elem_name_.c_str());

      // find the src pad of the element
      GstPad * pad = gst_element_get_static_pad(bin_, "src");
      // attach our callback to whenever a buffer crosses the pad
      gst_pad_add_probe(
        pad, GST_PAD_PROBE_TYPE_BUFFER,
        (GstPadProbeCallback)gst_pipes_buffer_observer::gst_pad_probe_cb, static_cast<gpointer>(this), NULL);

    }

    else {
      RCLCPP_ERROR(
        node_if->logging->get_logger(),
        "plugin gst_pipes_buffer_observer '%s' failed to locate a gstreamer element called '%s'",
        name_.c_str(), elem_name_.c_str());
    }
  } else {
    RCLCPP_ERROR(
      node_if->logging->get_logger(),
      "plugin gst_pipes_buffer_observer '%s' received invalid pipeline in initialisation",
      name_.c_str());
  }
}

GstPadProbeReturn gst_pipes_buffer_observer::gst_pad_probe_cb(
  GstPad * pad, GstPadProbeInfo * info, gpointer user_data)
{
  GstPadProbeReturn ret;
  (void)pad;
  auto* ptr = static_cast<gst_pipes_buffer_observer*>(user_data);

  GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);

  auto pts = GST_BUFFER_PTS(buf);

  // publish PTS time to /{mysrc}/gst_pts
  auto msg = gst_msgs::msg::BufferEvent();
  msg.buffer_pts = pts;

  ptr -> event_pub_->publish(msg);

  // life is peachy
  ret = GST_PAD_PROBE_OK;

  return ret;
}

}  // namespace gst_pipes

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipes::gst_pipes_buffer_observer, gst_pipes::gst_pipes_plugin)
