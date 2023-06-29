#include <metadata_hook.h>


#include "meta/gstmark.h"

namespace gst_pipeline_plugins
{
void metadata_hook::initialise(
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

  mark_ = node_if->parameters
                 ->declare_parameter(
                   name_ + ".mark", rclcpp::ParameterValue(true),
                   descr("mark the buffer (true) or report (false)", true))
                 .get<bool>();

  topic_name_ = node_if->parameters
                ->declare_parameter(
                  name_ + ".report_topic", rclcpp::ParameterValue("~/" + name_ + "/metamark"),
                  descr("the topic name to post events from the source", true))
                .get<std::string>();

  rclcpp::QoS qos = rclcpp::SensorDataQoS();

  mark_pub_ = rclcpp::create_publisher<gst_msgs::msg::MetaMark>(
              node_if->parameters, node_if->topics, topic_name_, qos);


  if (GST_IS_BIN(pipeline_)) {
    bin_ = gst_bin_get_by_name(GST_BIN_CAST(pipeline_), elem_name_.c_str());
    if (bin_) {
      RCLCPP_INFO(
        node_if->logging->get_logger(), "plugin metadata_hook '%s' found '%s'",
        name_.c_str(), elem_name_.c_str());

      // find the src pad of the element
      GstPad * pad = gst_element_get_static_pad(bin_, "src");
      // attach our callback to whenever a buffer crosses the pad

      if(mark_){
        gst_pad_add_probe(
          pad, GST_PAD_PROBE_TYPE_BUFFER,
          (GstPadProbeCallback)metadata_hook::mark_cb, static_cast<gpointer>(this), NULL);
      }
      else
      {
        gst_pad_add_probe(
          pad, GST_PAD_PROBE_TYPE_BUFFER,
          (GstPadProbeCallback)metadata_hook::report_cb, static_cast<gpointer>(this), NULL);
      }

    }

    else {
      RCLCPP_ERROR(
        node_if->logging->get_logger(),
        "plugin metadata_hook '%s' failed to locate a gstreamer element called '%s'",
        name_.c_str(), elem_name_.c_str());
    }
  } else {
    RCLCPP_ERROR(
      node_if->logging->get_logger(),
      "plugin metadata_hook '%s' received invalid pipeline in initialisation",
      name_.c_str());
  }
}

GstPadProbeReturn metadata_hook::mark_cb(
  GstPad * pad, GstPadProbeInfo * info, gpointer user_data)
{
  GstPadProbeReturn ret;
  (void)pad;
  auto this_ptr = static_cast<metadata_hook*>(user_data);

  GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);

  GstMetaMarking* meta = GST_META_MARKING_GET(buf);
  if(meta){
    RCLCPP_ERROR(
      this_ptr->node_if_->logging->get_logger(),
      "plugin metadata_hook '%s' this buffer already has metadata",
      this_ptr->name_.c_str());
  }
  else{
    // add a new metadata object to the buffer
    meta = GST_META_MARKING_ADD(buf);

    // store the pts in the buffer so we don't lose it 
    // when we transport it to another pipeline
    meta->timestamp = GST_BUFFER_PTS(buf);
  }

  // life is peachy
  ret = GST_PAD_PROBE_OK;
  return ret;
}


GstPadProbeReturn metadata_hook::report_cb(
  GstPad * pad, GstPadProbeInfo * info, gpointer user_data)
{
  GstPadProbeReturn ret;
  (void)pad;
  auto this_ptr = static_cast<metadata_hook*>(user_data);

  GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);

  GstMetaMarking* meta = GST_META_MARKING_GET(buf);
  if(meta){
    GstClockTime tx_pts = meta->timestamp;
    GstClockTime rx_pts = GST_BUFFER_PTS(buf);

    auto msg = gst_msgs::msg::MetaMark();

    msg.buffer_pts = rx_pts;
    msg.mark_timestamp = tx_pts;

    this_ptr -> mark_pub_->publish(msg);
  }
  else
  {
    RCLCPP_ERROR(
      this_ptr->node_if_->logging->get_logger(),
      "plugin metadata_hook '%s' could not find metadata to report",
      this_ptr->name_.c_str());
  }

  // life is peachy
  ret = GST_PAD_PROBE_OK;
  return ret;
}


}  // namespace gst_pipeline_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipeline_plugins::metadata_hook, gst_pipeline::plugin_base)
