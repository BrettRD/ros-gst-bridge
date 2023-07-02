#include <rtp_header_hook.h>

#include <gst/rtp/gstrtpbuffer.h>


#include "meta/gstmark.h"

namespace gst_pipeline_plugins
{
void rtp_header_hook::initialise(
  std::string name,  // the config name of the plugin
  std::shared_ptr<gst_bridge::node_interface_collection> node_if,
  GstElement * pipeline)
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
                  name_ + ".report_topic", rclcpp::ParameterValue("~/" + name_ + "/rtp_header_ext"),
                  descr("the topic name to post events from the source", true))
                .get<std::string>();

  frame_id_ = node_if_->parameters
    ->declare_parameter(
                        name_ + ".frame_id", rclcpp::ParameterValue(node_if_->base->get_name()),
                        descr("the frame_id denoting the reporting frame", true))
    .get<std::string>();


  if(!mark_){
    rclcpp::QoS qos = rclcpp::SensorDataQoS().reliable();
    mark_pub_ = rclcpp::create_publisher<gst_msgs::msg::MetaMark>(
                node_if->parameters, node_if->topics, topic_name_, qos);
  }


  if (GST_IS_BIN(pipeline_)) {
    bin_ = gst_bin_get_by_name(GST_BIN_CAST(pipeline_), elem_name_.c_str());
    if (bin_) {
      RCLCPP_INFO(
        node_if->logging->get_logger(), "plugin rtp_header_hook '%s' found '%s'",
        name_.c_str(), elem_name_.c_str());

      // find the src pad of the element
      GstPad * pad;

      if(mark_){
        // bind to the source of the rtp payloader to edit the newly generated header
        pad = gst_element_get_static_pad(bin_, "src");
        gst_pad_add_probe(
          pad, 
          (GstPadProbeType)(GST_PAD_PROBE_TYPE_BUFFER | GST_PAD_PROBE_TYPE_BUFFER_LIST),
          (GstPadProbeCallback)rtp_header_hook::mark_cb, static_cast<gpointer>(this), NULL);
      }
      else
      {
        // bind to the sink of the rtp de-payloader to retrieve the header
        pad = gst_element_get_static_pad(bin_, "sink");
        gst_pad_add_probe(
          pad, GST_PAD_PROBE_TYPE_BUFFER,
          (GstPadProbeCallback)rtp_header_hook::report_cb, static_cast<gpointer>(this), NULL);
      }

    }

    else {
      RCLCPP_ERROR(
        node_if->logging->get_logger(),
        "plugin rtp_header_hook '%s' failed to locate a gstreamer element called '%s'",
        name_.c_str(), elem_name_.c_str());
    }
  } else {
    RCLCPP_ERROR(
      node_if->logging->get_logger(),
      "plugin rtp_header_hook '%s' received invalid pipeline in initialisation",
      name_.c_str());
  }
}

GstPadProbeReturn rtp_header_hook::mark_cb(
  GstPad * pad, GstPadProbeInfo * info, gpointer user_data)
{
  GstPadProbeReturn ret;
  (void)pad;
  auto this_ptr = static_cast<rtp_header_hook*>(user_data);




  GstBuffer *buf;

  if(GST_PAD_PROBE_TYPE_BUFFER_LIST & GST_PAD_PROBE_INFO_TYPE(info))
  {
    GstBufferList *buflist;
    buflist = GST_PAD_PROBE_INFO_BUFFER_LIST(info);
    buf = gst_buffer_list_get(buflist, 0);  // the buffer has been packed into multiple 
  }
  else if(GST_PAD_PROBE_TYPE_BUFFER & GST_PAD_PROBE_INFO_TYPE(info))
  {
    buf = GST_PAD_PROBE_INFO_BUFFER(info);
  }

/*
  RCLCPP_INFO(
    this_ptr->node_if_->logging->get_logger(),
    "plugin rtp_header_hook '%s' pad probe type is %d, containing %d buffers",
    this_ptr->name_.c_str(),
    GST_PAD_PROBE_INFO_TYPE(info),
    gst_buffer_list_length(buflist)
  );

  for(int i=0; i<gst_buffer_list_length(buflist); i++){
    buf = gst_buffer_list_get(buflist, i);

    RCLCPP_INFO(
      this_ptr->node_if_->logging->get_logger(),
      "                          buf %d pts is %d",
      i,
      buf->pts
    );
  }

*/



  gboolean ext_added = false;

  GstClockTime timestamp = GST_BUFFER_PTS(buf);
  gpointer payload = &timestamp;
  size_t payload_size = sizeof(timestamp);

  guint8 appbits = 1; // Application specific bits
  guint8 header_ext_id = 1;  //The ID of the header extension

  // https://stackoverflow.com/questions/67603364/rtp-extension-headers-over-udp
  GstRTPBuffer rtp_buf = GST_RTP_BUFFER_INIT;
  gst_rtp_buffer_map(buf, GST_MAP_READWRITE, &rtp_buf);
  //gst_rtp_buffer_set_extension(rtp_buf, true);
  ext_added = gst_rtp_buffer_add_extension_twobytes_header(
    &rtp_buf,
    appbits,
    header_ext_id,
    payload,
    payload_size);
  gst_rtp_buffer_unmap(&rtp_buf);


  if(!ext_added){
    RCLCPP_ERROR(
      this_ptr->node_if_->logging->get_logger(),
      "plugin rtp_header_hook '%s' failed to add a header extension",
      this_ptr->name_.c_str());
  }

  // life is peachy
  ret = GST_PAD_PROBE_OK;
  return ret;
}


GstPadProbeReturn rtp_header_hook::report_cb(
  GstPad * pad, GstPadProbeInfo * info, gpointer user_data)
{
  GstPadProbeReturn ret;
  (void)pad;
  auto this_ptr = static_cast<rtp_header_hook*>(user_data);

  GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);

  GstClockTime timestamp;

  gboolean ext_found = false;
  guint8 appbits; // Application specific bits
  guint8 header_ext_id = 1;  //filter for header extensions matching this id.
  guint nth = 0; //fetch the first matching payload
  gpointer payload;
  guint payload_size = sizeof(timestamp);

  // https://stackoverflow.com/questions/67603364/rtp-extension-headers-over-udp
  GstRTPBuffer rtp_buf = GST_RTP_BUFFER_INIT;
  gst_rtp_buffer_map(buf, GST_MAP_READ, &rtp_buf);
  ext_found = gst_rtp_buffer_get_extension_twobytes_header(
    &rtp_buf,
    &appbits,
    header_ext_id,
    nth,
    &payload,
    &payload_size
  );
  gst_rtp_buffer_unmap(&rtp_buf);

  if(ext_found){

    // unpack the payload according to our ext_id
    memcpy(&timestamp, payload, sizeof(timestamp));

    GstClockTime tx_pts = timestamp;
    GstClockTime rx_pts = GST_BUFFER_PTS(buf);

    // pack a ros message
    auto msg = gst_msgs::msg::MetaMark();

    msg.header.stamp = this_ptr->node_if_->clock->get_clock()->now();
    msg.header.frame_id = this_ptr->frame_id_;

    msg.buffer_pts = rx_pts;
    msg.mark_timestamp = tx_pts;

    this_ptr -> mark_pub_->publish(msg);
  }


  // life is peachy
  ret = GST_PAD_PROBE_OK;
  return ret;
}


}  // namespace gst_pipeline_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipeline_plugins::rtp_header_hook, gst_pipeline::plugin_base)
