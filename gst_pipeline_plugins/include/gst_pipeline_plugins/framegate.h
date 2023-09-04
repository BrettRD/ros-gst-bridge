#ifndef GST_PIPELINE_PLUGINS__FRAMEGATE_H_
#define GST_PIPELINE_PLUGINS__FRAMEGATE_H_

#include <atomic>

#include <gst_bridge/gst_bridge.h>
#include <gst_pipeline/plugin_base.h>

#include <gst_msgs/msg/frame_gate.hpp>

#include "rclcpp/rclcpp.hpp"


namespace gst_pipeline_plugins
{
/*
  This plugin inserts a callback into the sink pad of the target element,
  controlling the flow of frames according the last recieved FrameGate message
  mode on a ros subscription.
*/
class framegate : public gst_pipeline::plugin_base
{
public:
  // during init, we need to
  //  fetch a param from the node for the element name of the target element
  //  search the pipeline for the target element
  //  insert a probe into the pipeline on the sink pad of the target element
  void initialise(
    std::string name,  // the config name of the plugin
    std::shared_ptr<gst_bridge::node_interface_collection> node_if,
    GstPipeline * pipeline);

  void gate_sub_cb(const gst_msgs::msg::FrameGate::SharedPtr msg);

  // This callback is run inside the pad of the sink element, the return will be ok or drop.
  //  This function needs to be static, we can pass a pointer to our logic in user-data (TODO validate need)
  static GstPadProbeReturn gst_pad_probe_cb(
    GstPad * pad, GstPadProbeInfo * info, gpointer user_data);

private:
  // state machine
  std::atomic<decltype(gst_msgs::msg::FrameGate().mode)> gate_mode_;

  // the name of the target element in the pipeline
  std::string elem_name_;

  // topic name
  std::string gate_topic_;

  // a pointer to the bridge elements in the pipeline
  GstElement * bin_;

  rclcpp::Subscription<gst_msgs::msg::FrameGate>::SharedPtr gate_sub_;
};

}  // namespace gst_pipeline_plugins

#endif  //GST_PIPELINE_PLUGINS__FRAMEGATE_H_
