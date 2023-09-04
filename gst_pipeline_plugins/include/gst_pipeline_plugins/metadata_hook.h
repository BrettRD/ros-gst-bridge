#ifndef GST_PIPELINE_PLUGINS__METADATA_HOOK_H_
#define GST_PIPELINE_PLUGINS__METADATA_HOOK_H_

#include <gst_bridge/gst_bridge.h>
#include <gst_pipeline/plugin_base.h>

#include <gst_msgs/msg/meta_mark.hpp>

#include "rclcpp/rclcpp.hpp"

namespace gst_pipeline_plugins
{
/*
    Inject or retrieve metadata into a buffer at a certain pad

*/
class metadata_hook : public gst_pipeline::plugin_base
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

  // This callback is run inside the pad of the sink element, the return will be ok or drop.
  //  This function needs to be static, we can pass a pointer to our logic in user-data
  static GstPadProbeReturn mark_cb(
    GstPad * pad, GstPadProbeInfo * info, gpointer user_data);

  static GstPadProbeReturn report_cb(
    GstPad * pad, GstPadProbeInfo * info, gpointer user_data);

private:
  // the name of the target element in the pipeline
  std::string elem_name_;

  // a pointer to the target element in the pipeline
  GstElement * bin_;

  // the topic to publish on
  std::string topic_name_;
  std::string frame_id_;

  // whether we mark the buffer or report on its marking
  bool mark_;

  // the publisher for the statistics messages
  rclcpp::Publisher<gst_msgs::msg::MetaMark>::SharedPtr mark_pub_;

};

}  // namespace gst_pipeline_plugins

#endif  //GST_PIPELINE_PLUGINS__METADATA_HOOK_H_
