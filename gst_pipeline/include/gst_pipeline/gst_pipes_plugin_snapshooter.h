#ifndef GST_PIPELINE__GST_PIPES_PLUGIN_SNAPSHOOTER_H_
#define GST_PIPELINE__GST_PIPES_PLUGIN_SNAPSHOOTER_H_

#include <gst_bridge/gst_bridge.h>
#include <gst_pipes_plugin_base.h>

#include <std_msgs/msg/empty.hpp>

#include "rclcpp/rclcpp.hpp"

namespace gst_pipes
{
/*
  This plugin inserts a callback into the sink pad of the target elenemt,
  conditionally dropping frames according to some traffic on a ros subscription
*/
class gst_pipes_snapshooter : public gst_pipes_plugin
{
public:
  // during init, we need to
  //  fetch a param from the node for the element name of the target element
  //  search the pipeline for the target element
  //  insert a probe into the pipeline on the sink pad of the target element
  void initialise(
    std::string name,  // the config name of the plugin
    std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline);

  void trigger_sub_cb(const std_msgs::msg::Empty::SharedPtr msg);

  // This callback is run inside the pad of the sink element, the return will be ok or drop.
  //  This function needs to be static, we can pass a pointer to our logic in user-data
  static GstPadProbeReturn gst_pad_probe_cb(
    GstPad * pad, GstPadProbeInfo * info, gpointer user_data);

private:
  // wrap some data for a user_data pointer into the gstreamer callback
  struct cb_user_data
  {
    gst_pipes_snapshooter * this_ptr;
    std::atomic_bool trigger;
  };

  // the name of the target element in the pipeline
  std::string elem_name_;

  std::string trigger_topic_;

  // a pointer to the bridge elements in the pipeline
  GstElement * bin_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;

  // A data type we can pass as pointer into the callback
  struct cb_user_data cb_data;
};

}  // namespace gst_pipes

#endif  //GST_PIPELINE__GST_PIPES_PLUGIN_SNAPSHOOTER_H_
