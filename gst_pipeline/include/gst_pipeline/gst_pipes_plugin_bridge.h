#ifndef GST_PIPELINE__GST_PIPES_PLUGIN_APP_ELEMENTS_H_
#define GST_PIPELINE__GST_PIPES_PLUGIN_APP_ELEMENTS_H_

#include <gst_bridge/gst_bridge.h>
#include <gst_pipes_plugin_base.h>

#include "rclcpp/rclcpp.hpp"

namespace gst_pipes
{
/*

  This plugin allows elements from gst_bridge to register publishers
  and subscribers directly on the node that hosts pipeline.
  Since the pipeline host can be run as a composable node, this is enough
  to offer zero-copy messaging between ROS2 and GStreamer

*/
class gst_pipes_bridge : public gst_pipes_plugin
{
public:
  // during init, we need to
  //  fetch a param from the node for the element name of the bridge element
  //  search the pipeline for the bridge element
  //  load the host's node interfaces into the element
  void initialise(
    std::string name,  // the config name of the plugin
    std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline);

private:
  // the name of the target element in the pipeline
  std::string elem_name_;
  // a pointer to the bridge elements in the pipeline
  GstElement * bin_;
};

}  // namespace gst_pipes

#endif  //GST_PIPELINE__GST_PIPES_PLUGIN_APP_ELEMENTS_H_
