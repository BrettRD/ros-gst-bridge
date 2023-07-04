#ifndef GST_PIPELINE_PLUGINS__PARAMETERS_H_
#define GST_PIPELINE_PLUGINS__PARAMETERS_H_

#include <gst_bridge/gst_bridge.h>

#include "rclcpp/rclcpp.hpp"

namespace gst_pipes
{
/*
  This plugin declares ros parameters for props on pipeline elements
*/
class parameters : public gst_pipeline_plugin
{
public:
  // during init, we need to
  //  fetch a param from the node for the list of elements to track
  //  find the elements in the pipeline
  //  declare parameters for each prop on the element
  void initialise(
    std::string name,  // the config name of the plugin
    std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline);

  // callback when the pipeline adds an element
  //  conditionally declare parameters
  static void deep_element_added_cb(
    GstBin * self, GstBin * sub_bin, GstElement * element, gpointer user_data);

  // callback when the pipeline removes an element
  //  undeclare any previously declared parameters
  static void deep_element_removed_cb(
    GstBin * self, GstBin * sub_bin, GstElement * element, gpointer user_data);

private:
  std::vector<std::string> element_names;
};

}  // namespace gst_pipes

#endif  //GST_PIPELINE_PLUGINS__PARAMETERS_H_
