#ifndef GST_PIPELINE__GST_PIPES_PLUGIN_CLOCK_OBSERVER_H_
#define GST_PIPELINE__GST_PIPES_PLUGIN_CLOCK_OBSERVER_H_

#include <gst_bridge/gst_bridge.h>
#include <gst_pipes_plugin_base.h>

#include <rosgraph_msgs/msg/clock.hpp>

#include "rclcpp/rclcpp.hpp"

namespace gst_pipes
{
/*
    A class publishes the gstreamer buffer PTS time as a ROSTime to topic `/{mysrc}/gst_pts`.
    This can be renamed to `/clock` to provide a synthetic clock for ROS2 that correlates with the rendered video.

    TODO Add a wall timer that periodically publishes system clock of this node, or maybe on a seperate plugin.
*/
class gst_pipes_clock_observer : public gst_pipes_plugin
{
public:
  // during init, we need to
  //  fetch a param from the node for the element name of the target element
  //  search the pipeline for the target element
  //  insert a probe into the pipeline on the sink pad of the target element
  void initialise(
    std::string name,  // the config name of the plugin
    std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline);

  // This callback is run inside the pad of the sink element, the return will be ok or drop.
  //  This function needs to be static, we can pass a pointer to our logic in user-data
  static GstPadProbeReturn gst_pad_probe_cb(
    GstPad * pad, GstPadProbeInfo * info, gpointer user_data);

private:
  // the name of the target element in the pipeline
  std::string elem_name_;

  // a pointer to the bridge elements in the pipeline
  GstElement * bin_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

};

}  // namespace gst_pipes

#endif  //GST_PIPELINE__GST_PIPES_PLUGIN_CLOCK_OBSERVER_H_
