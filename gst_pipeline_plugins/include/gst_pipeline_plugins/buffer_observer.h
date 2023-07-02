#ifndef GST_PIPELINE_PLUGINS__BUFFER_OBSERVER_H_
#define GST_PIPELINE_PLUGINS__BUFFER_OBSERVER_H_

#include <gst_bridge/gst_bridge.h>
#include <gst_pipeline/plugin_base.h>

#include <gst_msgs/msg/buffer_event.hpp>

#include "rclcpp/rclcpp.hpp"

namespace gst_pipeline_plugins
{
/*
    A class publishes the gstreamer buffer PTS time as a ROSTime to topic `/{mysrc}/gst_pts`.
    This can be used with `ros2 topic hz` to measure the frame rate of video

    TODO build-in the functionality of ros2 topic hz to reduce chatter on the DDS
*/
class buffer_observer : public gst_pipeline::plugin_base
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

  // a pointer to the target element in the pipeline
  GstElement * bin_;

  // the publisher for the statistics messages
  rclcpp::Publisher<gst_msgs::msg::BufferEvent>::SharedPtr event_pub_;

  // the number of events since the last fps timer reset
  // std::atomic_int_least32_t event_count_;


};

}  // namespace gst_pipeline_plugins

#endif  //GST_PIPELINE_PLUGINS__BUFFER_OBSERVER_H_
