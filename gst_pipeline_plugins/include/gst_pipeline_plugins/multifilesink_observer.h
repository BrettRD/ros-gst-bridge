#ifndef GST_PIPELINE_PLUGINS__MULTIFILESINK_OBSERVER_H_
#define GST_PIPELINE_PLUGINS__MULTIFILESINK_OBSERVER_H_

#include <gst_bridge/gst_bridge.h>
#include <gst_pipeline/plugin_base.h>

#include <gst_msgs/msg/multifilesink_event.hpp>

#include "rclcpp/rclcpp.hpp"

namespace gst_pipeline_plugins
{
/*
  A class publishes the gstreamer multifilesink messages if post-message=true is passed as a to a topic `/{mysink}/gst_multifilesink`.
  This can be used with `ros2 topic hz` to measure the serialisation rate of the stream

    TODO build-in the functionality of ros2 topic hz to reduce chatter on the DDS
*/
class multifilesink_observer : public gst_pipeline::plugin_base
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

  // This callback is attached to the GstBus via:
  // https://gstreamer.freedesktop.org/documentation/gstreamer/gstbus.html?gi-language=c#gst_bus_add_watch
  static gboolean gst_bus_cb(GstBus* bus, GstMessage* message, gpointer user_data);

private:
  // the name of the target element in the pipeline
  std::string elem_name_;
  std::string frame_id_;

  // the publisher for the statistics messages
  rclcpp::Publisher<gst_msgs::msg::MultifilesinkEvent>::SharedPtr event_pub_;

};

}  // namespace gst_pipeline_plugins

#endif  //GST_PIPELINE_PLUGINS__MULTIFILESINK_OBSERVER_H_
