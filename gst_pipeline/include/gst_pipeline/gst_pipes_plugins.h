#ifndef GST_PIPELINE__GST_PIPES_PLUGINS_H_
#define GST_PIPELINE__GST_PIPES_PLUGINS_H_

#include <gst_pipes_plugin_base.h>

//#include <rclcpp/node_interfaces/node_interfaces.hpp>  //not in galactic
//#include <rclcpp/node_interfaces/node_base_interface.hpp>


#include "rclcpp/rclcpp.hpp"

#include <gst_bridge/gst_bridge.h>


namespace gst_pipes {




// a set of callbacks that connect the appsink to a ROS publisher
class gst_pipes_appsink : public gst_pipes_plugin
{
  public:
  // during init, we need to
  //  fetch a param from the node for the element name of the appsink
  //  search the pipeline for the appsink
  //  bind to the appsink callbacks
  void initialise(
    std::string name, // the config name of the plugin
    node_interface_collection node_if,
    GstElement * pipeline);
  
  // handle the frame emitted from the pipeline
  void frame_cb(/* ideally the gstreamer buffer*/);

  private:

  // the name of the target element in the pipeline
  std::string appsink_name_;
  // a pointer to the appsink in the pipeline
  GstElement * sink;

};




// a set of callbacks that connect the appsrc to a ROS publisher
class gst_pipes_appsrc : public gst_pipes_plugin
{
  public:
  void initialise(
    std::string name, // the config name of the plugin
    node_interface_collection node_if,
    GstElement * pipeline);
  
  // handle the frame from the subscription
  void frame_cb(/* the ros image message / audio message */);

  private:

  // the name of the target element in the pipeline
  std::string appsrc_name_;
  // a pointer to the appsrc in the pipeline
  GstElement * src;
};





} // namespace gst_pipes 


#endif //GST_PIPELINE__GST_PIPES_PLUGINS_H_
