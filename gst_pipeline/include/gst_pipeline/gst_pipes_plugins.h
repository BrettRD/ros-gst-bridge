#ifndef GST_PIPELINE__GST_PIPES_PLUGINS_H_
#define GST_PIPELINE__GST_PIPES_PLUGINS_H_

#include <gst_bridge/gst_bridge.h>

//#include <rclcpp/node_interfaces/node_interfaces.hpp>  //not in galactic
#include <rclcpp/node_interfaces/node_base_interface.hpp>


#include "rclcpp/rclcpp.hpp"



// parameter descriptions could really do with a constructor
rcl_interfaces::msg::ParameterDescriptor descr(
  const std::string& description,
  const bool& read_only = false,
  const std::string additional_constraints = "",
  const bool& dynamic_typing = false)
{
    rcl_interfaces::msg::ParameterDescriptor descr;
    descr.description = description;
    descr.read_only = read_only;
    descr.additional_constraints = additional_constraints;
    descr.dynamic_typing = dynamic_typing;
    return descr;
}


namespace gst_pipes {



// This interfaces struct is valid for galactic only.
//   humble and rolling have more convenient interface collection types
typedef struct {
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr log;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr param;
} node_interface_collection;



class gst_pipes_plugin
{
  public:
  gst_pipes_plugin();
  //  pass a ros node interface, the pipeline, and a pointer to an element
  void initialise(
    std::string name, // the config name of the plugin
    node_interface_collection node_if,
    GstElement * pipeline);

  protected:
  std::string name_;
  node_interface_collection node_if_;
  GstElement * pipeline_;

};




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

  // a pointer to the appsrc in the pipeline
  GstElement * src;
};





} // namespace gst_pipes 


#endif //GST_PIPELINE__GST_PIPES_PLUGINS_H_
