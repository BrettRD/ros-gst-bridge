#ifndef GST_PIPELINE__GST_PIPES_PLUGIN_BASE_H_
#define GST_PIPELINE__GST_PIPES_PLUGIN_BASE_H_

#include <gst_bridge/gst_bridge.h>
#include <syntax_sugar.h>

//#include <rclcpp/node_interfaces/node_interfaces.hpp>  //not in galactic
//#include <rclcpp/node_interfaces/node_base_interface.hpp>

#include "rclcpp/rclcpp.hpp"

namespace gst_pipes
{

// This interfaces struct is valid for galactic only.
//   humble and rolling have more convenient interface collection types
typedef struct
{
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr log;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr param;
} node_interface_collection;

class gst_pipes_plugin
{
public:
  //  pass a ros node interface, the pipeline, and a pointer to an element
  virtual void initialise(
    std::string name,  // the config name of the plugin
    node_interface_collection node_if, GstElement * pipeline) = 0;

  virtual ~gst_pipes_plugin() {}

protected:
  gst_pipes_plugin() {}

  std::string name_;
  node_interface_collection node_if_;
  GstElement * pipeline_;
};

}  // namespace gst_pipes

#endif  //GST_PIPELINE__GST_PIPES_PLUGIN_BASE_H_
