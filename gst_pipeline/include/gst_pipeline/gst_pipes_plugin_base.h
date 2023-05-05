#ifndef GST_PIPELINE__GST_PIPES_PLUGIN_BASE_H_
#define GST_PIPELINE__GST_PIPES_PLUGIN_BASE_H_

#include <gst_bridge/gst_bridge.h>
#include <syntax_sugar.h>

//#include <rclcpp/node_interfaces/node_interfaces.hpp>  //not in galactic
//#include <rclcpp/node_interfaces/node_base_interface.hpp>

#include "rclcpp/rclcpp.hpp"

namespace gst_pipes
{
class gst_pipes_plugin
{
public:
  //  pass a ros node interface, the pipeline, and a pointer to an element
  virtual void initialise(
    std::string name,  // the config name of the plugin
    std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline) = 0;

  virtual ~gst_pipes_plugin() {}

protected:
  gst_pipes_plugin() {}

  std::string name_;
  std::shared_ptr<gst_bridge::node_interface_collection> node_if_;
  GstElement * pipeline_;
};

}  // namespace gst_pipes

#endif  //GST_PIPELINE__GST_PIPES_PLUGIN_BASE_H_
