#ifndef GST_PIPELINE__PLUGIN_BASE_H_
#define GST_PIPELINE__PLUGIN_BASE_H_

#include <gst_bridge/gst_bridge.h>
#include <gst_pipeline/syntax_sugar.h>


#include "rclcpp/rclcpp.hpp"

namespace gst_pipeline
{
class plugin_base
{
public:
  //  pass a ros node interface, the pipeline, and a pointer to an element
  virtual void initialise(
    std::string name,  // the config name of the plugin
    std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline) = 0;

  virtual ~plugin_base() {}

protected:
  plugin_base() {}

  std::string name_;
  std::shared_ptr<gst_bridge::node_interface_collection> node_if_;
  GstElement * pipeline_;
};

}  // namespace gst_pipeline

#endif  //GST_PIPELINE__PLUGIN_BASE_H_
