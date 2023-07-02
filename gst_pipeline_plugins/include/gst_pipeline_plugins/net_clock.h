#ifndef GST_PIPELINE_PLUGINS__NET_CLOCK_H_
#define GST_PIPELINE_PLUGINS__NET_CLOCK_H_

#include <gst_pipeline/plugin_base.h>
#include <gst_bridge/gst_bridge.h>
#include "rclcpp/rclcpp.hpp"

namespace gst_pipeline_plugins
{

/*

net_clock:
  insert a network synchronised clock into the pipeline

*/


class net_clock : public gst_pipeline::plugin_base
{
public:

  // during init, we need to
  //  load the port number from the config server
  //  start a gst net clock
  //  load the net clock into the pipeline
  void initialise(
    std::string name,  // the config-file name of the plugin (possibly unused)
    std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline);

private:


  GstNetTimeProvider * prov_clock_;
  std::string lead_address_;
  GstClock *client_clock_;
  guint16 clock_port_;
  GstClockTime base_time_;


};

}  // namespace gst_pipeline_plugins

#endif  //GST_PIPELINE_PLUGINS__NET_CLOCK_H_
