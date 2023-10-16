#include <net_clock.h>

namespace gst_pipeline_plugins
{


void net_clock::initialise(
  std::string name,  // the config name of the plugin
  std::shared_ptr<gst_bridge::node_interface_collection> node_if,
  GstPipeline * pipeline)
{
  name_ = name;
  node_if_ = node_if;
  pipeline_ = pipeline;

  // XXX at least check the pipeline is not null

  clock_leader_ = node_if->parameters
                 ->declare_parameter(
                   name + ".lead", rclcpp::ParameterValue(false),
                   descr("transmit the time to followers", true))
                 .get<bool>();


  // leaving the leader's lead_address empty should let it bind to all addresses
  lead_address_ = node_if->parameters
                 ->declare_parameter(
                   name + ".address", rclcpp::ParameterValue(""),
                   descr("the network address of the leader", true))
                 .get<std::string>();


  clock_port_ = node_if->parameters
                 ->declare_parameter(
                   name + ".port", rclcpp::ParameterValue(5456),
                   descr("the port to use for synchronisation messaging", true))
                 .get<int>();


  if(clock_leader_){
    create_clock_provider(lead_address_, clock_port_);
    
    // XXX it might make more sense to bind the client to lead_address_
    //     and let the kernel figure out the loopback
    //     or even split the lead into a different plugin
    create_clock_client("127.0.0.1", clock_port_);
  }
  else
  {
    create_clock_client(lead_address_, clock_port_);
  }

}



// create a new clock following the remote provider,
//   and use that clock in the pipeline.
void net_clock::create_clock_client(std::string lead_address, guint16 clock_port)
{
  // it would be nice if the parameter server could change the clock source.
  if(client_clock_) gst_object_unref(client_clock_);
  
  client_clock_ = gst_net_client_clock_new (NULL, lead_address.c_str(), clock_port_, 0);

  gst_pipeline_use_clock (GST_PIPELINE (pipeline_), client_clock_);
}


// most implementaions using the net clock provider seem to
//  use a net clock client as the pipeline clock source.

static GstNetTimeProvider *
net_clock::create_clock_provider(std::string lead_address, guint16 clock_port)
{
  GstClock *clock;

  // XXX this plugin needs to choose a clock for the clock provider
  // Other plugins may choose a different clock, and that will cause confusion.
  clock = gst_system_clock_obtain ();
  //     It might be ok to pull the clock from the
  //     pipeline, then replace it with our client
  //     that would allow earlier plugins to inject 
  //     clocks into this plugin.


  // Construct a network-connected clock provider
  // if lead_address_ is not empty, allow the user to specify the interface

  const gchar * lead_addr = NULL;
  if(lead_address != ""){
    lead_addr = lead_address.c_str();
  }
  prov_clock_ = gst_net_time_provider_new (clock, lead_addr, clock_port);

  gst_object_unref (clock);

  return net_time;

}




}  // namespace gst_pipeline_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipeline_plugins::net_clock, gst_pipeline::plugin_base)
