
#include <gst_pipes.h>
#include <pluginlib/class_loader.hpp>
#include <gst_pipes_plugins.h>

extern "C" {
#include "gst/gst.h"
}

/*
gst_pipes
This node replaces the gst_pipeline python node
Many thanks to Clyde McQueen for his work on gscam2

This node loads via ros_components, and exposes node interfaces to a
  variety of plugins.

plugins are expected to load their config from the node interface,
  search for relevant gst elements from the pipeline,
  and connect parsers to relevant bus messages
*/



namespace gst_pipes {



gst_pipes::gst_pipes(const rclcpp::NodeOptions & options):
  Node("gst_pipes", options)
{

  // get gstreamer ready
  // XXX check gstreamer args for easy functionality
  gst_init(nullptr, nullptr);

  RCLCPP_INFO(
    get_logger(),
    "Gstreamer version: %s",
    gst_version_string()
  );



  // validate and load the gstreamer plugin paths

  declare_parameter("gst_plugin_paths", gst_plugin_paths_,
    descr("paths to look for gstreamer plugins", true));


  auto registry = gst_registry_get();
  if (registry)
  {
    for(auto path : gst_plugin_paths_)
    {
      if (gst_registry_scan_path(registry, path.c_str())) {
        RCLCPP_INFO(
          get_logger(),
          "Scanned '%s' and added plugins to the registry",
          path.c_str());
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Scanned '%s' for plugins, but the registry did not change",
          path.c_str());
      }
    }
  }
  else
  {
    RCLCPP_ERROR(
      get_logger(),
      "Could not get gstreamer plugin registry, ignoring gst_plugin_paths"
    );
  }



  // read the pipeline parameter
  declare_parameter("pipeline_base_descr", gst_pipeline_base_descr_,
    descr("pipeline description to load before plugins", true));






  // read the pluginlib parameters (to handle elements)
  // fetch the names and types of the ros plugins
  declare_parameter("plugin_names", ros_plugin_names_,
    descr("list of names for each instance of a plugin to load", true));

  for(auto name : ros_plugin_names_){
    std::string type;
    declare_parameter(name+"/type", type,
      descr("the type of the \"" + name +
        "\" plugin this must be a pluginlib class name like \"gst_pipes::gst_pipes_appsink\"",
        true));
    ros_plugin_types_[name] = type;
  }


  // instantiate the pluginlib classloader
  pluginlib::ClassLoader<gst_pipes_plugin> loader(
    "gst_pipeline",
    "gst_pipes::gst_pipes_plugin"
  );



  // load the ros plugins, but don't initialise
  for(auto name_type : ros_plugin_types_){
    // instantiate the pluginlib plugins
    element_handlers_[name_type.first] =
      loader.createSharedInstance(name_type.second);
  }



  // instantiate the pipeline
  GError * error = nullptr;
  pipeline_ = gst_parse_launch(gst_pipeline_base_descr_.c_str(), &error);
  if (!pipeline_) {
    RCLCPP_FATAL(get_logger(), error->message);
    //return false;
  }


  // XXX Connect to the pipeline clock using a pipeline context variable


  node_interface_collection node_if = {
    get_node_base_interface(),
    get_node_logging_interface(),
    get_node_parameters_interface()
  };

  // initialise the ros plugins
  for(auto name_handler : element_handlers_){
    name_handler.second->initialise(
      name_handler.first,
      node_if,
      pipeline_);
  }


  // optionally wait for play/pause event (probably a subscription or service call)


}




} // namespace gst_pipes






#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gst_pipes::gst_pipes)
