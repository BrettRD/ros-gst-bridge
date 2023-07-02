
#include <gst_pipeline.h>

extern "C" {
#include "gst/gst.h"
}

/*
gst_pipeline
This node replaces the gst_pipeline python node
Many thanks to Clyde McQueen for his work on gscam2

This node loads via ros_components, and exposes node interfaces to a
  variety of plugins.

*/

namespace gst_pipeline
{
gst_pipeline::gst_pipeline(const rclcpp::NodeOptions & options) : Node("gst_pipes_node", options)
{
  // get gstreamer ready
  // XXX can gstreamer args add easy functionality?
  gst_init(nullptr, nullptr);

  RCLCPP_INFO(get_logger(), "Gstreamer version: %s", gst_version_string());

  // validate and load the gstreamer plugin paths
  // XXX add a helper version that asks colcon where packages put their libs
  declare_parameter(
    "gst_plugin_paths", gst_plugin_paths_, descr("paths to look for gstreamer plugins", true));
  gst_plugin_paths_ = get_parameter("gst_plugin_paths").get_value<std::vector<std::string> >();

  auto registry = gst_registry_get();
  if (registry) {
    for (auto path : gst_plugin_paths_) {
      if (gst_registry_scan_path(registry, path.c_str())) {
        RCLCPP_INFO(get_logger(), "Scanned '%s' and added plugins to the registry", path.c_str());
      } else {
        RCLCPP_WARN(
          get_logger(), "Scanned '%s' for plugins, but the registry did not change", path.c_str());
      }
    }
  } else {
    RCLCPP_ERROR(
      get_logger(), "Could not get gstreamer plugin registry, ignoring gst_plugin_paths");
  }

  // read the pipeline parameter
  declare_parameter(
    "pipeline_base_descr", gst_pipeline_base_descr_,
    descr("pipeline description to load before plugins", true));
  gst_pipeline_base_descr_ = get_parameter("pipeline_base_descr").get_value<std::string>();

  // read the pluginlib parameters (to handle elements)
  // fetch the names and types of the ros plugins
  declare_parameter(
    "ros_plugin_names", ros_plugin_names_,
    descr("list of names for each instance of a plugin to load", true));
  ros_plugin_names_ = get_parameter("ros_plugin_names").get_value<std::vector<std::string> >();

  for (auto name : ros_plugin_names_) {
    std::string type;
    declare_parameter(
      name + ".type", type,
      descr(
        "the type of the \"" + name +
          "\" plugin this must be a pluginlib class name like \"gst_pipeline_plugins::pause_srv\"",
        true));
    type = get_parameter(name + ".type").get_value<std::string>();

    ros_plugin_types_[name] = type;

    RCLCPP_INFO(get_logger(), "preparing to load a %s for %s", type.c_str(), name.c_str());
  }

  // instantiate the pluginlib classloader
  loader_ = std::make_unique<pluginlib::ClassLoader<plugin_base> >(
    "gst_pipeline", "gst_pipeline::plugin_base");

  // load the ros plugins, but don't initialise
  for (auto name_type : ros_plugin_types_) {
    RCLCPP_INFO(
      get_logger(), "loading a %s for %s", name_type.second.c_str(), name_type.first.c_str());
    // instantiate the pluginlib plugins
    element_handlers_[name_type.first] = loader_->createSharedInstance(name_type.second);
  }

  // instantiate the pipeline
  GError * error = nullptr;
  if ("" == gst_pipeline_base_descr_) {
    RCLCPP_WARN(get_logger(), "instantiating an empty pipeline");
  } else {
    RCLCPP_INFO(
      get_logger(), "instantiating a pipeline using description '%s'",
      gst_pipeline_base_descr_.c_str());

    pipeline_ = gst_parse_launch(gst_pipeline_base_descr_.c_str(), &error);
    if (!pipeline_) {
      RCLCPP_FATAL(get_logger(), error->message);
      //return false;
    }
  }


  // start a thread to execute the glib main loop (required for bus callbacks)
  mainloop_ = g_main_loop_new(nullptr, true);
  mainloop_thread_ = std::thread(
    [=](GMainLoop* l) { g_main_loop_run(l); },  // lambda spin
    mainloop_
  );

  // Find the bus associated with the pipeline
  GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE (pipeline_));
  // attach the default bus watcher that converts messages to signals
  gst_bus_add_signal_watch(bus);
  gst_object_unref(bus);

  // XXX Connect to the pipeline clock
  // XXX Measure the ros clock offset and add a GstContext to the pipeline
  //     Plugins (both here and inside the pipeline) should refer to
  //     the pipeline's GstContext

  std::shared_ptr<gst_bridge::node_interface_collection> node_if =
    gst_bridge::collect_all_node_interfaces(this);

  // initialise the ros plugins
  for (auto name_handler : element_handlers_) {
    name_handler.second->initialise(name_handler.first, node_if, pipeline_);
  }

  // #### start the pipeline ####

  GstStateChangeReturn state_return;
  state_return = gst_element_set_state(pipeline_, GST_STATE_READY);

  if (GST_STATE_CHANGE_FAILURE != state_return) {
    state_return = gst_element_set_state(pipeline_, GST_STATE_PAUSED);
    // XXX Sanity-check that the rosimagesink doesn't just play here
    if (GST_STATE_CHANGE_FAILURE != state_return) {
      // XXX add a play_automatically param defaulting to true
      state_return = gst_element_set_state(pipeline_, GST_STATE_PLAYING);

      if (GST_STATE_CHANGE_FAILURE != state_return) {
        RCLCPP_INFO(get_logger(), "Pipeline running");

      } else {
        RCLCPP_FATAL(get_logger(), "Failed to play stream");
      }
    } else {
      RCLCPP_FATAL(get_logger(), "Failed to pause stream");
    }
  } else {
    RCLCPP_FATAL(get_logger(), "Failed to ready stream");
  }
}

}  // namespace gst_pipeline

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gst_pipeline::gst_pipeline)
