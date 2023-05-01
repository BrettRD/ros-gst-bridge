#ifndef GST_PIPELINE__GST_PIPES_H_
#define GST_PIPELINE__GST_PIPES_H_

//#include <gst_bridge/gst_bridge.h>
//#include <pluginlib/class_loader.h>
//#include <gst_pipeline/gst_pipes_plugins.h>

#include <gst_pipes_plugin_base.h>

#include <pluginlib/class_loader.hpp>

#include "rclcpp/rclcpp.hpp"

namespace gst_pipes
{

// the node and ROS component for the gstreamer pipeline
class gst_pipes : public rclcpp::Node
{
public:
  // ### functions ###

  gst_pipes(const rclcpp::NodeOptions & options);
  std::shared_ptr<gst_pipes_plugin> load_handler();

  // ### gstreamer components ###
  GstElement * pipeline_;

  // ### pluginlib  ###

  std::unique_ptr<pluginlib::ClassLoader<gst_pipes_plugin>> loader_;

  std::unordered_map<std::string, std::shared_ptr<gst_pipes_plugin>> element_handlers_;

  // ### parameters ###

  // XXX This was previously loaded by a handler
  std::string gst_pipeline_base_descr_;
  // XXX This should be loaded by whatever loads the pipeline descr
  std::vector<std::string> gst_plugin_paths_;

  std::vector<std::string> ros_plugin_names_;
  std::unordered_map<std::string, std::string> ros_plugin_types_;
};

}  // namespace gst_pipes

#endif  //GST_PIPELINE__GST_PIPES_H_
