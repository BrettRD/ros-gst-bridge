#ifndef GST_PIPELINE__GST_PIPES_H_
#define GST_PIPELINE__GST_PIPES_H_


#include <plugin_base.h>

#include <pluginlib/class_loader.hpp>

#include "rclcpp/rclcpp.hpp"

namespace gst_pipeline
{
// the node and ROS component for the gstreamer pipeline
class gst_pipeline : public rclcpp::Node
{
public:
  // ### functions ###

  gst_pipeline(const rclcpp::NodeOptions & options);
  std::shared_ptr<plugin_base> load_handler();

  // ### gstreamer components ###
  GstElement * pipeline_;
  // a Glib main loop for async callbacks
  GMainLoop* mainloop_;
  std::thread mainloop_thread_;


  // ### pluginlib  ###

  std::unique_ptr<pluginlib::ClassLoader<plugin_base>> loader_;

  std::unordered_map<std::string, std::shared_ptr<plugin_base>> element_handlers_;

  // ### parameters ###

  // XXX This was previously loaded by a handler
  std::string gst_pipeline_base_descr_;
  // XXX This should be loaded by whatever loads the pipeline descr
  std::vector<std::string> gst_plugin_paths_;

  std::vector<std::string> ros_plugin_names_;
  std::unordered_map<std::string, std::string> ros_plugin_types_;
};

}  // namespace gst_pipeline

#endif  //GST_PIPELINE__GST_PIPES_H_
