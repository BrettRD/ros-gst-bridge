#include <gst_pipeline.h>
#include <gtest/gtest.h>

TEST(Test, gst_pipeline_loader)
{
  std::unique_ptr<pluginlib::ClassLoader<gst_pipeline::plugin_base>> loader;

  std::unordered_map<std::string, std::string> ros_plugin_types = {
    {"bridge", "gst_pipeline_plugins::bridge"},
    {"pause", "gst_pipeline_plugins::pause_srv"},
    {"framegate", "gst_pipeline_plugins::framegate"}};

  std::unordered_map<std::string, std::shared_ptr<gst_pipeline::plugin_base>> element_handlers;

  // instantiate the pluginlib classloader
  loader = std::make_unique<pluginlib::ClassLoader<gst_pipeline::plugin_base>>(
    "gst_pipeline", "gst_pipeline::plugin_base");

  EXPECT_NE(loader, nullptr);

  // load the ros plugins, but don't initialise
  for (auto name_type : ros_plugin_types) {
    // instantiate the pluginlib plugins
    element_handlers[name_type.first] = loader->createSharedInstance(name_type.second);

    EXPECT_NE(element_handlers[name_type.first], nullptr);
  }
}