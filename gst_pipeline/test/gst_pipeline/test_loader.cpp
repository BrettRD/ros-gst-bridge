#include <gst_pipes.h>
#include <gtest/gtest.h>

TEST(Test, gst_pipeline_loader)
{
  std::unique_ptr<pluginlib::ClassLoader<gst_pipes::gst_pipes_plugin>> loader;

  std::unordered_map<std::string, std::string> ros_plugin_types = {
    {"appsrc", "gst_pipes::gst_pipes_appsrc"},
    {"appsink", "gst_pipes::gst_pipes_appsink"},
    {"pause", "gst_pipes::gst_pipes_pause_srv"}};

  std::unordered_map<std::string, std::shared_ptr<gst_pipes::gst_pipes_plugin>> element_handlers;

  // instantiate the pluginlib classloader
  loader = std::make_unique<pluginlib::ClassLoader<gst_pipes::gst_pipes_plugin>>(
    "gst_pipeline", "gst_pipes::gst_pipes_plugin");

  EXPECT_NE(loader, nullptr);

  // load the ros plugins, but don't initialise
  for (auto name_type : ros_plugin_types) {
    // instantiate the pluginlib plugins
    element_handlers[name_type.first] = loader->createSharedInstance(name_type.second);

    EXPECT_NE(element_handlers[name_type.first], nullptr);
  }
}