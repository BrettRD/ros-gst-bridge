#ifndef GST_PIPELINE__GST_PIPES_H_
#define GST_PIPELINE__GST_PIPES_H_

//#include <gst_bridge/gst_bridge.h>
//#include <pluginlib/class_loader.h>
//#include <gst_pipeline/gst_pipes_plugins.h>


#include "rclcpp/rclcpp.hpp"


namespace gst_pipes {



// the node and ROS component for the gstreamer pipeline
class gst_pipes : public rclcpp::Node
{
  public:
  gst_pipes(const rclcpp::NodeOptions & options);
  void load_handler();
};




} // namespace gst_pipes



#endif //GST_PIPELINE__GST_PIPES_H_
