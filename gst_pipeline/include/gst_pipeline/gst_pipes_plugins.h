#ifndef GST_PIPELINE__GST_PIPES_PLUGINS_H_
#define GST_PIPELINE__GST_PIPES_PLUGINS_H_

#include <gst_bridge/gst_bridge.h>

//#include <rclcpp/node_interfaces/node_interfaces.hpp>  //not in galactic
#include <rclcpp/node_interfaces/node_base_interface.hpp>


#include "rclcpp/rclcpp.hpp"


namespace gst_pipes {




class gst_pipes_plugin
{
  public:
  gst_pipes_plugin();
  //  pass a ros node interface, the pipeline, and a pointer to an element
  void initialise(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_if,
    GstElement * pipeline,
    GstElement * elem);
};




// a set of callbacks that connect the appsink to a ROS publisher
class gst_pipes_appsink : public gst_pipes_plugin
{
  public:
  void initialise(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_if,
    GstElement * pipeline,
    GstElement * elem);
  
  // handle the frame emitted from the pipeline
  void frame_cb(/* ideally the gstreamer buffer*/);

  private:

  // a pointer to the appsink in the pipeline
  GstElement * sink;
};




// a set of callbacks that connect the appsrc to a ROS publisher
class gst_pipes_appsrc : public gst_pipes_plugin
{
  public:
  //void initialise(
  //  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_if,
  //  GstElement * pipeline,
  //  GstElement * elem);
  
  // handle the frame from the subscription
  void frame_cb(/* the ros image message / audio message */);

  private:

  // a pointer to the appsrc in the pipeline
  GstElement * src;
};





} // namespace gst_pipes 


#endif //GST_PIPELINE__GST_PIPES_PLUGINS_H_
