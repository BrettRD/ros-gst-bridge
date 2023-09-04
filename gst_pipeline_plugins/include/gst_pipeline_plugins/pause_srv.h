#ifndef GST_PIPELINE_PLUGINS__PAUSE_SRV_H_
#define GST_PIPELINE_PLUGINS__PAUSE_SRV_H_

#include <gst_pipeline/plugin_base.h>

//#include <rclcpp/node_interfaces/node_interfaces.hpp>  //not in galactic
//#include <rclcpp/node_interfaces/node_base_interface.hpp>

#include <gst_bridge/gst_bridge.h>

#include <gst_msgs/srv/seek.hpp>
#include <std_srvs/srv/empty.hpp>

#include "rclcpp/rclcpp.hpp"

namespace gst_pipeline_plugins
{
/*

Play/Pause:
  provide a ROS service with a Gazebo style play/pause API

*/

// a ros service that can pause and resume the pipeline
class pause_srv : public gst_pipeline::plugin_base
{
public:
  // during init, we need to
  //  create a ros service
  //  point the service callbacks to the pipeline
  void initialise(
    std::string name,  // the config-file name of the plugin (possibly unused)
    std::shared_ptr<gst_bridge::node_interface_collection> node_if,
    GstPipeline * pipeline);

  // handle the play/pause service requests

  void pause_srv_cb(
    std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr);

  void play_srv_cb(
    std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr);

  void seek_srv_cb(
    gst_msgs::srv::Seek::Request::SharedPtr, gst_msgs::srv::Seek::Response::SharedPtr);

private:
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr play_service_;
  rclcpp::Service<gst_msgs::srv::Seek>::SharedPtr seek_service_;
};

}  // namespace gst_pipeline_plugins

#endif  //GST_PIPELINE_PLUGINS__PAUSE_SRV_H_
