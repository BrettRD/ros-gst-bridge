#include <clock_observer.h>

namespace gst_pipeline_plugins
{

using namespace std::chrono_literals;


void clock_observer::initialise(
  std::string name,  // the config name of the plugin
  std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline)
{
  name_ = name;
  node_if_ = node_if;
  pipeline_ = pipeline;


  topic_name_ = node_if->parameters
                ->declare_parameter(
                  name_ + ".topic", rclcpp::ParameterValue("~/" + elem_name_ + "/time"),
                  descr("the topic name to post events from the source", true))
                .get<std::string>();

  frame_id_ = node_if->parameters
                ->declare_parameter(
                  name_ + ".frame_id", rclcpp::ParameterValue("pipeline"),
                  descr("the topic name to post events from the source", true))
                .get<std::string>();

  // XXX convert this to a parameter callback to cast integer parameters from CLI
  observation_interval_ = rclcpp::Duration(
    (rcl_duration_value_t) (1e9  * 
      node_if->parameters->declare_parameter(
        name_ + ".period", rclcpp::ParameterValue(1.0),
        descr("the number of seconds (in ros time) between clock observations", true)
      ).get<double>()
    )
  );


  rclcpp::QoS qos = rclcpp::SensorDataQoS();


  timer_ = rclcpp::create_timer(
    node_if_->clock->get_clock(),
    observation_interval_,
    std::bind(&clock_observer::timer_cb, this)
    //optional callback group for threading
    );

  obs_pub_ = rclcpp::create_publisher<gst_msgs::msg::GstClockObservation>(
              node_if->parameters, node_if->topics, topic_name_, qos);


  if (GST_IS_BIN(pipeline_)) {
    
  } else {
    RCLCPP_ERROR(
      node_if->logging->get_logger(),
      "plugin clock_observer '%s' received invalid pipeline in initialisation",
      name_.c_str());
  }

}

void clock_observer::timer_cb()
{

  gst_msgs::msg::GstClockObservation msg;
  msg.header.frame_id = frame_id_;
  // XXX probably do something to deref these pointers before actually sampling the clocks
  msg.header.stamp = node_if_->clock->get_clock()->now();
  msg.pipeline_time = gst_clock_get_time();
  obs_pub_->publish(msg);

}

}  // namespace gst_pipeline_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipeline_plugins::clock_observer, gst_pipeline::plugin_base)
