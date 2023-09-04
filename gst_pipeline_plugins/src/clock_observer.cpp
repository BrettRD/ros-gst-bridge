#include <clock_observer.h>

namespace gst_pipeline_plugins
{

using namespace std::chrono_literals;


void clock_observer::initialise(
  std::string name,  // the config name of the plugin
  std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstPipeline * pipeline)
{
  name_ = name;
  node_if_ = node_if;
  pipeline_ = pipeline;


  topic_name_ = node_if->parameters
                ->declare_parameter(
                  name_ + ".topic", rclcpp::ParameterValue("~/" + name_ + "/clock_observation"),
                  descr("the topic name to post observations", true))
                .get<std::string>();


  frame_id_ = node_if->parameters
                ->declare_parameter(
                  name_ + ".frame_id", rclcpp::ParameterValue(node_if->base->get_name()),
                  descr("the frame_id denoting the clock", true))
                .get<std::string>();

  // XXX convert this to a parameter callback to cast integer parameters from CLI
  observation_interval_ = node_if->parameters->declare_parameter(
      name_ + ".period", rclcpp::ParameterValue(1.0),
      descr("the number of seconds (in ros time) between clock observations", true)
    ).get<double>();


  rclcpp::QoS qos = rclcpp::SensorDataQoS();


  timer_ = rclcpp::create_timer(
    node_if_->base,
    node_if_->timers,
    node_if_->clock->get_clock(),
    rclcpp::Duration::from_nanoseconds( 1e9 * observation_interval_),
    std::bind(&clock_observer::timer_cb, this),
    nullptr //optional callback group for threading
    );

  obs_pub_ = rclcpp::create_publisher<gst_msgs::msg::ClockObservation>(
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

  gst_msgs::msg::ClockObservation msg;
  msg.header.frame_id = frame_id_;
  
  // deref the pointers wrapping each clock
  GstClock * pipe_clock = gst_pipeline_get_pipeline_clock (GST_PIPELINE_CAST(pipeline_));
  rclcpp::Clock::SharedPtr ros_clock = node_if_->clock->get_clock();

  // XXX may need additional magic to reduce jitter between these two calls
  msg.header.stamp = ros_clock->now();
  GstClockTime gst_clock_time = gst_clock_get_time(pipe_clock);

  // deal with the offsets to yield running time
  GstClockTime base_time = gst_element_get_base_time(GST_ELEMENT(pipeline_));
  msg.pipeline_time = gst_clock_time - base_time;

  obs_pub_->publish(msg);

  //cleanup
  gst_object_unref(pipe_clock);
}

}  // namespace gst_pipeline_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipeline_plugins::clock_observer, gst_pipeline::plugin_base)
