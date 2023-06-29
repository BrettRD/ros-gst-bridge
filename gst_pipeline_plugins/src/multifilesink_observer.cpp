#include <type_traits>
#include <multifilesink_observer.h>


namespace gst_pipeline_plugins
{
  void multifilesink_observer::initialise(std::string name, std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline) {
    name_ = name;
    node_if_ = node_if;
    pipeline_ = pipeline;
    //loop_ = g_main_loop_new(nullptr, true);

    frame_id_ = node_if_->parameters
      ->declare_parameter(
                          name_ + ".frame_id", rclcpp::ParameterValue(node_if_->base->get_name()),
                          descr("the frame_id denoting the observation", true))
      .get<std::string>();

    elem_name_ = node_if_->parameters
      ->declare_parameter(
                          name_ + ".element_name", rclcpp::ParameterValue("mysink"),
                          descr("the name of the sink element inside the pipeline", true))
      .get<std::string>();

    auto topic_name_ = node_if_->parameters
      ->declare_parameter(
                          name_ + ".event_topic", rclcpp::ParameterValue("/" + elem_name_ + "/gst_multifilesink"),
                          descr("the topic name to post events from the sink", true))
      .get<std::string>();

    rclcpp::QoS qos = rclcpp::SensorDataQoS();

    event_pub_ = rclcpp::create_publisher<gst_msgs::msg::MultifilesinkEvent>(
                                                                             node_if_->parameters, node_if_->topics, topic_name_, qos);

    if (GST_IS_BIN(pipeline_)) {
      GstElement* bin = gst_bin_get_by_name(GST_BIN_CAST(pipeline_), elem_name_.c_str());
      if (bin) {
        RCLCPP_INFO(
                    node_if->logging->get_logger(), "plugin multifilesink_observer '%s' found '%s'",
                    name_.c_str(), elem_name_.c_str());

        // Set messages posting cap manually
        g_object_set(bin, "post-messages", true, nullptr);
        // Find the bus associated with the element
        GstBus* bus = gst_element_get_bus(bin);
        // Attach our callback
        guint event_source_id = gst_bus_add_watch(bus, static_cast<GstBusFunc>(multifilesink_observer::gst_bus_cb), static_cast<gpointer>(this));
        // Safe to unref bus
        gst_object_unref(bus);
        if (event_source_id != 0) {
          RCLCPP_INFO(
                      node_if->logging->get_logger(), "plugin multifilesink_observer '%s' attached watch callback on '%s'",
                      name_.c_str(), elem_name_.c_str());

        } else {
          RCLCPP_ERROR(
                       node_if->logging->get_logger(), "plugin multifilesink_observer '%s' failed to attach watch callback on '%s' as an event source already exists",
                       name_.c_str(), elem_name_.c_str());

        }
      } else {
        RCLCPP_ERROR(
                     node_if->logging->get_logger(),
                     "plugin multifilesink_observer '%s' failed to locate a gstreamer element called '%s'",
                     name_.c_str(), elem_name_.c_str());
      }
    } else {
      RCLCPP_ERROR(
                   node_if->logging->get_logger(),
                   "plugin multifilesink_observer '%s' received invalid pipeline in initialisation",
                   name_.c_str());
    }

    //g_main_loop_run(loop_);
  }

  gboolean multifilesink_observer::gst_bus_cb(GstBus* bus, GstMessage* message, gpointer user_data) {
    (void)bus;
    auto* this_ptr = static_cast<multifilesink_observer*>(user_data);
    const GstStructure* s;

    RCLCPP_INFO(this_ptr -> node_if_ -> logging -> get_logger(), "got bus callback");

    switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_ELEMENT:
      s = gst_message_get_structure(message);
      RCLCPP_INFO(this_ptr -> node_if_ -> logging -> get_logger(), "got bus msg");
      if (strcmp(gst_structure_get_name(s), "GstMultiFileSink")) {
        // Create the ROS Message
        auto msg = gst_msgs::msg::MultifilesinkEvent();
        // Fill the standard header
        rclcpp::Clock::SharedPtr ros_clock = this_ptr->node_if_->clock->get_clock();
        msg.header.stamp = ros_clock->now();
        msg.header.frame_id = this_ptr->frame_id_;
        // Now populate it with the GST Message
        msg.filename = std::string(gst_structure_get_string(s, "filename"));
        gst_structure_get_int(s, "index", &msg.index);
        gst_structure_get_clock_time(s, "timestamp", static_cast<GstClockTime*>(&msg.timestamp));
        gst_structure_get_clock_time(s, "stream-time", static_cast<GstClockTime*>(&msg.stream_time));
        gst_structure_get_clock_time(s, "running-time", static_cast<GstClockTime*>(&msg.running_time));
        gst_structure_get_clock_time(s, "duration", static_cast<GstClockTime*>(&msg.duration));
        gst_structure_get_uint64(s, "offset", &msg.offset);
        gst_structure_get_uint64(s, "offset-end", &msg.offset_end);
        // Publish
        this_ptr -> event_pub_ -> publish(msg);
      }
      break;
    default:
      RCLCPP_INFO(this_ptr -> node_if_ -> logging -> get_logger(), "message unknown");
      /* adjust code here if segfaults */
      break;
    }

    return true;
  }

}  // namespace gst_pipeline_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipeline_plugins::multifilesink_observer, gst_pipeline::plugin_base)
