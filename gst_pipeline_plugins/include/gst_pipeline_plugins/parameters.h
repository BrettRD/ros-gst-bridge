#ifndef GST_PIPELINE_PLUGINS__PARAMETERS_H_
#define GST_PIPELINE_PLUGINS__PARAMETERS_H_

#include <gst_pipeline/plugin_base.h>

#include <gst_bridge/gst_bridge.h>

//#include <rcl_interfaces/msg/ParameterDescriptor.hpp>


#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gst_pipeline_plugins
{

using rclcpp::ParameterType::PARAMETER_NOT_SET;
using rclcpp::ParameterType::PARAMETER_BOOL;
using rclcpp::ParameterType::PARAMETER_INTEGER;
using rclcpp::ParameterType::PARAMETER_DOUBLE;
using rclcpp::ParameterType::PARAMETER_STRING;
using rclcpp::ParameterType::PARAMETER_BYTE_ARRAY;
using rclcpp::ParameterType::PARAMETER_BOOL_ARRAY;
using rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
using rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
using rclcpp::ParameterType::PARAMETER_STRING_ARRAY;


/*
  This plugin declares ros parameters for props on pipeline elements
*/
class parameters : public gst_pipeline::plugin_base
{
public:

  enum update_source_t {
    UPDATE_SOURCE_NONE,
    UPDATE_SOURCE_ROS,
    UPDATE_SOURCE_GST
  };

  struct parameter_mapping {
    GstElement* element;
    GParamSpec* prop;
    update_source_t source;
      // prop->name
      // prop->value_type
      // g_param_spec_get_blurb(prop)
    //rcl_interfaces::msg::ParameterDescriptor param;
      // param.name
      // param.type
      // param.description
  };
  // during init, we need to
  //  fetch a param from the node for the list of elements to track
  //  find the elements in the pipeline
  //  declare parameters for each prop on the element
  void initialise(
    std::string name,  // the config name of the plugin
    std::shared_ptr<gst_bridge::node_interface_collection> node_if,
    GstPipeline * pipeline);

  void iterate_elements(GstBin * item, std::string prefix);
  void iterate_props(GstElement * element, std::string prefix);
  void declare_property(GstElement * element, GParamSpec* prop, std::string ros_param_name);

  rclcpp::ParameterValue g_value_to_ros_value(const GValue* value);

  bool ros_value_to_g_value(const rclcpp::Parameter& parameter, GValue* value);

  rcl_interfaces::msg::SetParametersResult
  validate_parameters_cb(std::vector<rclcpp::Parameter> parameters);

  void
  update_parameters_cb(const rclcpp::Parameter &parameter);

  // callback when the pipeline adds an element
  //  conditionally declare parameters
  static void deep_element_added_cb(
    GstBin * self,GstBin * sub_bin, GstElement * element, gpointer user_data);

  // callback when the pipeline removes an element
  //  undeclare any previously declared parameters
  static void deep_element_removed_cb(
    GstBin * self, GstBin * sub_bin, GstElement * element, gpointer user_data);

  // property_changed_cb
  // called after receiving a message containing the new property value
  // uses the output of gst_message_parse_property_notify

  void polling_timer_cb();

  void property_changed_cb(
    GstElement * element,
    const gchar * property_name,
    const GValue * property_value
  );

  void update_property(
    std::string ros_param_name,
    const GValue * property_value,
    update_source_t* source
  );

  static gboolean gst_bus_cb(
    GstBus* bus,
    GstMessage* message,
    gpointer user_data
  );

private:

  std::vector<std::string> elem_names_;
  double polling_interval_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr validate_param_handle_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
  std::vector< rclcpp::ParameterCallbackHandle::SharedPtr > param_handles_;

  std::unordered_map<std::string, parameter_mapping > param_map_;  // use with std::find_if for reverse name lookup
  rclcpp::TimerBase::SharedPtr polling_timer_;

};

}  // namespace gst_pipeline_plugins

#endif  //GST_PIPELINE_PLUGINS__PARAMETERS_H_
