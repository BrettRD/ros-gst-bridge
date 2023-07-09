#ifndef GST_PIPELINE_PLUGINS__PARAMETERS_H_
#define GST_PIPELINE_PLUGINS__PARAMETERS_H_

#include <gst_pipeline/plugin_base.h>

#include <gst_bridge/gst_bridge.h>

//#include <rcl_interfaces/msg/ParameterDescriptor.hpp>


#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gst_pipeline_plugins
{
/*
  This plugin declares ros parameters for props on pipeline elements
*/
class parameters : public gst_pipeline::plugin_base
{
public:

  struct parameter_mapping {
    GObject* element;
    GParamSpec* prop;
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
    std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline);


  void iterate_props(GObject * element, std::string prefix);

  rclcpp::ParameterValue g_value_to_ros_value(GValue* value);

  bool ros_value_to_g_value(const rclcpp::Parameter& parameter, GValue* value);

  rcl_interfaces::msg::SetParametersResult
  validate_parameters(std::vector<rclcpp::Parameter> parameters);

  void update_parameters(const rclcpp::Parameter &parameter);

  // callback when the pipeline adds an element
  //  conditionally declare parameters
  //static void deep_element_added_cb(
  //  GstBin * self, GstBin * sub_bin, GstElement * element, gpointer user_data);

  // callback when the pipeline removes an element
  //  undeclare any previously declared parameters
  //static void deep_element_removed_cb(
  //  GstBin * self, GstBin * sub_bin, GstElement * element, gpointer user_data);

private:
  
  GObject * bin_;
  std::string elem_name_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr validate_param_handle_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
  std::vector< rclcpp::ParameterCallbackHandle::SharedPtr > param_handles_;

  std::unordered_map<std::string, parameter_mapping > param_map_;  // use with std::find_if for reverse name lookup

};

}  // namespace gst_pipeline_plugins

#endif  //GST_PIPELINE_PLUGINS__PARAMETERS_H_
