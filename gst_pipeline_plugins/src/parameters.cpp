#include <parameters.h>

// on startup:
//  iterate through bin
//    iterate through sub_bin
//  generate a map of elements, paths

// on pipeline element added:
//  iterate through the top level bin,
//  find the path to the element added,
//  declare parameters

// on pipeline element removed:
//  iterate through the top level bin,
//  find the path to the element removed,
//  undeclare parameters

// on prop param update proposed:
//  try to bounds check the prop
//  return pass/fail

// on prop param update confirm:
//  set the prop

// on tracked_params update:
//  iterate through the list of declared parameters, undeclare parameters where elements are no longer tracked.
//  iterate through the list of tracked elements, declare parameters for them if they now exist

namespace gst_pipeline_plugins
{
void parameters::initialise(
  std::string name,  // the config name of the plugin
  std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline)
{
  name_ = name;
  node_if_ = node_if;
  pipeline_ = pipeline;

  elem_names_param_ = node_if->parameters->declare_parameter(
    name_ + ".element_names",
    rclcpp::ParameterValue(std::vector<std::string>()),
    descr("the name of the source element inside the pipeline", true));

  if (elem_names_param.get_type() == rclcpp::PARAMETER_STRING_ARRAY) {
    elem_names_ = elem_names_param.get<std::vector<std::string>>();
  }

  if (GST_IS_BIN(pipeline_)) {
  }
  else 
  {
    RCLCPP_ERROR(
      node_if->logging->get_logger(),
      "plugin parameters '%s' received invalid pipeline in initialisation",
      name_.c_str());
  }
}

void iterate_elements(GstBin * item, std::string prefix)
{
  GstIterator * it =
    item->iterate_elements() iterator.foreach (self.build_param_table, prefix) if (GST_IS_BIN(item))
  {
    iterate_elements()
  }
  else if (GST_IS_ELEMENT(item))
  {
    // Bin is a sub-class of Element
    iterate_props(item, prefix);
  }
}

rclcpp::ParameterValue parameters::g_value_to_ros_value(GValue g_value)
{
  rclcpp::ParameterType param_type;

  GType g_type = g_value_get_gtype(g_value)

  switch(g_type){
    case G_TYPE_BOOLEAN:
    {
      param_type = rclcpp::ParameterType::PARAMETER_BOOL;
    } break;

    case G_TYPE_CHAR:
    case G_TYPE_UCHAR:
    case G_TYPE_INT:
    case G_TYPE_UINT:
    case G_TYPE_LONG:
    case G_TYPE_ULONG:
    case G_TYPE_INT64:
    case G_TYPE_UINT64:
    {
      param_type = rclcpp::ParameterType::PARAMETER_INTEGER;
    } break;

    case G_TYPE_DOUBLE:
    {
      // G_TYPE_FLOAT
      param_type = rclcpp::ParameterType::PARAMETER_DOUBLE;
    } break;

    case G_TYPE_STRING:
    {
      param_type = rclcpp::ParameterType::PARAMETER_STRING;
    } break;

    case GST_TYPE_ARRAY:  // GSTreamer introduced an array type where each element is a GValue
    {
      const GValue * first = g_value.gst_value_array_get_value();
      GType arr_type = g_value_get_gtype(first);
      // XXX sanity check that the rest of the array has the same type.

      switch(arr_type){
          // param_type =  rclcpp::ParameterType::PARAMETER_BYTE_ARRAY
        case G_TYPE_BOOLEAN:
        {
          param_type = rclcpp::ParameterType::PARAMETER_BOOL_ARRAY
        } break;

        case G_TYPE_CHAR:
        case G_TYPE_UCHAR:
        case G_TYPE_INT:
        case G_TYPE_UINT:
        case G_TYPE_LONG:
        case G_TYPE_ULONG:
        case G_TYPE_INT64:
        case G_TYPE_UINT64:
        {
          param_type = rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY
        } break;

        case G_TYPE_DOUBLE:
        {
          param_type = rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY
        } break;

        case G_TYPE_STRING:
        {
          param_type = rclcpp::ParameterType::PARAMETER_STRING_ARRAY
        } break;

        default:
        {
          param_type = rclcpp::ParameterType::PARAMETER_NOT_SET;
        } break;

      }
    } break;

    default:
    {
      param_type = rclcpp::ParameterType::PARAMETER_NOT_SET;
    } break;
  }


  return param_type != rclcpp::ParameterType::PARAMETER_NOT_SET;
}



void parameters::iterate_props(GstElement * element, std::string prefix)
{
  guint n_props = 0;
  GParamSpec ** prop_list =
    g_object_class_list_properties((GObjectClass *)G_OBJECT_GET_CLASS(element), &n_props);

  for (guint i = 0; i < n_props; i++) {
    GParamSpec* prop = prop_list[i];
    std::string ros_param_name = prefix + '.' + prop->name;
    rclcpp::ParameterValue ros_value = g_value_to_ros_value(item.get_property(prop->name));
    // test if the parameter was already declared
    // declare the parameter
  }


}


// callback when the pipeline adds an element
//  conditionally declare parameters
void parameters::deep_element_added_cb(
  GstBin * self, GstBin * sub_bin, GstElement * element, gpointer user_data)
{
}

// callback when the pipeline removes an element
//  undeclare any previously declared parameters
void parameters::deep_element_removed_cb(
  GstBin * self, GstBin * sub_bin, GstElement * element, gpointer user_data)
{
}

}  // namespace gst_pipeline_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipeline_plugins::parameters, gst_pipeline::gst_pipeline_plugin)
