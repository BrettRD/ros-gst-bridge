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
    // dynamic parameter callbacks
    // this callback allows us to reject invalid params before they take effect
    validate_param_handle_ = add_on_set_parameters_callback(std::bind(
      &aprameters::validate_parameters, this, std::placeholders::_1));

    //these callbacks update params with validated values
    param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);



  }
  else 
  {
    RCLCPP_ERROR(
      node_if->logging->get_logger(),
      "plugin parameters '%s' received invalid pipeline in initialisation",
      name_.c_str());
  }
}


/*
  XXX doing single-elements for now
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
*/

rclcpp::ParameterValue parameters::g_value_to_ros_value(GValue* value)
{
  rclcpp::ParameterValue param_value = rclcpp::ParameterValue();
  GType g_type = g_value_get_gtype(value)

  switch(g_type){

    case G_TYPE_BOOLEAN:
      param_value = rclcpp::ParameterValue(g_value_get_boolean(value));
      break;

    case G_TYPE_CHAR:
      param_value = rclcpp::ParameterValue(g_value_get_char(value));
      break;
    case G_TYPE_UCHAR:
      param_value = rclcpp::ParameterValue(g_value_get_uchar(value));
      break;
    case G_TYPE_INT:
      param_value = rclcpp::ParameterValue(g_value_get_int(value));
      break;
    case G_TYPE_UINT:
      param_value = rclcpp::ParameterValue(g_value_get_uint(value));
      break;
    case G_TYPE_LONG:
      param_value = rclcpp::ParameterValue(g_value_get_ulong(value));
      break;
    case G_TYPE_ULONG:
      param_value = rclcpp::ParameterValue(g_value_get_ulong(value));
      break;
    case G_TYPE_INT64:
      param_value = rclcpp::ParameterValue(g_value_get_int64(value));
      break;

    // XXX there's no safe way of allowing ROS to set or read this value
    //case G_TYPE_UINT64:
    //  param_value = rclcpp::ParameterValue(g_value_get_uint64());

    case G_TYPE_DOUBLE:
      param_value = rclcpp::ParameterValue(g_value_get_float(value));
      break;
    case G_TYPE_FLOAT:
      param_value = rclcpp::ParameterValue(g_value_get_double(value));
      break;

    case G_TYPE_STRING:
      param_value = rclcpp::ParameterValue(g_value_get_string(value));
      break;

    /*
    XXX Time pressure says no
    case GST_TYPE_ARRAY:  // GSTreamer introduced an array type where each element is a GValue
    {
      const GValue * first = gst_value_array_get_value(value);
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

    */
    default:
      param_value = rclcpp::ParameterValue();
      break;
  }

  return param_value;

}




void parameters::iterate_props(GstElement * element, std::string prefix)
{
  guint n_props = 0;
  GParamSpec ** prop_list = // free prop_list after use
    g_object_class_list_properties((GObjectClass *)G_OBJECT_GET_CLASS(element), &n_props);

  for (guint i = 0; i < n_props; i++) {
    GParamSpec* prop = prop_list[i];  // don't free prop

    std::string ros_param_name = prefix + '.' + g_param_spec_get_name(prop);
    std::string ros_description = g_param_spec_get_blurb(prop);

    GValue prop_value;
    g_value_init(&prop_value, prop->value_type);  // initialise the value according to the type
    g_object_get_property(element, prop->name, prop_value);   // get the current value
    

    rclcpp::ParameterValue ros_value = g_value_to_ros_value(prop_value);
    // test the parameter has a sensible type
    if(ros_value.get_type() ! rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      // test if the parameter was already declared
      if( ! node_if_->param->has_parameter(ros_param_name))
      {
        // keep track of this pairing
        parameter_mapping map = {element, prop};
        param_map_.push_back({ros_param_name, map});

        // declare the parameter with ROS
        node_if_->param->declare_parameter(
          ros_param_name,
          ros_value,
          descr("the name of the source element inside the pipeline", true)
        );
        // register a callback the parameter updates from ROS
        param_handles_.push_back(
          param_handler_->add_parameter_callback(
            ros_param_name,
            std::bind(
              &parameters::update_parameters,
              this,
              std::placeholders::_1
            )
          )
        );

      }
    }
  }
}


bool ros_value_to_g_value(const rclcpp::Parameter& parameter, GValue* value)
{
  GType g_type = g_value_get_gtype(value);


  switch(g_type){

    case G_TYPE_BOOLEAN:
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        g_value_set_boolean(value, parameter.as_bool());
        break;
      }
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        g_value_set_boolean(value, 0 != parameter.as_int());
        break;
      }


    case G_TYPE_CHAR:
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        g_value_set_char(value, (gchar)parameter.as_int());
      }
      break;
    case G_TYPE_UCHAR:
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        g_value_set_uchar(value, (guchar)parameter.as_int());
      }
      break;
    case G_TYPE_INT:
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        g_value_set_int(value, (gint)parameter.as_int());
      }
      break;
    case G_TYPE_UINT:
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        g_value_set_uint(value, (guint)parameter.as_int());
      }
      break;
    case G_TYPE_LONG:
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        g_value_set_long(value, (glong)parameter.as_int());
      }
      break;
    case G_TYPE_ULONG:
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        g_value_set_ulong(value, (gulong)parameter.as_int());
      }
      break;
    case G_TYPE_INT64:
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        g_value_set_int64(value, (gint64)parameter.as_int());
      }
      break;

    case G_TYPE_FLOAT:
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        g_value_set_float(value, (gfloat)parameter.as_int());
      }
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        g_value_set_float(value, (gfloat)parameter.as_double());
      }
      break;
    case G_TYPE_DOUBLE:
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        g_value_set_double(value, (gdouble)parameter.as_int());
      }
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        g_value_set_double(value, (gdouble)parameter.as_double());
      }
      break;

    case G_TYPE_STRING:
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        g_value_set_string(value, parameter.as_string().c_str());
      }
      break;
    
    default:
      return false;
  }

  return true;

}


rcl_interfaces::msg::SetParametersResult
  parameters::validate_parameters(
    std::vector<rclcpp::Parameter> parameters
){
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const rclcpp::Parameter& parameter : parameters) {
    RCLCPP_DEBUG_STREAM(get_logger(), "validating: " << parameter);


    // find the property matching the parameter
    GObject* element = NULL;
    GParamSpec* prop = NULL;
    try {
      parameter_mapping map = param_map_.at(parameter.get_name());
      element = map.element;
      prop = map.prop;
    }
    catch (const std::out_of_range& oor) {
      // not found, not our problem
    }

    if(NULL != prop)
    {
      // recover the type of the prop
      GValue value;
      g_value_init (&value, prop->value_type);
      bool valid;
      
      // check the type and recover a corresponding gvalue
      if(! ros_value_to_g_value(parameter, &value)){
        result.successful = false;
        result.reason = "wrong type";
        // ... " try a my_ros_type_string(prop->value_type)"
        break;
      }
      // validate the g_value
      if(param_value_validate(prop, &value)){
        result.successful = false;
        result.reason = "the Gobject rejected the value";
        // ... " and suggested my_to_string(&value)."
        break;
      }

    }

  }

  return result;
}



void parameters::update_parameters(const rclcpp::Parameter &parameter)
{
  RCLCPP_DEBUG_STREAM(get_logger(), "updating: " << parameter);

  // find the property matching the parameter
  GObject* element = NULL;
  GParamSpec* prop = NULL;
  try {
    parameter_mapping map = param_map_.at(parameter.get_name());
    element = map.element;
    prop = map.prop;
  }
  catch (const std::out_of_range& oor) {
    // not found, not our problem
  }

  if(NULL != prop)
  {
    // recover the type of the prop
    GValue value;
    g_value_init (&value, prop->value_type);
    ros_value_to_g_value(parameter, value);
    param_value_validate(prop, &value)
    g_object_set_property(element, prop->name, value);

  }
}
/*
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
*/

}  // namespace gst_pipeline_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipeline_plugins::parameters, gst_pipeline::gst_pipeline_plugin)
