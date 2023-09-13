#include <parameters.h>




namespace gst_pipeline_plugins
{

// XXX maybe hint g_value_to_ros_value with the desired/expected ros param type
rclcpp::ParameterValue parameters::g_value_to_ros_value(const GValue* value)
{

  rclcpp::ParameterValue param_value = rclcpp::ParameterValue();

  GValue value_trans = {0,0};
  g_value_unset(&value_trans);


  switch(G_VALUE_TYPE(value)){

    case G_TYPE_BOOLEAN:
    {
      param_value = rclcpp::ParameterValue((bool)g_value_get_boolean(value));
    } break;
    case G_TYPE_CHAR:
    case G_TYPE_UCHAR:
    case G_TYPE_INT:
    case G_TYPE_UINT:
    case G_TYPE_LONG:
    case G_TYPE_ULONG:
    case G_TYPE_INT64:
    case G_TYPE_UINT64: // this will roll over but it's too useful to ignore
    {
      g_value_transform(value, g_value_init(&value_trans, G_TYPE_INT64));
      param_value = rclcpp::ParameterValue((int64_t)g_value_get_int64(&value_trans));
    } break;

    case G_TYPE_FLOAT:
    case G_TYPE_DOUBLE:
    {
      g_value_transform(value, g_value_init(&value_trans, G_TYPE_DOUBLE));
      param_value = rclcpp::ParameterValue((double)g_value_get_double(&value_trans));
    } break;

    case G_TYPE_STRING:
    {
      const char* val_str = g_value_get_string(value);
      if(val_str == NULL) val_str = "";
      param_value = rclcpp::ParameterValue(val_str);
    } break;


    default:
    {
      if(G_TYPE_IS_ENUM(G_VALUE_TYPE(value))) {
        g_value_transform(value, g_value_init(&value_trans, G_TYPE_INT64));
        param_value = rclcpp::ParameterValue((int64_t)g_value_get_int64(&value_trans));
        break;
      }

      // GSTreamer introduced an array type where each element is a GValue
      // GST_TYPE_ARRAY can't be resolved at compile time, so we do this.
      if(GST_TYPE_ARRAY == G_VALUE_TYPE(value)) {
        // XXX sanity check that the rest of the array has the same type.
        size_t len = gst_value_array_get_size(value);
        if(len > 0){
          switch (G_VALUE_TYPE (gst_value_array_get_value (value, 0))) {

            case G_TYPE_BOOLEAN:
            {
              std::vector<bool> vec;
              for(size_t i=0; i<len; i++) {
                vec.push_back(
                  g_value_get_boolean(
                    gst_value_array_get_value(value, i)));
              }
              param_value = rclcpp::ParameterValue(vec);
            } break;

            case G_TYPE_CHAR:
            case G_TYPE_UCHAR:
            case G_TYPE_INT:
            case G_TYPE_UINT:
            case G_TYPE_LONG:
            case G_TYPE_ULONG:
            case G_TYPE_INT64:
            case G_TYPE_UINT64: // this will roll over but it's too useful to ignore
            {
              std::vector<int64_t> vec;
              for(size_t i=0; i<len; i++) {
                g_value_transform(
                  gst_value_array_get_value(value, i),
                    g_value_init(&value_trans, G_TYPE_INT64));
                vec.push_back(
                  g_value_get_int64(&value_trans));
              }
              param_value = rclcpp::ParameterValue(vec);
            } break;

            case G_TYPE_FLOAT:
            case G_TYPE_DOUBLE:
            {
              std::vector<double> vec;
              for(size_t i=0; i<len; i++) {
                g_value_transform(
                  gst_value_array_get_value(value, i),
                    g_value_init(&value_trans, G_TYPE_DOUBLE));
                vec.push_back(
                  g_value_get_double(&value_trans));
              }
              param_value = rclcpp::ParameterValue(vec);
            } break;

            case G_TYPE_STRING:
            {
              std::vector<std::string> vec;
              for(size_t i=0; i<len; i++) {
                vec.push_back(
                  g_value_get_string(
                    gst_value_array_get_value(value, i)));
              }
              param_value = rclcpp::ParameterValue(vec);
            } break;

            default:
              break;
          }
        }
      }
    } break;
  }

  return param_value;

}






bool parameters::ros_value_to_g_value(const rclcpp::Parameter& parameter, GValue* value)
{
  GValue v = {0,0};
  switch(parameter.get_type()){
    case PARAMETER_BOOL:
      g_value_set_boolean(g_value_init(&v, G_TYPE_BOOLEAN), parameter.as_bool());
      break;
    case PARAMETER_INTEGER:
      g_value_set_int64(g_value_init(&v, G_TYPE_INT64), parameter.as_int());
      break;
    case PARAMETER_DOUBLE:
      g_value_set_double(g_value_init(&v, G_TYPE_DOUBLE), parameter.as_double());
      break;
    case PARAMETER_STRING:
      g_value_set_string(g_value_init(&v, G_TYPE_STRING), parameter.as_string().c_str());
      break;

    /*
    case PARAMETER_BOOL_ARRAY:
      g_value_init(&v, G_VALUE_ARRAY);
      parameter.as_bool_array();
      break;
    case PARAMETER_BYTE_ARRAY:
      g_value_init(&v, G_VALUE_ARRAY);
      parameter.as_byte_array();
      break;
    case PARAMETER_INTEGER_ARRAY:
      g_value_init(&v, G_VALUE_ARRAY);
      parameter.as_integer_array();
      break;
    case PARAMETER_DOUBLE_ARRAY:
      g_value_init(&v, G_VALUE_ARRAY);
      parameter.as_double_array();
      break;
    case PARAMETER_STRING_ARRAY:
      g_value_init(&v, G_VALUE_ARRAY);
      parameter.as_string_array();
      break;
    */

    default:
      break;
  }

  switch(G_VALUE_TYPE(value)){
    case G_TYPE_BOOLEAN:
    {
      if ((parameter.get_type() == PARAMETER_BOOL) ||
          (parameter.get_type() == PARAMETER_INTEGER)) {
        g_value_transform(&v,value);
      }
    } break;
    case G_TYPE_ENUM:
    case G_TYPE_CHAR:
    case G_TYPE_UCHAR:
    case G_TYPE_INT:
    case G_TYPE_UINT:
    case G_TYPE_LONG:
    case G_TYPE_ULONG:
    case G_TYPE_INT64:
    case G_TYPE_UINT64: // this will roll over but it's too useful to ignore
    {
      if (parameter.get_type() == PARAMETER_INTEGER) {
        g_value_transform(&v,value);
      }
    } break;

    case G_TYPE_FLOAT:
    case G_TYPE_DOUBLE:
    {
      if ((parameter.get_type() == PARAMETER_INTEGER) ||
          (parameter.get_type() == PARAMETER_DOUBLE)) {
        g_value_transform(&v,value);
      }
    } break;

    case G_TYPE_STRING:
    {
      if (parameter.get_type() == PARAMETER_STRING) {
        g_value_transform(&v,value);
      }
    } break;
    
    default:
      if(G_TYPE_IS_ENUM(G_VALUE_TYPE(value))) {
        g_value_transform(&v, value);
        break;
      }
      // XXX Is it possible to reliably infer the cell type of an empty array value?
      // https://docs.gtk.org/gobject/func.param_spec_value_array.html

      // XXX G_TYPE_ARRAY // newer, limited docs and examples
      // XXX G_TYPE_VALUE_ARRAY // deprecated in 2.32, we're running 2.35, still in use in gstreamer
      // XXX GST_TYPE_ARRAY // seems to have transformer utilities available for value_array

      // GSTreamer introduced an array type where each element is a GValue
      // GST_TYPE_ARRAY can't be resolved at compile time, so we use an if statement.

      /*
      if(GST_TYPE_ARRAY == G_VALUE_TYPE(value)) {
        // XXX sanity check that the rest of the array has the same type.
        size_t len = gst_value_array_get_size(value);
        if(len>0){
          switch (G_VALUE_TYPE (gst_value_array_get_value (value, 0))) {
            case G_TYPE_BOOLEAN:
            {


            }
            case G_TYPE_CHAR:
            case G_TYPE_UCHAR:
            case G_TYPE_INT:
            case G_TYPE_UINT:
            case G_TYPE_LONG:
            case G_TYPE_ULONG:
            case G_TYPE_INT64:
            case G_TYPE_UINT64: // this will roll over but it's too useful to ignore
            {

            }

          }

        }
      }
      else
      */
      {
        RCLCPP_INFO(node_if_->logging->get_logger(), "unknown type");
        return false;
      }
      break;
  }

  return true;

}










}  // namespace gst_pipeline_plugins
