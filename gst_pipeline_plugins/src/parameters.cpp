#include <parameters.h>
#include <algorithm> // std::find_if

// on startup:
//  iterate through bin
//    iterate through sub_bin
//  generate a map of elements, paths

// on param update proposed:
//  try to bounds check the prop
//  return pass/fail

// on prop param update confirm:
//  set the prop

// on tracked_params update:
//  iterate through the list of declared parameters, undeclare parameters where elements are no longer tracked.
//  iterate through the list of tracked elements, declare parameters for them if they now exist

// on pipeline element added:
//  iterate through the top level bin,
//  find the path to the element added,
//  declare parameters

// on pipeline element removed:
//  iterate through the top level bin,
//  find the path to the element removed,
//  undeclare parameters


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


void parameters::initialise(
  std::string name,  // the config name of the plugin
  std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline)
{
  name_ = name;
  node_if_ = node_if;
  pipeline_ = pipeline;

  elem_names_ = node_if_->parameters
                 ->declare_parameter(
                   name_ + ".element_names", rclcpp::ParameterValue(std::vector<std::string>()),
                   descr("the names of elements to expose properties for", true))
                 .get<std::vector<std::string>>();

  // dynamic parameter callbacks
  // this callback allows us to reject invalid params before they take effect
  validate_param_handle_ = node_if_->parameters->add_on_set_parameters_callback(std::bind(
    &parameters::validate_parameters, this, std::placeholders::_1));

  //these callbacks update params with validated values
  //param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  // XXX this needs a node-like interfaces struct like Fuse has
  //     https://github.com/ros2/rclcpp/pull/2041

  if (GST_IS_BIN(pipeline_)) {
    std::string prefix = "props";

    // if the element list is empty, iterate through all elements
    if(0 == elem_names_.size()){
      RCLCPP_INFO(
        node_if->logging->get_logger(), "plugin parameters mapping all elements"
      );
      iterate_elements(GST_BIN_CAST(pipeline_), prefix);
    }

    // if the element list is not empty, find each element by name
    for(std::string elem_name : elem_names_){

      GstElement * bin_ = gst_bin_get_by_name(
        GST_BIN_CAST(pipeline_),
        elem_name.c_str()
      );

      if (bin_) {
        iterate_props(GST_ELEMENT_CAST(bin_), prefix + "." + elem_name);

      } else {
        RCLCPP_ERROR(
          node_if->logging->get_logger(),
          "plugin parameters '%s' failed to locate a gstreamer element called '%s'",
          name_.c_str(), elem_name.c_str());
      }
    }

  }
  else 
  {
    RCLCPP_ERROR(
      node_if_->logging->get_logger(),
      "plugin parameters '%s' received invalid pipeline in initialisation",
      name_.c_str());
  }

}


// iterate over all elements
void parameters::iterate_elements(GstBin * item, std::string prefix)
{
  GstIterator * it = gst_bin_iterate_elements(item);

  GValue it_val;
  g_value_unset(&it_val);

  while(GST_ITERATOR_OK == gst_iterator_next(it, &it_val)){
    GstElement *item = GST_ELEMENT (g_value_get_object (&it_val) );
    // Bin is a sub-class of Element, handle it first
    if (GST_IS_BIN(item)) {
      // create a new prefix denoting the name of the containing bin
      iterate_elements(GST_BIN_CAST(item), prefix + "." + GST_OBJECT_NAME (item));
    }
    else if (GST_IS_ELEMENT(item)) {
      // create a parameter for the prop under the name of the element that owns it
      iterate_props(item, prefix + "." + GST_OBJECT_NAME (item));
    }
  }
  g_value_unset (&it_val);
  gst_iterator_free (it);
}


rclcpp::ParameterValue parameters::g_value_to_ros_value(const GValue* value)
{

  rclcpp::ParameterValue param_value = rclcpp::ParameterValue();

  // XXX GLib offers a whole lot of duck-typing
  //     https://github.com/GNOME/glib/blob/main/gobject/gvaluetransform.c
  GValue value_trans;
  g_value_unset(&value_trans);

  switch(g_value_get_gtype(value)){

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
      param_value = rclcpp::ParameterValue(g_value_get_string(value));
    } break;


    default:
    {
      // GSTreamer introduced an array type where each element is a GValue
      // GST_TYPE_ARRAY can't be resolved at compile time, so we do this.
      if(GST_TYPE_ARRAY == g_value_get_gtype(value)) {
        // XXX sanity check that the rest of the array has the same type.
        size_t len = gst_value_array_get_size(value);
        if(len > 0){
          switch (g_value_get_gtype (gst_value_array_get_value (value, 0))) {

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




void parameters::iterate_props(GstElement * element, std::string prefix)
{
  guint n_props = 0;
  GParamSpec ** prop_list = // free prop_list after use
    g_object_class_list_properties((GObjectClass *)G_OBJECT_GET_CLASS(element), &n_props);

  for (guint i = 0; i < n_props; i++) {
    GParamSpec* prop = prop_list[i];  // don't free prop

    std::string ros_param_name = prefix + '.' + g_param_spec_get_name(prop);
    std::string ros_description = g_param_spec_get_blurb(prop);

    // XXX save the association between the full param name and the element/prop pair


    RCLCPP_INFO_STREAM(node_if_->logging->get_logger(), "prop: " << ros_param_name << " type: " << prop->value_type);

    GValue prop_value = {};
    g_value_init(&prop_value, prop->value_type);
    g_value_copy(g_param_spec_get_default_value(prop), &prop_value);
    g_object_get_property(G_OBJECT(element), prop->name, &prop_value);   // get the current value

    RCLCPP_INFO_STREAM(node_if_->logging->get_logger(), "converting");

    rclcpp::ParameterValue ros_value = g_value_to_ros_value(&prop_value);
    // test the parameter has a sensible type

    if(ros_value.get_type() != PARAMETER_NOT_SET)
    {
      // test if the parameter was already declared
      if( ! node_if_->parameters->has_parameter(ros_param_name))
      {
        // keep track of this pairing
        parameter_mapping map = {element, prop};
        param_map_.insert({ros_param_name, map});

        // declare the parameter with ROS
        node_if_->parameters->declare_parameter(
          ros_param_name,
          ros_value,
          descr(ros_description, false)
        );


        // hook to the async "message::property-notify" signal emitted by the bus
        GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE (pipeline_));
        g_signal_connect (bus, "message::property-notify", (GCallback) parameters::gst_bus_cb, static_cast<gpointer>(this));
        gst_object_unref(bus);


        // send a bus message if any of these properties change value
        //  we expect to receive a GstMessage of type GST_MESSAGE_PROPERTY_NOTIFY
        gst_element_add_property_notify_watch(
          element,
          g_param_spec_get_name(prop),
          true
        );



        // register a callback the parameter updates from ROS
        //param_handles_.push_back(
        //  param_handler_->add_parameter_callback(
        //    ros_param_name,
        //    std::bind(
        //      &parameters::update_parameters,
        //      this,
        //      std::placeholders::_1
        //    )
        //  )
        //);

      }
    }
  }
}


bool parameters::ros_value_to_g_value(const rclcpp::Parameter& parameter, GValue* value)
{
  GValue v;
  switch(parameter.get_type()){
    case PARAMETER_BOOL:
      g_value_set_bool(g_value_init(&v, G_TYPE_BOOL), parameter.as_bool());
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


    case PARAMETER_BOOL_ARRAY:
      g_value_init(&v, G_TYPE_STRING)
      parameter.as_bool_array()
      break;
    case PARAMETER_BYTE_ARRAY:
      parameter.as_byte_array();
      break;
    case PARAMETER_INTEGER_ARRAY:
      parameter.as_integer_array();
      break;
    case PARAMETER_DOUBLE_ARRAY:
      parameter.as_double_array();
      break;
    case PARAMETER_STRING_ARRAY:
      parameter.as_string_array();
      break;

    default:
      break;
  }

  switch(g_value_get_gtype(value)){
    case G_TYPE_BOOLEAN:
    {
      if ((parameter.get_type() == PARAMETER_BOOL) ||
          (parameter.get_type() == PARAMETER_INTEGER)) {
        g_value_transform(&v,value);
      }
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

      // XXX Is it possible to reliably infer the cell type of an empty array value?
      // https://docs.gtk.org/gobject/func.param_spec_value_array.html

      // XXX G_TYPE_ARRAY // newer, limited docs and examples
      // XXX G_TYPE_VALUE_ARRAY // deprecated in 2.32, we're running 2.35, still in use in gstreamer
      // XXX GST_TYPE_ARRAY // seems to have transformer utilities available for value_array


      // GSTreamer introduced an array type where each element is a GValue
      // GST_TYPE_ARRAY can't be resolved at compile time, so we do this.
      if(GST_TYPE_ARRAY == g_value_get_gtype(value)) {
        // XXX sanity check that the rest of the array has the same type.
        size_t len = gst_value_array_get_size(value);
        if(len>0){
          switch (g_value_get_gtype (gst_value_array_get_value (value, 0))) {
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
      {
        RCLCPP_INFO(node_if_->logging->get_logger(), "unknown type");
        return false;
      }
      break;
  }

  return true;

}


rcl_interfaces::msg::SetParametersResult
parameters::validate_parameters(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const rclcpp::Parameter& parameter : parameters) {
    // find the property matching the parameter
    GstElement* element = NULL;
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
      GValue value;
      // check the type and recover a corresponding gvalue
      // XXX we might need access to g_param_spec_get_default_value
      //     to resolve the element types of array properties
      if(! ros_value_to_g_value(parameter, g_value_init (&value, G_PARAM_SPEC_VALUE_TYPE(prop)))){
        result.successful = false;
        result.reason = "wrong type";
        // ... " try a my_ros_type_string(prop->value_type)"
        break;
      }


      // validate the g_value
      if( 0 != (G_PARAM_WRITABLE & prop->flags)){
        bool changed = g_param_value_validate(prop, &value);
        if(changed){
          RCLCPP_WARN(node_if_->logging->get_logger(), "value clamped");
          result.successful = false;
          result.reason = "the GObject rejected the value";
          // ... " and suggested my_to_string(&value)."
          break;
        }


        // XXX Mutating the parameter here is strictly incorrect,
        //     but just about eveyone does it.
        //     we should use the update parameters call below instead,
        //     but our node interfaces aren't compatible with the
        //     ParameterEventHandler constructor.
        //     Waiting for the unified interfaces struct in ROS Iron
        // XXX Check that the new value differs
        g_object_set_property(G_OBJECT(element), prop->name, &value);
      }
      else
      {
        RCLCPP_INFO(node_if_->logging->get_logger(), "skipping read-only property");
      }

    }

  }

  return result;
}



void parameters::update_parameters(const rclcpp::Parameter &parameter)
{
  RCLCPP_DEBUG_STREAM(node_if_->logging->get_logger(), "updating: " << parameter);

  // find the property matching the parameter
  GstElement* element = NULL;
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
    ros_value_to_g_value(parameter, g_value_init (&value, prop->value_type));
    g_param_value_validate(prop, &value);
    // XXX Check that the new value differs
    g_object_set_property(G_OBJECT(element), prop->name, &value);

  }
}


// callback when the pipeline adds an element
//  conditionally declare parameters
void parameters::deep_element_added_cb(
  GstBin * self, GstBin * sub_bin, GstElement * element, gpointer user_data)
{
  (void) self;
  (void) sub_bin;
  auto* this_ptr = static_cast<parameters*>(user_data);

  // XXX check if the elem_names_ list is empty
    // recursively find parent bins until you find self
    // build a ros param prefix
    // iterate_props


  // XXX check if this element is listed.
    // build a ros param prefix
    // iterate_props



}

// callback when the pipeline removes an element
//  undeclare any previously declared parameters
void parameters::deep_element_removed_cb(
  GstBin * self, GstBin * sub_bin, GstElement * element, gpointer user_data)
{
  (void) self;
  (void) sub_bin;
  auto* this_ptr = static_cast<parameters*>(user_data);

  // find the ROS parameters associated with element
  auto it = std::find_if(this_ptr->param_map_.begin(), this_ptr->param_map_.end(),
    [element](std::pair<std::string,parameter_mapping> it){
      return (it.second.element == element);}
  );

  for (; it != this_ptr->param_map_.end(); it++) {
    this_ptr->node_if_->parameters->undeclare_parameter(it->first);
    this_ptr->param_map_.erase(it);
  }
}

void parameters::property_changed_cb(
  GstElement * element,
  const gchar * property_name,
  const GValue * property_value
){

  // find the ROS parameter name for the property

  //std::string prefix = GST_OBJECT_NAME (object);
  //std::string ros_param_name = prefix + '.' + property_name;
  std::string ros_param_name;

  // define a match condition lambda for std::find_if
  auto matching = [property_name, element](
    std::pair<std::string,parameter_mapping> it
  ){
    return (0 == g_strcmp0(it.second.prop->name, property_name)) &&
            (it.second.element == element);
  };

  // find the parameter matching the property
  auto it = std::find_if(param_map_.begin(), param_map_.end(), matching);
  if (it == param_map_.end())
  {
    RCLCPP_WARN(
      node_if_->logging->get_logger(),
      "Could not find a ros parameter name for '%s.%s'",
      GST_OBJECT_NAME(element),
      property_name
    );
    return;
  }
  else
  {
    ros_param_name = it->first;
  }


  // check the parameter has been declared (XXX should be redundant)
  if(! node_if_->parameters->has_parameter(ros_param_name)){
    RCLCPP_WARN(
      node_if_->logging->get_logger(),
      "Ignoring change of undeclared property '%s'",
      ros_param_name.c_str()
    );
    return;
  }
  

  // cast the GValue to a ros parameter
  // XXX maybe hint g_value_to_ros_value with the ros param type
  rclcpp::ParameterValue param_val = g_value_to_ros_value(property_value);

  // check the value differs between ROS and GST
  rclcpp::ParameterValue old_param_val = 
    node_if_->parameters->get_parameter(ros_param_name).
      get_parameter_value();

  if(old_param_val == param_val){
      return;
  }
  RCLCPP_INFO(
    node_if_->logging->get_logger(),
    "property update '%s': old value '%s' differs from new value '%s'",
    ros_param_name.c_str(),
    rclcpp::to_string(old_param_val).c_str(),
    rclcpp::to_string(param_val).c_str()
  );

  // publish the new value
  auto res = node_if_->parameters->set_parameters(
    {rclcpp::Parameter(ros_param_name, param_val)})[0];


  if(!res.successful){
    RCLCPP_ERROR(
      node_if_->logging->get_logger(),
      "property update '%s': to '%s' failed. reason:'%s'",
      ros_param_name.c_str(),
      rclcpp::to_string(param_val).c_str(),
      res.reason.c_str()
    );
    
  }

}


// callback when the pipeline changes a property

gboolean parameters::gst_bus_cb(
  GstBus* bus,
  GstMessage* message,
  gpointer user_data
){
  (void)bus;
  auto* this_ptr = static_cast<parameters*>(user_data);
  const GstStructure* s;

  GstObject * object;
  const gchar* property_name;
  const GValue* property_value;

  if(GST_MESSAGE_PROPERTY_NOTIFY == GST_MESSAGE_TYPE(message)) {
    s = gst_message_get_structure(message);
    //if (0 == g_strcmp0(gst_structure_get_name(s), "Property")) {
    //if (0 == g_strcmp0(GST_OBJECT_NAME (message->src), this_ptr->elem_name_.c_str())) {

    RCLCPP_DEBUG(
      this_ptr->node_if_->logging->get_logger(),
      "got bus msg, %s, originating from %s",
      gst_structure_get_name(s),
      GST_OBJECT_NAME (message->src)
    );

    gst_message_parse_property_notify(
      message,
      &object,
      &property_name,
      &property_value
    );

    if (GST_IS_ELEMENT(object)) {
      this_ptr->property_changed_cb(
        GST_ELEMENT_CAST(object),
        property_name,
        property_value
      );
    }
  }


  return true;
}



}  // namespace gst_pipeline_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipeline_plugins::parameters, gst_pipeline::plugin_base)
