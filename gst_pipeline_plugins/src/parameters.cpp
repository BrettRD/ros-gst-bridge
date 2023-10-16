#include <parameters.h>

// state updates:
// gst elements expose props through polling-mode getters
// ros nodes hold a black-board separate from the node internals
// this element needs to:
//   update ros as though it were a cache of gst props
//   update gst when an external update is called from ROS
//   poll relevant elements when changes are expected
// ros callbacks can't distinguih between local and remote updates,
//   so we implement a simple lock per property, to prevent concurrent updates.

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


// XXX DELETE ME  waiting for backport of https://github.com/ros2/rclcpp/pull/2041
class node_interfaces_node_t_getter_shim {
  public:
  node_interfaces_node_t_getter_shim(std::shared_ptr<gst_bridge::node_interface_collection> node_if):node_if_(node_if){}
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
  get_node_topics_interface(){return node_if_->topics;}
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface(){return node_if_->base;}
  std::shared_ptr<gst_bridge::node_interface_collection> node_if_;
};


void parameters::initialise(
  std::string name,  // the config name of the plugin
  std::shared_ptr<gst_bridge::node_interface_collection> node_if,
  GstPipeline * pipeline)
{
  name_ = name;
  node_if_ = node_if;
  pipeline_ = pipeline;

  elem_names_ = node_if_->parameters->declare_parameter(
      name_ + ".element_names", rclcpp::ParameterValue(std::vector<std::string>()),
      descr("the names of elements to expose properties for", true)
    ).get<std::vector<std::string>>();

  polling_interval_ = node_if->parameters->declare_parameter(
      name_ + ".polling_interval", rclcpp::ParameterValue(1.0),
      descr("number of seconds between we polls of the entire property tree", true)
    ).get<double>();


  if (GST_IS_BIN(pipeline_)) {
    // dynamic parameter callbacks
    // this callback allows us to reject invalid params before they take effect
    validate_param_handle_ = node_if_->parameters->add_on_set_parameters_callback(std::bind(
      &parameters::validate_parameters_cb, this, std::placeholders::_1));

    //these callbacks update params with validated values
    param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(node_interfaces_node_t_getter_shim(node_if_));

    // hook to the async "message::property-notify" signal emitted by the bus
    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE (pipeline_));
    g_signal_connect (bus, "message::property-notify", (GCallback) parameters::gst_bus_cb, static_cast<gpointer>(this));
    g_signal_connect (bus, "message::async-done", (GCallback) parameters::gst_bus_cb, static_cast<gpointer>(this));
    g_signal_connect (bus, "message", (GCallback) parameters::gst_bus_cb, static_cast<gpointer>(this));

    gst_object_unref(bus);



    // prefixing under the plugin name means it will cause issues if anyone registers
    //    an element called 'type' or 'element_names'
    std::string prefix = name_;

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


    polling_timer_ = rclcpp::create_timer(
      node_if_->base,
      node_if_->timers,
      node_if_->clock->get_clock(),
      rclcpp::Duration::from_nanoseconds( 1e9 * polling_interval_),
      std::bind(&parameters::polling_timer_cb, this),
      nullptr //optional callback group for threading
    );


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

  GValue it_val = {0,0};
  g_value_unset(&it_val);

  while(GST_ITERATOR_OK == gst_iterator_next(it, &it_val)){
    GstElement *item = GST_ELEMENT_CAST (g_value_get_object (&it_val) );
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



void parameters::iterate_props(GstElement * element, std::string prefix)
{
  guint n_props = 0;
  GParamSpec ** prop_list = // free prop_list after use
    g_object_class_list_properties((GObjectClass *)G_OBJECT_GET_CLASS(element), &n_props);

  for (guint i = 0; i < n_props; i++) {
    GParamSpec* prop = prop_list[i];  // don't free prop

    std::string ros_param_name = prefix + '.' + g_param_spec_get_name(prop);

    declare_property(element, prop, ros_param_name);

  }
}


void parameters::declare_property(GstElement * element, GParamSpec* prop, std::string ros_param_name)
{
  // save the association between the full param name and the element/prop pair

  //RCLCPP_INFO_STREAM(node_if_->logging->get_logger(), "prop: " << ros_param_name << " type: " << prop->value_type);


  // test if the parameter was already declared
  if( ! node_if_->parameters->has_parameter(ros_param_name))
  {
    // get the current value
    GValue prop_value = {0,0};
    g_value_init(&prop_value, prop->value_type);
    g_object_get_property(G_OBJECT(element), prop->name, &prop_value);
    GST_OBJECT_LOCK(element);
    rclcpp::ParameterValue ros_value = g_value_to_ros_value(&prop_value);

    // test the parameter has a sensible type
    if(ros_value.get_type() != PARAMETER_NOT_SET)
    {
      // keep track of this pairing
      parameter_mapping map = {element, prop, UPDATE_SOURCE_GST};
      param_map_.insert({ros_param_name, map});

      // declare the parameter with ROS
      node_if_->parameters->declare_parameter(
        ros_param_name,
        ros_value,
        descr(
          g_param_spec_get_blurb(prop),
          false,
          (0 != (G_PARAM_WRITABLE & prop->flags)) ? "Read Write" : "Read Only", // rclcpp readonly means const
          true  // dynamic typing
        )
      );

      // send a bus message if any of the property changes value
      //  we expect to receive a GstMessage of type GST_MESSAGE_PROPERTY_NOTIFY
      gst_element_add_property_notify_watch(
        element,
        g_param_spec_get_name(prop),
        true
      );


      // register a callback the parameter updates from ROS
      param_handles_.push_back(
        param_handler_->add_parameter_callback(
          ros_param_name,
          std::bind(
            &parameters::update_parameters_cb,
            this,
            std::placeholders::_1
          )
        )
      );

    }
    GST_OBJECT_UNLOCK(element);

  }
}




rcl_interfaces::msg::SetParametersResult
parameters::validate_parameters_cb(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const rclcpp::Parameter& parameter : parameters) {
    // find the property matching the parameter
    GParamSpec* prop = NULL;
    //GstElement* element = NULL;
    update_source_t* source;


    RCLCPP_DEBUG(node_if_->logging->get_logger(), "validate %s -> %s",
      parameter.get_name().c_str(),
      parameter.value_to_string().c_str()
    );

    try {
      parameter_mapping &map = param_map_.at(parameter.get_name());
      prop = map.prop;
      //element = map.element;
      source = &map.source;
    }
    catch (const std::out_of_range& oor) {
      // not found, not our problem
    }

    if(NULL != prop)
    {
      GValue new_value = {0,0};
      // check the type and recover a corresponding gvalue
      if(! ros_value_to_g_value(parameter, g_value_init (&new_value, G_PARAM_SPEC_VALUE_TYPE(prop)))){
        result.successful = false;
        result.reason = "wrong type";
        // XXX  ... " try a my_ros_type_string(prop->value_type)"
        break;
      }

      if( 0 != (G_PARAM_WRITABLE & prop->flags)){  // writable?
      // validate the g_value
        bool changed = g_param_value_validate(prop, &new_value);
        if(changed){
          RCLCPP_WARN(node_if_->logging->get_logger(), "value clamped");
          result.successful = false;
          result.reason = "the GObject rejected the value and suggested '" +
            rclcpp::to_string(g_value_to_ros_value(&new_value)) + "'";
          break;
        }
      }
      else
      {
        // Read-only on the gstreamer side, expect no change
        if (UPDATE_SOURCE_GST == *source)
        {
          RCLCPP_DEBUG(node_if_->logging->get_logger(),
            "validate internal update to read-only property %s",
            parameter.get_name().c_str()
          );
          break;
        }
        else
        {
          result.successful = false;
          result.reason = "This GStreamer property is read-only";
          break;
        }
      }
    }
  }

  return result;
}


// rclcpp fires this callback after a parameter has been validated
void parameters::update_parameters_cb(const rclcpp::Parameter &parameter)
{
  RCLCPP_DEBUG_STREAM(node_if_->logging->get_logger(), "updating: " << parameter);

  // find the property matching the parameter
  GstElement* element = NULL;
  GParamSpec* prop = NULL;
  update_source_t* source;
  try {
    parameter_mapping &map = param_map_.at(parameter.get_name());
    element = map.element;
    prop = map.prop;
    source = &map.source;
  }
  catch (const std::out_of_range& oor) {
    // not found, not our problem
  }
  if(NULL != prop)
  {
    GValue new_value = {0,0};
    GValue old_value = {0,0};
    ros_value_to_g_value(parameter, g_value_init (&new_value, prop->value_type));
    g_object_get_property(G_OBJECT(element), prop->name, &old_value);

    switch(*source){
      case UPDATE_SOURCE_NONE:   // update starting
        if( 0 != (G_PARAM_WRITABLE & prop->flags))
        {
          bool changed = g_param_value_validate(prop, &new_value);
          if(!changed)
          {
            if(parameter.get_parameter_value() != g_value_to_ros_value(&old_value))
            {
              RCLCPP_DEBUG(node_if_->logging->get_logger(),
                "ros updating property %s -> %s",
                parameter.get_name().c_str(),
                parameter.value_to_string().c_str()
              );
              *source = UPDATE_SOURCE_ROS;
              g_object_set_property(G_OBJECT(element), prop->name, &new_value);
            }
            else
            {
              RCLCPP_DEBUG(node_if_->logging->get_logger(),
                "skipping unchanged property %s",
                parameter.get_name().c_str()
              );
            }
          }
          else
          {
            RCLCPP_ERROR(node_if_->logging->get_logger(),
              "update on an un-validated property, %s can't be %s, try %s",
              parameter.get_name().c_str(),
              parameter.value_to_string().c_str(),
              rclcpp::to_string(g_value_to_ros_value(&new_value)).c_str()
            );
          }
        }
        else
        {
          RCLCPP_DEBUG(node_if_->logging->get_logger(),
            "skipping read-only property %s",
            parameter.get_name().c_str()
          );
        }

        break;
      case UPDATE_SOURCE_GST:    // internal update from gstreamer is complete
        *source = UPDATE_SOURCE_NONE; // unlock updates
        RCLCPP_DEBUG(node_if_->logging->get_logger(),
          "ros finished update %s",
          parameter.get_name().c_str()
        );
        property_changed_cb(element, prop->name, NULL);  // check for blocked updates
        break;
      default:
        RCLCPP_DEBUG(node_if_->logging->get_logger(),
          "ros update blocked %s",
          parameter.get_name().c_str()
        );

        break;
    }
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
  std::string prefix = "props";

  if (! GST_IS_BIN(element)) {  // ignore containers

    // check if the elem_names_ list is empty
    if(0 == this_ptr->elem_names_.size())
    {
      GstElement * par = GST_ELEMENT_CAST(sub_bin);
      std::string path = GST_OBJECT_NAME (element);
      // recursively find parent bins until you find self
      while((NULL != par) && (GST_ELEMENT_CAST(self) != par)) {
        // build a ros param prefix
        path = std::string(GST_OBJECT_NAME(par)) + "." + path;
        par = GST_ELEMENT_PARENT(par);
      }
      this_ptr->iterate_props(element, prefix + "." + path);
    }
    else
    {
      // check if this element is listed.
      auto it = std::find_if(this_ptr->elem_names_.begin(), this_ptr->elem_names_.end(),
        [element](std::string elem_name){
          return (0 == g_strcmp0(elem_name.c_str(), GST_OBJECT_NAME (element)));}
      );

      if (it != this_ptr->elem_names_.end()){
        this_ptr->iterate_props(element, prefix + "." + GST_OBJECT_NAME (element));
      }
    }
  }
}


// callback when the pipeline removes an element
//  undeclare any previously declared parameters
void parameters::deep_element_removed_cb(
  GstBin * self, GstBin * sub_bin, GstElement * element, gpointer user_data)
{
  (void) self;
  (void) sub_bin;
  auto* this_ptr = static_cast<parameters*>(user_data);

  // find all ROS parameters associated with element
  for (auto it = this_ptr->param_map_.begin(); it != this_ptr->param_map_.end(); it++) {
    if(it->second.element == element) {
      // undeclare every parameter caught by the filter, remove the association from the map
      this_ptr->node_if_->parameters->undeclare_parameter(it->first);
      this_ptr->param_map_.erase(it);
    }
  }
}


void parameters::polling_timer_cb(){
  property_changed_cb(NULL, NULL, NULL);
}

// triggered when an element's properties are changed on the gst side
// can also be triggered from polling event with all NULL args
void parameters::property_changed_cb(
  GstElement * element,
  const gchar * property_name,
  const GValue * property_value
){
  for (auto &it : param_map_)
  {
    if((
        (NULL == property_name) ||
        (0 == g_strcmp0(it.second.prop->name, property_name))
      ) &&
      (
        (NULL == element) ||
        (it.second.element == element)
      ))
    {
      update_source_t *source = &it.second.source;
      std::string ros_param_name = it.first;
      switch(*source){
        case UPDATE_SOURCE_NONE:   // update starting
          if(NULL == property_value)
          {
            GValue value = {0,0};
            g_object_get_property(G_OBJECT(it.second.element), it.second.prop->name, &value);
            update_property(ros_param_name, &value, source);  // allow the update method to lock the param
            g_value_unset(&value);
          }
          else
          {
            update_property(ros_param_name, property_value, source);
          }
          break;
        case UPDATE_SOURCE_ROS:    // update from ROS is complete
          *source = UPDATE_SOURCE_NONE; // unlock updates
          RCLCPP_DEBUG(node_if_->logging->get_logger(), "gst finished update");
          break;
        default:
          RCLCPP_DEBUG(node_if_->logging->get_logger(), "gst update blocked");
          break;
      }
    }
  }
}

// XXX refactor - there is no safe way to call this outside of property_changed_cb
void parameters::update_property(std::string ros_param_name, const GValue * property_value, update_source_t* source)
{
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
  rclcpp::ParameterValue new_param_val = g_value_to_ros_value(property_value);

  // check the value differs between ROS and GST
  rclcpp::ParameterValue old_param_val = 
    node_if_->parameters->get_parameter(ros_param_name).
      get_parameter_value();

  if(old_param_val == new_param_val){
    return;
  }

  RCLCPP_DEBUG(
    node_if_->logging->get_logger(),
    "property update '%s': old value '%s' differs from new value '%s'",
    ros_param_name.c_str(),
    rclcpp::to_string(old_param_val).c_str(),
    rclcpp::to_string(new_param_val).c_str()
  );
  
  // publish the new value
    RCLCPP_DEBUG(node_if_->logging->get_logger(), "gst starting update");
  *source = UPDATE_SOURCE_GST; // lock updates
  auto res = node_if_->parameters->set_parameters(
    {rclcpp::Parameter(ros_param_name, new_param_val)})[0];


  if(!res.successful){
    *source = UPDATE_SOURCE_NONE; // lock updates

    RCLCPP_ERROR(
      node_if_->logging->get_logger(),
      "property update '%s': to '%s' failed. reason:'%s'",
      ros_param_name.c_str(),
      rclcpp::to_string(new_param_val).c_str(),
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
  const GstStructure* message_struct;

  GstObject * object;
  const gchar* property_name;
  const GValue* property_value;

  message_struct = gst_message_get_structure(message);
  if(message_struct){
    RCLCPP_DEBUG(
      this_ptr->node_if_->logging->get_logger(),
      "got bus msg, %s, originating from %s",
      gst_structure_get_name(message_struct),
      GST_OBJECT_NAME (message->src)
    );
  }
  switch (GST_MESSAGE_TYPE (message)) {

    case GST_MESSAGE_PROPERTY_NOTIFY:
      {
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
      break;

    case GST_MESSAGE_STREAM_START:
    case GST_MESSAGE_STREAM_STATUS:
    case GST_MESSAGE_STATE_CHANGED:
    case GST_MESSAGE_ASYNC_DONE:
    case GST_MESSAGE_NEW_CLOCK:
      if(GST_ELEMENT_CAST(object) == GST_ELEMENT_CAST(this_ptr->pipeline_)){
        // poll all of the elements when the pipeline changes state
        this_ptr->property_changed_cb(NULL, NULL, NULL);
      }
      else
      {
        // handle just the node for limited changes
        this_ptr->property_changed_cb(GST_ELEMENT_CAST(object), NULL, NULL);
      }
      break;
    default:
      break;
  }



  return true;
}



}  // namespace gst_pipeline_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipeline_plugins::parameters, gst_pipeline::plugin_base)
