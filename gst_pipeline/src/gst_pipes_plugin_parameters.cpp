#include <gst_pipes_plugin_parameters.h>

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

namespace gst_pipes
{
void gst_pipes_parameters::initialise(
  std::string name,  // the config name of the plugin
  std::shared_ptr<gst_bridge::node_interface_collection> node_if, GstElement * pipeline)
{
  name_ = name;
  node_if_ = node_if;
  pipeline_ = pipeline;

  elem_names_param_ = node_if->parameters->declare_parameter(
    name_ + ".element_names", rclcpp::ParameterValue(),
    descr("the name of the source element inside the pipeline", true));

  if (elem_names_param.get_type() == rclcpp::PARAMETER_STRING_ARRAY) {
    elem_names_ = elem_names_param.get<std::vector<std::string>>();
  }

  if (GST_IS_BIN(pipeline_)) {
  } else {
    RCLCPP_ERROR(
      node_if->logging->get_logger(),
      "plugin gst_pipes_snapshooter '%s' received invalid pipeline in initialisation",
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

void iterate_props(GstElement * element, std::string prefix)
{
  guint n_props = 0;
  GParamSpec ** prop_list =
    g_object_class_list_properties((GObjectClass *)G_OBJECT_GET_CLASS(element), &n_props);

  for (guint i = 0; i < n_props; i++) {
    GParamSpec * prop = prop_list[i];
    std::string ros_param_name =
      prefix + '.' + prop.name value =
        item.get_property(prop.name) std::string ros_param_nameros_param_type =
          find_ros_param_type(value)
  }

  def find_ros_param_type(value)
  : if type (value) == bool : param_type = Parameter.Type.BOOL elif type(value) == float
  : param_type = Parameter.Type.DOUBLE elif type(value) == int : param_type =
                   Parameter.Type.INTEGER elif type(value) == str : param_type =
                     Parameter.Type.STRING else : param_type = Parameter.Type.NOT_SET
}

}  // namespace gst_pipes

// callback when the pipeline adds an element
//  conditionally declare parameters
void deep_element_added_cb(
  GstBin * self, GstBin * sub_bin, GstElement * element, gpointer user_data)
{
}

// callback when the pipeline removes an element
//  undeclare any previously declared parameters
void deep_element_removed_cb(
  GstBin * self, GstBin * sub_bin, GstElement * element, gpointer user_data)
{
}

}  // namespace gst_pipes

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gst_pipes::gst_pipes_snapshooter, gst_pipes::gst_pipes_plugin)
