import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject

from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType

import diagnostic_updater
import diagnostic_msgs


# XXX this is an awful hack to deal with broken param type mappings in galactic.
# https://github.com/ros2/rclpy/issues/829#issuecomment-937517881
import os
rosdistro=os.environ['ROS_DISTRO']
if rosdistro == 'galactic':
  distro_dynamic_typing={'dynamic_typing':True}
else:
  distro_dynamic_typing={}


# create a gstreamer playbin using gst-launch style syntax,
# and create a diagnostic_updater entry to list it.

def find_ros_param_type(value):
  if type(value) == bool:
    param_type = Parameter.Type.BOOL
  elif type(value) == float:
    param_type = Parameter.Type.DOUBLE
  elif type(value) == int:
    param_type = Parameter.Type.INTEGER
  elif type(value) == str:
    param_type = Parameter.Type.STRING
  else:
    param_type = Parameter.Type.NOT_SET

  #param_type = Parameter.Type.BOOL_ARRAY
  #param_type = Parameter.Type.BYTE_ARRAY
  #param_type = Parameter.Type.DOUBLE_ARRAY
  #param_type = Parameter.Type.INTEGER_ARRAY
  #param_type = Parameter.Type.STRING_ARRAY


  return param_type

class Simplebin:
  def __init__(self, node_, name_,
    param_prefix=None,
    bin_description=''
  ):

    self.node = node_
    self.name = name_
    self.param_table = {}
    self.bin_description = bin_description
    if param_prefix != None:
      self.bin_description = self.fetch_params(param_prefix)
    
    self.node.get_logger().debug('simple pipeline ' + self.name + ' with description "' + self.bin_description + '"')

    self.bin = self.make_gst_bin(self.bin_description)
    self.diagnostics = self.make_diagnostic_task()
    self.build_param_table(self.bin, '')
    #self.node.get_logger().warn('element has prop table ' + str(self.param_table))
    self.node.add_on_set_parameters_callback(self.param_cb)


  def make_gst_bin(self, bin_description):
    gst_bin = Gst.parse_bin_from_description(bin_description, False)
    gst_bin.name = self.name
    return gst_bin


  def make_diagnostic_task(self):
    diagnostic_task = diagnostic_updater.FunctionDiagnosticTask(self.name + ' Status', self.diagnostic_task)
    return diagnostic_task


  def diagnostic_task(self, stat):
    stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, self.name + ' ok')
    stat.add(self.name + ' State', 'yes?')
    return stat

  # This function will recurse through the elements and declare all properties it finds
  # XXX this feature should be moved into class Pipeline(Node) and update when elements are created or destroyed
  def build_param_table(self, item, prefix):
    if prefix != '':
      prefix = prefix + '.' + item.name
    else:
      prefix = item.name
    if type(item) == Gst.Bin:
      iterator = item.iterate_elements()
      iterator.foreach(self.build_param_table, prefix)
      # don't wrap props for a bin
      self.node.get_logger().debug('building param table bin')
    else:
      self.node.get_logger().debug('building param table element')
      param_spec_list = item.list_properties()
      for prop in param_spec_list:
        value = item.get_property(prop.name)
        ros_param_name = prefix + '.' + prop.name
        ros_param_type = find_ros_param_type(value)

        if Parameter.Type.NOT_SET != ros_param_type:
          self.node.get_logger().debug('adding param ' + ros_param_name + ' ' + str(ros_param_type))
          self.param_table[ros_param_name] = {'prop_name':prop.name, 'element':item, 'default':value}
          self.node.declare_parameter(ros_param_name, value) # XXX declare all at once
          # add a callback to update ROS parameters when element props change
          item.connect('notify::' + prop.name, self.prop_cb, ros_param_name, ros_param_type) # XXX test this, 
        else:
          self.node.get_logger().warn('ignoring prop ' + prop.name + ' type is ' + str(prop.value_type))
      # GLib.free(param_spec_list) # XXX docs say the list needs freeing, but this call fails


  def prop_cb(self, element, prop, ros_param_name, ros_param_type):
    value = element.get_property(prop.name)
    if value != self.node.get_parameter(ros_param_name).value:
      self.node.get_logger().debug('element callback ' + ros_param_name + ' changed')
      self.node.set_parameters([Parameter(ros_param_name, ros_param_type, value)])

  def param_cb(self, params):
    for param in params:
      if param.name in self.param_table:
        self.node.get_logger().debug('param callback ' + param.name)
        element = self.param_table[param.name]['element']
        prop_name = self.param_table[param.name]['prop_name']
        if param.value != element.get_property(prop_name):
          element.set_property(prop_name, param.value)
      else:
        self.node.get_logger().warn('simple pipeline ' + self.name + ' param_cb: unrecognised parameter "' + param.name + '"')
    return SetParametersResult(successful=True)



  def fetch_params(self, param_prefix):
    if param_prefix != '':
      param_prefix = param_prefix + '.'

    self.node.declare_parameters(
      namespace='',
      parameters=[
          (param_prefix + 'descr', None, ParameterDescriptor(
              name=param_prefix + 'descr',
              type=ParameterType.PARAMETER_STRING,
              **distro_dynamic_typing
              )
          ),
        ]
      )
    bin_description = self.node.get_parameter(param_prefix + 'descr').value
    if bin_description == None:
      self.node.get_logger().error('parameter "' + param_prefix + 'descr' + '" not found')
    return bin_description
