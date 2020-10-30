import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

import std_msgs
import diagnostic_updater
import diagnostic_msgs


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

  # This function will recurse through the elements
  # under item, and declare all properties it finds
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
    #elif type(item) == Gst.Element:
      self.node.get_logger().debug('building param table element')
      param_spec_list = item.list_properties()
      for prop in param_spec_list:
        # make a name to appear in the ROS param list
        prop_name = prop.name
        ros_param_name = prefix + '.' + prop_name
        #param_type = gtype_to_param_type(prop.value_type)
        #Parameter.Type.NOT_SET

        self.node.get_logger().debug('element has prop ' + prop_name + ' ' + str(prop.value_type))
        
        # store the link from ROS param to element
        value = item.get_property(prop_name)
        ros_param_type = find_ros_param_type(value)

        if Parameter.Type.NOT_SET != ros_param_type:
          self.node.get_logger().debug('adding param ' + ros_param_name + ' ' + str(ros_param_type))
          self.param_table[ros_param_name] = {'prop_name':prop_name, 'element':item, 'default':value}
          self.node.declare_parameter(ros_param_name, value) # XXX declare all at once
          # add a callback to update ROS parameters when element props change
          item.connect('notify::' + prop_name, self.prop_cb, ros_param_name, ros_param_type) # XXX test this, 

      # GLib.free(param_spec_list) # XXX

  def prop_cb(self, element, prop, ros_param_name, ros_param_type):
    prop_name = prop.get_name()
    param_type = gtype_to_param_type(prop.value_type)
    value = element.get_property(prop.name)
    self.node.set_parameters([ros_param_name, param_type, value])

  def param_cb(self, params):
    for param in params:
      if param in self.param_table:
        element = param_table[param.name]['element']
        prop_name = param_table[param.name]['prop_name']
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
          (param_prefix + 'descr', None)
        ]
      )
    bin_description = self.node.get_parameter(param_prefix + 'descr').value
    if bin_description == None:
      self.node.get_logger().error('parameter "' + param_prefix + 'descr' + '" not found')
    return bin_description