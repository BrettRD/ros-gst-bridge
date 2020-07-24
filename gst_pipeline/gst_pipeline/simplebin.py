import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject

import rclpy
from rclpy.node import Node

import std_msgs
import diagnostic_updater
import diagnostic_msgs


# create a gstreamer playbin using gst-launch style syntax,
# and create a diagnostic_updater entry to list it.

class Simplebin:
  def __init__(self, node_, name_,
    param_prefix=None,
    bin_description=''
  ):

    self.node = node_
    self.name = name_

    self.bin_description = bin_description
    if param_prefix != None:
      self.bin_description = self.fetch_params(param_prefix)
    
    self.node.get_logger().debug('simple pipeline ' + self.name + ' with description "' + self.bin_description + '"')

    self.bin = self.make_gst_bin(self.bin_description)
    self.diagnostics = self.make_diagnostic_task()


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