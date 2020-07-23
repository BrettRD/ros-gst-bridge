import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject

import rclpy
from rclpy.node import Node

import std_msgs
import diagnostic_updater
import diagnostic_msgs



class Simplebin:
  def __init__(self, node_, bin_description, name_):
    self.node = node_
    self.name = name_
    self.bin = Gst.parse_bin_from_description(bin_description, False)
    self.bin.name = self.name
    self.diagnostics = diagnostic_updater.FunctionDiagnosticTask(self.name + ' Status', self.diagnostic_task)

  def diagnostic_task(self, stat):
    stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, self.name + ' ok')
    stat.add(self.name + ' State', 'yes?')
    return stat
