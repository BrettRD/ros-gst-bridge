import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject

import roslib
roslib.load_manifest('diagnostic_updater')
import rospy
import diagnostic_updater
import diagnostic_msgs
import std_msgs



class Simplebin:
  def __init__(self, bin_description, _name):
    self.name = _name
    self.bin = Gst.parse_bin_from_description(bin_description, False)
    self.bin.name = self.name
    self.diagnostics = diagnostic_updater.FunctionDiagnosticTask(self.name + ' Status', self.diagnostic_task)

  def diagnostic_task(self, stat):
    stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, self.name + ' ok')
    stat.add(self.name + ' State', 'eh?')
    return stat
