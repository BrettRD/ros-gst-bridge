import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject


import rclpy
from rclpy.node import Node

import std_msgs
import diagnostic_updater
import diagnostic_msgs

import asyncio

class Pipeline(Node):
  def __init__(self, node_name='pipeline_node'):
    super().__init__(node_name)
    self.updater = diagnostic_updater.Updater(self)
    self.updater.setHardwareID("Gst1.0")
    self.updater.add("Pipe Status", self.diagnostic_task)
    self.pipe = Gst.Pipeline.new("rospipe")
    self.bus = self.pipe.get_bus()
    self.bus.add_signal_watch()
    self.bus.connect('message::error', self.on_error)
    self.bus.connect('message::state-changed', self.on_status_changed)
    self.bus.connect('message::eos', self.on_eos)
    self.bus.connect('message::info', self.on_info)
    self.get_logger().info('startup')
    self.updater.force_update()


  async def async_task(self):
    while True:
      self.updater.update()
      await asyncio.sleep(0.5)



  def on_status_changed(self, bus, message):
    (oldstate, newstate, pending) = message.parse_state_changed()
    logmsg = str(message.timestamp) + ' status_changed ' + message.src.name
    logmsg += ' from ' + Gst.Element.state_get_name(oldstate)
    logmsg += ' to ' + Gst.Element.state_get_name(newstate)
    if pending != Gst.State.VOID_PENDING:
      logmsg += ' targetting ' + Gst.Element.state_get_name(pending)
    self.get_logger().info(logmsg)


  def on_eos(self, bus, message):
    logmsg = str(message.timestamp) + ' eos message from ' + message.src.name
    self.get_logger().info(logmsg)

  def on_info(self, bus, message):
    logmsg = str(message.timestamp) + ' info message from ' + message.src.name
    logmsg += ' ' + str(message.parse_info_details())
    self.get_logger().info(logmsg)

  def on_error(self, bus, message):
    logmsg = str(message.timestamp) + ' error message from ' + message.src.name
    logmsg += ' ' + str(message.parse_error_details())
    self.get_logger().info(logmsg)

  def start_pipeline(self):
    self.get_logger().info('Current state of my pipeline is '+ Gst.Element.state_get_name(self.pipe.current_state))
    self.get_logger().info('setting pipeline to PLAYING')
    self.pipe.set_state(Gst.State.PLAYING)

  def add_section(self, section):
    self.get_logger().info('adding section ' + section.name)
    if section.bin != None:
      self.pipe.add(section.bin)
      # XXX self.updater only seems to produce messages for the last task add()ed
      #if section.diagnostics != None:
      #  self.updater.add(section.diagnostics)
      #else:
      #  self.get_logger().warn('section has no diagnostics')
    else:
      self.get_logger().warn('section is empty')

  def diagnostic_task(self, stat):
    stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'pipeline ok')
    stat.add('pipeline state',        Gst.Element.state_get_name(self.pipe.current_state))
    clock = self.pipe.get_pipeline_clock()
    stat.add('clock resolution (ns)', str(clock.get_resolution()))
    stat.add('clock time (ns)',       str(clock.get_time()))
    stat.add('clock is synced',       str(clock.is_synced()))
    base_time = self.pipe.get_base_time()
    stat.add('pipeline base_time ', str(base_time))
    user_data = {'bin_dir' : 'pipeline', 'stat' : stat}
    iterator = self.pipe.iterate_elements()
    iterator.foreach(self.foreach_element_diagnostics, user_data)
    self.get_logger().info('log')
    return stat


  def foreach_element_diagnostics(self, item, user_data):
    stat = user_data['stat']
    bin_dir = user_data['bin_dir'] + '/' + item.name


    (last_ret, state, pending) = item.get_state(0)
    state_string = Gst.Element.state_change_return_get_name(last_ret)
    state_string += ', ' + Gst.Element.state_get_name(state)
    if pending != Gst.State.VOID_PENDING:
      state_string += ', ' + Gst.Element.state_get_name(pending)

    stat.add(bin_dir + ' state',     state_string)

    if type(item) == Gst.Bin:

      user_data = {'bin_dir' : bin_dir , 'stat' : stat}
      iterator = item.iterate_elements()
      iterator.foreach(self.foreach_element_diagnostics, user_data)

