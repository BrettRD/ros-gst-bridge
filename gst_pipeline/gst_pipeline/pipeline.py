import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GObject


import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from ament_index_python.packages import get_package_prefix

import diagnostic_updater
import diagnostic_msgs

import asyncio



import os

# XXX this is an awful hack to deal with broken param type mappings in galactic.
# https://github.com/ros2/rclpy/issues/829#issuecomment-937517881
rosdistro=os.environ['ROS_DISTRO']
if rosdistro == 'galactic':
  distro_dynamic_typing={'dynamic_typing':True}
else:
  distro_dynamic_typing={}



class Pipeline(Node):
  def __init__(self, node_name='pipeline_node',
    param_prefix='',
    gst_plugin_paths = [], # list of strings
    gst_plugin_ros_packages = [], # list of tuples (package_name,sub_dir)
    gst_plugins_required = [] # list of strings
  ):
    super().__init__(node_name)
    self.registry = Gst.Registry()
    self.registry.connect('plugin-added', self.plugin_added)
    self.node_name = node_name
    # pull additional info from parameter server
    (paths, packages, plugins) = self.fetch_params(param_prefix)
    self.gst_plugin_paths = gst_plugin_paths + paths
    self.gst_plugin_ros_packages = gst_plugin_ros_packages + packages
    self.gst_plugins_required = gst_plugins_required + plugins

    # unpack ros packages into paths
    for (pack, subdir) in self.gst_plugin_ros_packages:
      plugin_path = os.path.join(get_package_prefix(pack), subdir)
      self.gst_plugin_paths.append(plugin_path)

    self.get_logger().debug('additional gst plugin paths are ' + str(paths))
    self.get_logger().debug('required gst plugins are ' + str(plugins))

    self.add_plugin_paths(self.gst_plugin_paths)
    self.check_plugins(self.gst_plugins_required)

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
    self.updater.force_update()
    self.diagnostic_timer = self.create_timer(0.5, self.updater.update)

  def fetch_params(self, param_prefix=''):
    if param_prefix != '':
      param_prefix = param_prefix + '.'
    self.declare_parameters(
      namespace=param_prefix,
      parameters=[
        (param_prefix + 'gst_plugin_paths', [], ParameterDescriptor(
          name='gst_plugin_paths',
          type=ParameterType.PARAMETER_STRING_ARRAY,
          **distro_dynamic_typing,
          )
        ),
        (param_prefix + 'gst_plugins_required', [], ParameterDescriptor(
          name='gst_plugins_required',
          type=ParameterType.PARAMETER_STRING_ARRAY,
          **distro_dynamic_typing,
          )
        ),
        (param_prefix + 'gst_plugin_ros_packages', [], ParameterDescriptor(
          name='gst_plugin_ros_packages',
          type=ParameterType.PARAMETER_STRING_ARRAY,
          **distro_dynamic_typing,
          )
        ),
      ]
    )
    paths =         self.get_parameter(param_prefix + 'gst_plugin_paths').value
    plugins =       self.get_parameter(param_prefix + 'gst_plugins_required').value
    packages = []
    package_list =  self.get_parameter(param_prefix + 'gst_plugin_ros_packages').value

    for pack in package_list:
      subdir = self.declare_parameter(param_prefix + 'gst_plugin_ros_package_subdirs.' + pack).value
      if subdir == None:
        self.get_logger().error('parameter "'+ param_prefix + 'gst_plugin_ros_package_subdirs' + '.' + pack + '" not found')
      else:
        packages.append( (pack,subdir) )

    return (paths, packages, plugins)


  async def async_task(self):
    while True:
      rclpy.spin_once(self, timeout_sec=0)
      await asyncio.sleep(0.1)


  def on_status_changed(self, bus, message):
    (oldstate, newstate, pending) = message.parse_state_changed()
    logmsg = str(message.timestamp) + ' status_changed ' + message.src.name
    logmsg += ' from ' + Gst.Element.state_get_name(oldstate)
    logmsg += ' to ' + Gst.Element.state_get_name(newstate)
    if pending != Gst.State.VOID_PENDING:
      logmsg += ' targetting ' + Gst.Element.state_get_name(pending)
    self.get_logger().debug(logmsg)


  def on_eos(self, bus, message):
    logmsg = str(message.timestamp) + ' eos message from ' + message.src.name
    self.get_logger().info(logmsg)

  def on_info(self, bus, message):
    logmsg = str(message.timestamp) + ' info message from ' + message.src.name
    logmsg += ' ' + str(message.parse_info_details())
    self.get_logger().info(logmsg)

  def on_error(self, bus, message):
    logmsg = str(message.timestamp) + ' error message from ' + message.src.name
    logmsg += ' ' + str(message.parse_error())
    self.get_logger().error(logmsg)

  def start_pipeline(self):
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
    # iteratively report on all bins in this pipeline
    user_data = {'bin_dir' : 'pipeline', 'stat' : stat}
    iterator = self.pipe.iterate_elements()
    iterator.foreach(self.foreach_element_diagnostics, user_data)
    return stat


  def foreach_element_diagnostics(self, item, user_data):
    stat = user_data['stat']
    bin_dir = user_data['bin_dir'] + '/' + item.name
    (last_ret, state, pending) = item.get_state(0)
    # gst element states return if they succeeded in their last transition,
    #    their current state, and (optionally) their next state
    state_string = Gst.Element.state_change_return_get_name(last_ret)
    state_string += ', ' + Gst.Element.state_get_name(state)
    if pending != Gst.State.VOID_PENDING:
      state_string += ', ' + Gst.Element.state_get_name(pending)
    # report on the state of the bin
    stat.add(bin_dir + ' state', state_string)
    #recursively inspect all bins until all child elements are found
    if type(item) == Gst.Bin:
      user_data = {'bin_dir' : bin_dir , 'stat' : stat}
      iterator = item.iterate_elements()
      iterator.foreach(self.foreach_element_diagnostics, user_data)

  # XXX functional style list check can be swapped for something more legible
  def check_plugins(self, needed):
    # XXX pull the list from the parameter server
    missing = list(filter(lambda p: self.registry.get().find_plugin(p) is None, needed))
    if len(missing):
      self.get_logger().warn('Missing gstreamer plugins: ' + str(missing))
      return False
    else:
      self.get_logger().debug('all plugins accounted for')
    return True



  def add_plugin_paths(self, gst_plugin_paths):
    for path in gst_plugin_paths:
      self.get_logger().debug('scanning path "' + path + '"')
      if self.registry.scan_path(path):
        self.get_logger().debug('registry changed')
      else:
        self.get_logger().warn('no plugins found in path "' + path + '"')



  def plugin_added(self, registry, plugin):
    self.get_logger().info('plugin "' + plugin.get_name() + '" added')
    self.get_logger().debug('plugin filename: "' + plugin.get_filename() + '"')
    self.get_logger().debug('plugin description: "' + plugin.get_description() + '"')
    if None == plugin.load():
      self.get_logger().error('plugin load error')
    else:
      self.get_logger().debug('plugin loaded')
