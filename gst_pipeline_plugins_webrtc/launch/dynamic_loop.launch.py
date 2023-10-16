# This example shows how to link the webrtc bin into a filter loop
#  The sink pipeline must have a static pad that is not linked.
#  This dangling pad will prompt the connection logic to search for a
#  named element, disconnect its sources, and connect the dangling pad there.



from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  config_filename = LaunchConfiguration('config_filename')
  config_filepath = LaunchConfiguration('config_filepath')

  return LaunchDescription([

    DeclareLaunchArgument('config_filename',
      default_value='dynamic_loop'),

    DeclareLaunchArgument('config_filepath', default_value=[
      TextSubstitution(text=os.path.join(
        get_package_share_directory('gst_pipeline_plugins_webrtc'),
        'config', '')),
      config_filename, TextSubstitution(text='.config.yaml')]),

    Node(
      name='gst_pipeline_node',
      package='gst_pipeline',
      executable='gst_pipeline_node',
      #arguments=['--ros-args', '--log-level', 'debug'],
      parameters=[config_filepath],
      #prefix=['xterm -e gdb -ex=r --args'],
    ),
  ])
  