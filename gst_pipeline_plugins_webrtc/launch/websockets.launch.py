
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
      default_value='websockets'),

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
  