import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
def generate_launch_description():

    config_filename = launch.substitutions.LaunchConfiguration('config_filename')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    return launch.LaunchDescription([
        #launch.actions.DeclareLaunchArgument('config_filename', default_value='example'),
        launch.actions.DeclareLaunchArgument('config_filename', default_value='simpleparams'),

        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('gst_bridge'), 'config', '')),
            config_filename, launch.substitutions.TextSubstitution(text='.config.yaml')]),

        launch_ros.actions.Node(
            node_name='gst_pipeline_node',
            package='gst_pipeline',
            node_executable='pipeline_node',
            parameters=[config_filepath]),
    ])
    