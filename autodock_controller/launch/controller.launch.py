import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

tag_detect_dir = get_package_share_directory('apriltag_ros')
tag_launch_dir = os.path.join(tag_detect_dir, 'launch')
config = os.path.join(get_package_share_directory('auto_dock'), 'param', 'neuronbot.yaml') 

def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
            SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

            DeclareLaunchArgument(
                'params_file',
                default_value = config,
                description='Full path to the ROS2 parameters file to use for all launched nodes'),
                
            Node(
                package='auto_dock',
                executable='controller',
                name='autodock_controller',
                output='screen',
                parameters = [params_file])])
