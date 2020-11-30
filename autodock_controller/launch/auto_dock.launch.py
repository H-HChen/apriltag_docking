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
controll_dir = get_package_share_directory('auto_dock')
controll_launch_dir = os.path.join(controll_dir, 'launch')
config = os.path.join(get_package_share_directory('auto_dock'), 'param', 'neuronbot.yaml') 

def generate_launch_description():

    bringup_cmd_group = GroupAction([
    Node(
        package='auto_dock',
        executable='controller',
        name='autodock_controller',
        output='screen',
        parameters = [config]),
    
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tag_launch_dir,'tag_gazebo.launch.py')))
    ])

    ld = LaunchDescription()
    ld.add_action(bringup_cmd_group)

    return ld
