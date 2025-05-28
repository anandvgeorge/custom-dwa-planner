from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    nav2_bringup_dir = FindPackageShare('nav2_bringup').find('nav2_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                FindPackageShare('custom_dwa_planner').find('custom_dwa_planner'),
                'config',
                'nav2_params.yaml'
            )
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'slam': 'True',
                'use_sim_time': 'True',
                'params_file': LaunchConfiguration('params_file'),
                'map': 'dummy.yaml'
            }.items()
        ),
    ])