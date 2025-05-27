from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'custom_dwa_planner'
    params_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'nav2_params.yaml'
    )

    return LaunchDescription([
        # Start SLAM Toolbox in synchronous mode
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),
        # Node(
        #     package='nav2_recoveries',
        #     executable='recoveries_server',
        #     name='recoveries_server',
        #     output='screen',
        #     parameters=[params_file]
        # ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'slam_toolbox',
                    'controller_server',
                    'planner_server',
                    'bt_navigator',
                    'recoveries_server'
                ]
            }]
        )
    ])
