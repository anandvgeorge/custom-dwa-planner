from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['rviz2', '-d', os.path.join(
                get_package_share_directory('custom_dwa_planner'), 'rviz', 'turtlebot_custom_controller.rviz')],
            output='screen'
        )
    ])