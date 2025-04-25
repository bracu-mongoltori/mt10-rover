from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node 1: diffdrive
        Node(
            package='mt10_control',
            executable='diff_drive',
            name='diffdrive',
            output='screen',
        ),
    ])
