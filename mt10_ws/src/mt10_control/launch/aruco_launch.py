import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Launch the 'autonomous' node
        Node(
            package='mt10_control',
            executable='zed',
            output='screen',
#            namespace='',
            parameters=[],
        ),

        # Launch the 'serial_control' node
        Node(
            package='mt10_control',
            executable='aruco_new',
            output='screen',
#            namespace='',
            parameters=[],
        ),

        # Include the 'sbg_device_launch.py' launch file
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource(
#                os.path.join(
#                    get_package_share_directory('mt10_control'),
#                    'launch',
#                    'mt_sbg_launch.py'
 #               )
#            ),
#        ),
    ])
