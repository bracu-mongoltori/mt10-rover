import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, ExecuteProcess, LogInfo
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown

def generate_launch_description():
    config = '/home/mt10/mt10_ws/src/mt10_control/config/mt_sbg_config.yaml'


    # SBG driver node
    sbg_node = Node(
        package='sbg_driver',
        executable='sbg_device',
        output='screen',
        parameters=[config]
    )

    


    return LaunchDescription([
        sbg_node,
    ])