import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	#config = os.path.join(
	#	get_package_share_directory('sbg_driver'),
	#	'config',
	#	'example',
	#	'ellipse_D_default.yaml'
	#)
    #config = '/home/mt10/mt10_ws/src/mt10_control/config/mt_sbg_config.yaml'

	return LaunchDescription([
		Node(
			package='sbg_driver',
		#	name='sbg_device_1',
			executable = 'sbg_device_mag',
			output = 'screen',
			parameters = ['/home/mt10/mt10_ws/src/mt10_control/config/mt_sbg_config.yaml']
		)
	])
