import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():

	return LaunchDescription([
		Node(
			package='camera',
			executable='talker_stereo',
			parameters=[{'id_cam':"0"}],
			name="camera0",
			output='screen',
			remappings=[
				("/imu","/imu0"),
				("/events","/events0")
			]
		),
		Node(
			package='camera',
			executable='talker_stereo',
			parameters=[{'id_cam':"1"}],
			name="camera1",
			output='screen',
			remappings=[
				("/imu","/imu1"),
				("/events","/events1")
			]
		),
		Node(
			package='camera',
			executable='event',
			parameters=[],
			name="event0",
			output='screen',
			remappings=[
				("/events","/events0")
			]
		),
		Node(
			package='camera',
			executable='event',
			parameters=[],
			name="event1",
			output='screen',
			remappings=[
				("/events","/events1")
			]
		)
		])
