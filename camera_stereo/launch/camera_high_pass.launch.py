import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

import dv_processing as dv

def generate_launch_description():
	camera = dv.io.discoverDevices()
	
	frame = False
	
	for cam in camera:
		if(cam.split("_")[0]=="DAVIS346"):
			print(cam.split("_")[0])
			frame =True
	
	frame = False
	
	number_of_topics=DeclareLaunchArgument(
		'number_of_topics',
		default_value='3'
	)
	
	cap_of_event=DeclareLaunchArgument(
		'cap_of_event',
		default_value='5000'
	)
	
	frame_activate=DeclareLaunchArgument(
		'frame_activate',
		default_value=str(frame)
	)
	
	imu_activate=DeclareLaunchArgument(
		'imu_activate',
		default_value='False'
	)
	trigger_activate=DeclareLaunchArgument(
		'trigger_activate',
		default_value='False'
	)
	
	
	kernel_size_high_pass=DeclareLaunchArgument(
		'kernel_size_high_pass',
		default_value='3'
	)
	
	topic_name_in_high_pass=DeclareLaunchArgument(
		'topic_name_in_high_pass',
		default_value='ACCUMULATOR_OUT'
	
	)
	
	topic_name_in_image_visu=DeclareLaunchArgument(
		'topic_name_in_image_visu',
		default_value='HIGH_PASS_OUT'
	
	)
	
	event_topic_name=DeclareLaunchArgument(
		'event_topic_name',
		default_value='unsigned'
	)
	
	parameter_talker = {'number_of_topics':LaunchConfiguration('number_of_topics'),
			    'cap_of_event':LaunchConfiguration('cap_of_event'),
			    'frame_activate':LaunchConfiguration('frame_activate'),
			    'imu_activate':LaunchConfiguration('imu_activate'),
			    'trigger_activate':LaunchConfiguration('trigger_activate')} 
	
	parameter_high_pass = {'number_of_topics':LaunchConfiguration('number_of_topics'),'kernel_size_high_pass':LaunchConfiguration('kernel_size_high_pass'),'topic_name_in_high_pass':LaunchConfiguration('topic_name_in_high_pass'),'event_topic_name':LaunchConfiguration('event_topic_name')}
			   
	parameter_visualizer = {'topic_name_in_image_visu':LaunchConfiguration('topic_name_in_image_visu')}	
	
	parameter_accumulator = {'number_of_topics':LaunchConfiguration('number_of_topics')}	
	
	nodes = []
	nodes.append(number_of_topics)
	nodes.append(cap_of_event)
	nodes.append(frame_activate)
	nodes.append(imu_activate)
	nodes.append(trigger_activate)
	nodes.append(kernel_size_high_pass)
	nodes.append(topic_name_in_high_pass)
	nodes.append(topic_name_in_image_visu)
	nodes.append(event_topic_name)
	nodes.append(Node(package='camera',executable='talker_duo',parameters=[parameter_talker],output='screen'))	
	nodes.append(Node(package='camera',executable='accumulator',parameters=[parameter_accumulator],output='screen'))						
	nodes.append(Node(package='camera',executable='high_pass_filter',parameters=[parameter_high_pass],output='screen'))
	nodes.append(Node(package='camera',executable='eros_visualizer',parameters=[parameter_visualizer],output='screen'))		


	return LaunchDescription(nodes)
