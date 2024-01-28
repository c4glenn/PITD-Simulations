#!/usr/bin/env python3
#
# Copyright 2024 Philip Smith Working under UNCC COAR LAB
# 
# Authors: Philip Smith

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    #robot_model = LaunchConfiguration('robot_model', default="turtlebot3")

    #if robot_model == "turtlebot3":
    if True:
        TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
        model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
        urdf_path = os.path.join(
            get_package_share_directory('pitd_gazebo'),
            'models',
            model_folder,
            'model.sdf'
        )
    #else:
        #print(f"unkown model {robot_model}")
    
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')


    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')

    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld
