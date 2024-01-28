#!/usr/bin/env python3
#
# Copyright 2024 Philip Smith Working under UNCC COAR LAB
# 
# Authors: Philip Smith

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    #-----------------------------------------------------
    #                   Launch Variables
    #-----------------------------------------------------
    #create the launch file directory variable to use to find other needed launch files
    launch_file_dir = os.path.join(
        get_package_share_directory('pitd_gazebo'),
        'launch'
    )

    #Gazebo Sim Variables
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    #Sim Variables 
    defender_sensing_radius = 5

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')


    world = os.path.join(
        get_package_share_directory('pitd_gazebo'),
        'worlds',
        'circular_world.world'
    )

    #-----------------------------------------------------
    #               Gazebo Launch Actions
    #-----------------------------------------------------

    #Launch the Gazebo Server with the selected world
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    #Launch the Gazebo Client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    #-----------------------------------------------------
    #               Defender Launch Actions
    #-----------------------------------------------------

    #spawn defender
    spawn_defender_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_defender.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )


    #-----------------------------------------------------
    #             Generate Launch Description
    #-----------------------------------------------------

    ld = LaunchDescription()

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_defender_cmd)

    return ld