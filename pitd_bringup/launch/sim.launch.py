import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pitd_gazebo_pkg = get_package_share_directory('pitd_gazebo')

    config = os.path.join(get_package_share_directory('pitd_bringup'), 'config', 'config.yaml')

    startup_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pitd_gazebo_pkg, 'launch', 'circular_world.launch.py')
        ),
    )

    gazebo_spawner = Node(
        package='pitd_gazebo',
        executable='gazebo_spawner.py',
        output='screen'
    )

    pathing_controller = Node(
        package='pitd_controllers',
        executable='pathing',
        parameters=[config],
        output='screen'
    )

    attacker_controller = Node(
        package='pitd_controllers',
        executable='attacker',
        arguments=[config],
        output='screen'
    )

    defender_controller = Node(
        package='pitd_controllers',
        executable='defender',
        arguments=[config],
        output='screen'
    )

    world_controller = Node(
        package='pitd_controllers',
        executable='world',
        arguments=[config],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(startup_gazebo)
    ld.add_action(gazebo_spawner)
    ld.add_action(pathing_controller)
    ld.add_action(attacker_controller)
    ld.add_action(defender_controller)
    ld.add_action(world_controller)
    return ld