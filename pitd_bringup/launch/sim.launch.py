import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node





def generate_launch_description():

    pitd_gazebo_pkg = get_package_share_directory('pitd_gazebo')

    defender_sensing_radius = LaunchConfiguration('defender_sensing_radius', default=5)
    attacker_speed = LaunchConfiguration('attacker_speed', default=.8)
    defender_strategy = LaunchConfiguration('defender_strategy', default='paper_1')
    attacker_sensing_radius = LaunchConfiguration('attacker_sensing_radius', default=2)
    target_radius = LaunchConfiguration('target_radius', default=10)
    attack_spawn_pattern = LaunchConfiguration('attack_spawn_pattern', default=1)
    capture_distance = LaunchConfiguration('capture_distance', default=0.01)

    startup_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pitd_gazebo_pkg, 'launch', 'circular_world.launch.py')
        ),
        launch_arguments={'defender_sensing_radius': defender_sensing_radius}.items()
    )

    gazebo_spawner = Node(
        package='pitd_gazebo',
        executable='gazebo_spawner.py',
        output='screen'
    )

    pathing_controller = Node(
        package='pitd_controllers',
        executable='pathing',
        arguments=[
            '-attacker_speed', attacker_speed
        ],
        output='screen'
    )

    attacker_controller = Node(
        package='pitd_controllers',
        executable='attacker',
        arguments=[
            '-attacker_speed', attacker_speed
        ],
        output='screen'
    )

    defender_controller = Node(
        package='pitd_controllers',
        executable='defender',
        arguments=[
            '-defender_sensing_radius', defender_sensing_radius,
            '-defender_strategy', defender_strategy,
            '-attacker_speed', attacker_speed,
            '-attacker_sensing_radius', attacker_sensing_radius,
            "-target_radius", target_radius
        ],
        output='screen'
    )

    world_controller = Node(
        package='pitd_controllers',
        executable='world',
        arguments=[
            '-attack_spawn_pattern', attack_spawn_pattern,
            '-defense_sensing_radius', defender_sensing_radius,
            '-target_radius', target_radius,
            '-capture_distance', capture_distance
        ],
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