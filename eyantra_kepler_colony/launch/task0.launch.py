#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Task 0 bringup: load the kepler world, then spawn the UR7e arm and the eBot.

Run `ros2 launch eyantra_kepler_colony task0.launch.py`.
'''

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("eyantra_kepler_colony")

    declared_arguments = [
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution(
                [pkg_share, "worlds", "eyantra_kepler_world.world"]
            ),
            description="Absolute path of the SDF world to load.",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Run the Gazebo GUI. Set false for headless.",
        ),
        DeclareLaunchArgument(
            "verbosity",
            default_value="1",
            description="gz sim console verbosity (0-4).",
        ),
        DeclareLaunchArgument(
            "arm",
            default_value="true",
            description="Spawn the UR7e arm.",
        ),
        DeclareLaunchArgument(
            "ebot",
            default_value="true",
            description="Spawn the eBot.",
        ),
        DeclareLaunchArgument(
            "arm_delay", default_value="5.0", description="Delay before the arm spawns."
        ),
        DeclareLaunchArgument(
            "ebot_delay", default_value="6.0", description="Delay before the eBot spawns."
        ),
        DeclareLaunchArgument(
            "spawn_objects_delay",
            default_value="8.0",
            description="Delay before the ores and rocks spawn.",
        ),
    ]

    world_file = LaunchConfiguration("world_file")
    verbosity = LaunchConfiguration("verbosity")

    server_only_flag = PythonExpression(
        ["'-s ' if '", LaunchConfiguration("gui"), "'.lower() == 'false' else ''"]
    )

    resource_paths = [
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([pkg_share, "models"])
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            PathJoinSubstitution([pkg_share, "models", "rocks"])
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([pkg_share, "worlds"])
        ),
    ]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": [server_only_flag, "-r -v ", verbosity, " ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )

    arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_description"), "launch", "spawn_ur7e.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("arm")),
    )

    ebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ebot_description"), "launch", "spawn_ebot.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("ebot")),
    )

    spawn_objects = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_share, "launch", "spawn_objects.launch.py"]
            )
        ),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        declared_arguments + resource_paths + [
            gz_sim,
            clock_bridge,
            TimerAction(period=LaunchConfiguration("arm_delay"), actions=[arm]),
            TimerAction(period=LaunchConfiguration("ebot_delay"), actions=[ebot]),
            TimerAction(period=LaunchConfiguration("spawn_objects_delay"), actions=[spawn_objects]),
        ]
    )
