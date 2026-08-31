#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Spawn the UR7e arm into a running Gazebo world and bring up its controllers.

    ros2 launch ur_description spawn_ur7e.launch.py
'''

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur7e",
            description="UR model to build.",
        ),
        DeclareLaunchArgument(
            "name",
            default_value="ur7e",
            description="Model name in gz.",
        ),

        DeclareLaunchArgument("x", default_value="-0.597278"),
        DeclareLaunchArgument("y", default_value="1.8653"),
        DeclareLaunchArgument("z", default_value="0.7887"),
        DeclareLaunchArgument("roll", default_value="0.0"),
        DeclareLaunchArgument("pitch", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="3.1415927"),
        DeclareLaunchArgument("world_z", default_value="0.3862"),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Open RViz with the arm display config.",
        ),
    ]

    ur_type = LaunchConfiguration("ur_type")
    name = LaunchConfiguration("name")
    tf_prefix = ""
    launch_rviz = LaunchConfiguration("launch_rviz")

    controllers_file = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur_controllers.yaml"]
    )
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "urdf", "ur_gz.urdf.xacro"]
    )

    robot_description = {

        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ", xacro_file,
                " ", "name:=", name,
                " ", "ur_type:=", ur_type,
                " ", "tf_prefix:=", tf_prefix,
                " ", "force_abs_paths:=true",
                " ", "simulation_controllers:=", controllers_file,
            ]),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen",
    )

    world_to_arm_mount = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_arm_mount",
        arguments=[
            "--x", LaunchConfiguration("x"),
            "--y", LaunchConfiguration("y"),
            "--z", PythonExpression(
                ["str(", LaunchConfiguration("z"), " - ",
                 LaunchConfiguration("world_z"), ")"]),
            "--roll", LaunchConfiguration("roll"),
            "--pitch", LaunchConfiguration("pitch"),
            "--yaw", LaunchConfiguration("yaw"),
            "--frame-id", "world",
            "--child-frame-id", "arm_mount",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    flange_to_end_effector = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="flange_to_end_effector",
        arguments=["--frame-id", "flange", "--child-frame-id", "end_effector"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", name,
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-R", LaunchConfiguration("roll"),
            "-P", LaunchConfiguration("pitch"),
            "-Y", LaunchConfiguration("yaw"),
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    def gz_bridge(name, config, condition=None):
        return Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=name,
            parameters=[{
                "config_file": PathJoinSubstitution(
                    [FindPackageShare("ur_description"), "config", config]),
                "use_sim_time": True,
            }],
            condition=condition,
            output="screen",
        )

    bridge = gz_bridge("ur_bridge", "gz_bridge_sensors.yaml")
    camera_bridge = gz_bridge("ur_camera_bridge", "gz_bridge_camera.yaml")
    camera_depth_bridge = gz_bridge(
        "ur_camera_depth_bridge", "gz_bridge_camera_depth.yaml")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("ur_description"), "rviz", "ur7e.rviz"]
            ),
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(launch_rviz),
        output="screen",
    )

    def spawner(controller):
        args = [
            controller,
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
            "--switch-timeout", "60",
        ]
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=args,
            parameters=[{"use_sim_time": True}],
            output="screen",
        )

    joint_state_broadcaster_spawner = spawner("joint_state_broadcaster")

    forward_position_controller_spawner = spawner("forward_position_controller")

    cartesian_servo = Node(
        package="ur_description",
        executable="cartesian_servo",
        name="cartesian_servo_node",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher,
            world_to_arm_mount,
            flange_to_end_effector,
            spawn_entity,
            bridge,
            camera_bridge,
            camera_depth_bridge,
            rviz,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[forward_position_controller_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=forward_position_controller_spawner,
                    on_exit=[cartesian_servo],
                )
            ),
        ]
    )
