# Copyright (c) 2021 Juan Miguel Jimeno
# Adapted for the custom SolidWorks-exported quad robot.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_package = FindPackageShare("quad_config")
    descr_package = FindPackageShare("quad_description")

    joints_config = PathJoinSubstitution(
        [this_package, "config", "joints", "joints.yaml"]
    )
    gait_config = PathJoinSubstitution(
        [this_package, "config", "gait", "gait.yaml"]
    )
    links_config = PathJoinSubstitution(
        [this_package, "config", "links", "links.yaml"]
    )
    description_path = PathJoinSubstitution(
        [descr_package, "xacro", "robot.xacro"]
    )
    bringup_launch_path = PathJoinSubstitution(
        [FindPackageShare("champ_bringup"), "launch", "bringup.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="robot_name",
                default_value="",
                description="Set robot name for multi robot",
            ),
            DeclareLaunchArgument(
                name="sim",
                default_value="false",
                description="Enable use_sim_time",
            ),
            DeclareLaunchArgument(
                name="rviz",
                default_value="false",
                description="Run rviz",
            ),
            DeclareLaunchArgument(
                name="hardware_connected",
                default_value="false",
                description="Set true if connected to a physical robot",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup_launch_path),
                launch_arguments={
                    "description_path": description_path,
                    "use_sim_time": LaunchConfiguration("sim"),
                    "robot_name": LaunchConfiguration("robot_name"),
                    "gazebo": LaunchConfiguration("sim"),
                    "rviz": LaunchConfiguration("rviz"),
                    "hardware_connected": LaunchConfiguration("hardware_connected"),
                    "publish_foot_contacts": "true",
                    "close_loop_odom": "true",
                    "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
                    "joints_map_path": joints_config,
                    "links_map_path": links_config,
                    "gait_config_path": gait_config,
                }.items(),
            ),
        ]
    )
