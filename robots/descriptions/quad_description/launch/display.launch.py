"""RViz-only visualization of the SolidWorks-derived quad model (no Gazebo)."""
import os

import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="quad_description"
    ).find("quad_description")
    default_model = os.path.join(pkg_share, "xacro/robot.xacro")
    default_rviz = os.path.join(pkg_share, "rviz/urdf.rviz")

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value=default_model),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_gui", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(LaunchConfiguration("use_gui")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", default_rviz] if os.path.exists(default_rviz) else [],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
