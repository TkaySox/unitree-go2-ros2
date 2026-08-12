import os

import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    description_path = LaunchConfiguration("description_path")

    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="quad_description"
    ).find("quad_description")
    default_model_path = os.path.join(pkg_share, "xacro/robot.xacro")

    robot_description = ParameterValue(
        Command(["xacro ", description_path]), value_type=str
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                name="description_path",
                default_value=default_model_path,
                description="Absolute path to robot xacro/urdf",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {"robot_description": robot_description},
                    {"use_tf_static": False},
                    {"publish_frequency": 200.0},
                    {"ignore_timestamp": True},
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
