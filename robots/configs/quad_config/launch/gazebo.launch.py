import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    config_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="quad_config"
    ).find("quad_config")
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="quad_description"
    ).find("quad_description")

    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    default_model_path = os.path.join(descr_pkg_share, "xacro/robot.xacro")
    default_world_path = os.path.join(config_pkg_share, "worlds/default.world")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="quad", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path, description="Gazebo world path"
    )
    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )
    declare_headless = DeclareLaunchArgument(
        "headless", default_value="false", description="Run Gazebo without GUI"
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    # Standing hip height ~0.22 m; spawn a bit higher so feet settle onto ground.
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.30")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )

    bringup_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_bringup"),
                "launch",
                "bringup.launch.py",
            )
        ),
        launch_arguments={
            "description_path": default_model_path,
            "joints_map_path": joints_config,
            "links_map_path": links_config,
            "gait_config_path": gait_config,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "gazebo": "true",
            "lite": LaunchConfiguration("lite"),
            "rviz": LaunchConfiguration("rviz"),
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "true",
            "close_loop_odom": "true",
        }.items(),
    )

    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_gazebo"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "world": LaunchConfiguration("world"),
            "lite": LaunchConfiguration("lite"),
            "world_init_x": LaunchConfiguration("world_init_x"),
            "world_init_y": LaunchConfiguration("world_init_y"),
            "world_init_z": LaunchConfiguration("world_init_z"),
            "world_init_heading": LaunchConfiguration("world_init_heading"),
            "gui": LaunchConfiguration("gui"),
            "headless": LaunchConfiguration("headless"),
            "close_loop_odom": "true",
            "description_path": default_model_path,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_rviz,
            declare_robot_name,
            declare_lite,
            declare_gazebo_world,
            declare_gui,
            declare_headless,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            bringup_ld,
            gazebo_ld,
        ]
    )
