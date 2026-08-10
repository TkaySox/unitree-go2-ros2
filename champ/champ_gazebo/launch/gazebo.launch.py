import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,
                            IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression


def generate_launch_description():

    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless")
    lite = LaunchConfiguration("lite")
    ros_control_file = LaunchConfiguration("ros_control_file")
    world_init_x = LaunchConfiguration("world_init_x")
    world_init_y = LaunchConfiguration("world_init_y")
    world_init_z = LaunchConfiguration("world_init_z")
    world_init_heading = LaunchConfiguration("world_init_heading")
    gazebo_world = LaunchConfiguration("world")
    gz_pkg_share = launch_ros.substitutions.FindPackageShare(package="champ_gazebo").find(
        "champ_gazebo"
    )

    declare_robot_name = DeclareLaunchArgument("robot_name", default_value="champ")
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="True")
    declare_gui = DeclareLaunchArgument("gui", default_value="True")
    declare_headless = DeclareLaunchArgument("headless", default_value="False")
    declare_lite = DeclareLaunchArgument("lite", default_value="False")
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=os.path.join(gz_pkg_share, "config/ros_control.yaml"),
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=os.path.join(gz_pkg_share, "worlds/default.world")
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.6")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.6"
    )

    pkg_share = launch_ros.substitutions.FindPackageShare(package="champ_description").find("champ_description")
    default_model_path = os.path.join(pkg_share, "urdf/champ.urdf.xacro")

    declare_description_path = DeclareLaunchArgument(name="description_path", default_value=default_model_path, description="Absolute path to robot urdf file")

    # Make sure gz-sim can find the meshes referenced by go2_description /
    # champ_description / velodyne_description when they are loaded as an
    # SDF model (xacro -> URDF -> SDF happens implicitly when the robot is
    # spawned from the /robot_description topic).
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join(
            [
                os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
                os.path.dirname(pkg_share),
                gz_pkg_share,
            ]
        ),
    )

    # Gazebo Harmonic replaces gzserver/gzclient with a single gz sim
    # process. "-r" runs the world unpaused. "-s" (server only, no GUI) is
    # added whenever the caller asked for headless mode OR gui:=false --
    # both launch args are honored here, and the comparison is
    # case-insensitive since launch arguments arrive as raw strings
    # (headless:=true / headless:=True / gui:=false all need to work).
    gz_args = PythonExpression(
        [
            "'-r ' + '",
            gazebo_world,
            "' + (' -s' if ('",
            headless,
            "'.lower() == 'true' or '",
            gui,
            "'.lower() == 'false') else '')",
        ]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    robot_description = {"robot_description": Command(["xacro ", LaunchConfiguration("description_path")])}

    start_gazebo_spawner_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            robot_name,
            "-topic",
            "robot_description",
            "-x",
            world_init_x,
            "-y",
            world_init_y,
            "-z",
            world_init_z,
            "-R",
            "0",
            "-P",
            "0",
            "-Y",
            world_init_heading,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Bridges gz-sim's clock, IMU and ground-truth odometry topics to ROS 2.
    # Sensor-specific bridges (2D laser / Velodyne) are declared by the robot
    # config package (e.g. go2_config) since not every robot has them.
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="champ_gz_bridge",
        output="screen",
        arguments=[
            "--ros-args",
            "-p",
            "config_file:=" + os.path.join(gz_pkg_share, "config", "gz_bridge.yaml"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Foot contact sensors are disabled. quadruped_controller publishes
    # gait-phase foot contacts instead (publish_foot_contacts:=true).
    # Re-enable contact_bridge + contact_sensor below if you want real
    # Gazebo contact feedback (historically halved RTF).
    # contact_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     name="champ_contact_bridge",
    #     output="screen",
    #     arguments=[
    #         "--ros-args",
    #         "-p",
    #         "config_file:=" + os.path.join(gz_pkg_share, "config", "contact_bridge.yaml"),
    #     ],
    #     parameters=[{"use_sim_time": use_sim_time}],
    # )
    # contact_sensor = Node(
    #     package="champ_gazebo",
    #     executable="contact_sensor",
    #     output="screen",
    #     parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}, links_config],
    # )

    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_states_controller", "--controller-manager-timeout", "60"],
        output="screen",
    )

    load_joint_trajectory_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_effort_controller", "--controller-manager-timeout", "60"],
        output="screen",
    )

    # joint_group_position_controller
    return LaunchDescription(
        [
            declare_robot_name,
            declare_use_sim_time,
            declare_gui,
            declare_headless,
            declare_lite,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            declare_description_path,
            set_gz_resource_path,
            gz_sim,
            start_gazebo_spawner_cmd,
            gz_bridge,
            # contact_bridge,  # disabled — no foot contact sensors
            load_joint_state_controller,
            # load_joint_trajectory_position_controller
            load_joint_trajectory_effort_controller,
            # contact_sensor,  # disabled — gait-phase contacts from controller
        ]
    )
