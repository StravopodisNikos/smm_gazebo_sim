from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("smm_gazebo_sim")
    smm_synthesis_share = get_package_share_directory("smm_synthesis")
    smm_synthesis_parent = os.path.dirname(smm_synthesis_share)

    default_world = os.path.join(
        pkg_share,
        "worlds",
        "smm_empty_world.sdf",
    )

    default_xacro = os.path.expanduser(
        "~/ros2_ws/src/smm_class_pkgs/smm_synthesis/urdf/6dof/smm_structure_anatomy_assembly_6dof.xacro"
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo Sim world SDF file.",
    )

    xacro_path_arg = DeclareLaunchArgument(
        "xacro_path",
        default_value=default_xacro,
        description="Absolute path to the SMM xacro file.",
    )

    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="smm",
        description="Name of the spawned Gazebo entity.",
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            smm_synthesis_parent,
            ":",
            smm_synthesis_share,
            ":",
            pkg_share,
        ],
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro",
                " ",
                LaunchConfiguration("xacro_path"),
            ]
        ),
        value_type=str,
    )

    gz_sim = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-r",
            LaunchConfiguration("world"),
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_smm",
                output="screen",
                arguments=[
                    "-name",
                    LaunchConfiguration("robot_name"),
                    "-topic",
                    "robot_description",
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.05",
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            set_gz_resource_path,
            world_arg,
            xacro_path_arg,
            robot_name_arg,
            gz_sim,
            robot_state_publisher,
            spawn_robot,
        ]
    )