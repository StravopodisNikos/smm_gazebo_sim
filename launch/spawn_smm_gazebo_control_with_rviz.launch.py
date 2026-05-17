from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    """
    Launch Gazebo + ros2_control + RViz for SMM controller validation.

    This launch file wraps the existing Gazebo/control launch file and starts RViz
    in parallel. Gazebo remains responsible for physics simulation and controller
    execution, while RViz visualizes the robot state, TF tree, and additional
    visualization/debug topics from smm_viz_tools.

    Main data flow:
    Gazebo + gz_ros2_control
    -> joint_state_broadcaster
    -> /joint_states
    -> robot_state_publisher
    -> /tf
    -> RViz RobotModel

    Controller debug topics available for plotting/inspection:

    Joint-space controllers:
    /smm_joint_controller/desired_joint_state
    /smm_joint_controller/error_state
    /smm_joint_controller/q_error_i

    Cartesian-space controllers:
    /smm_cartesian_controller/desired_cartesian_state
    /smm_cartesian_controller/current_cartesian_state
    /smm_cartesian_controller/cartesian_error_state

    RViz Cartesian goal marker:
    /smm_viz/desired_cartesian_position_marker
    """

    smm_gazebo_sim_share = get_package_share_directory("smm_gazebo_sim")
    smm_viz_tools_share = get_package_share_directory("smm_viz_tools")

    gazebo_control_launch = os.path.join(
        smm_gazebo_sim_share,
        "launch",
        "spawn_smm_gazebo_control.launch.py",
    )

    default_rviz_config = os.path.join(
        smm_viz_tools_share,
        "rviz",
        "smm_gazebo_control.rviz",
    )

    controller_type_arg = DeclareLaunchArgument(
        "controller_type",
        default_value="joint_inverse_dynamics",
        description=(
            "Controller type: position, velocity, effort, joint_pd_effort, "
            "pd_gravity, joint_inverse_dynamics, cartesian_pd_gravity."
        ),
    )

    start_rqt_plot_arg = DeclareLaunchArgument(
        "start_rqt_plot",
        default_value="false",
        description="Forwarded to the Gazebo/control launch file.",
    )

    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="Start RViz together with Gazebo.",
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="RViz configuration file.",
    )

    start_desired_tcp_marker_arg = DeclareLaunchArgument(
        "start_desired_tcp_marker",
        default_value="true",
        description="Start desired TCP Cartesian position marker visualizer.",
    )

    gazebo_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_control_launch),
        launch_arguments={
            "controller_type": LaunchConfiguration("controller_type"),
            "start_rqt_plot": LaunchConfiguration("start_rqt_plot"),
        }.items(),
    )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            LaunchConfiguration("rviz_config"),
        ],
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
    )

    desired_tcp_marker_node = Node(
        condition=IfCondition(LaunchConfiguration("start_desired_tcp_marker")),
        package="smm_viz_tools",
        executable="desired_tcp_position_marker_node",
        name="desired_tcp_position_marker_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/smm_cartesian_controller/desired_cartesian_state",
                "marker_topic": "/smm_viz/desired_cartesian_position_marker",
                "fixed_frame": "world",
                "sphere_diameter": 0.01,
            }
        ],
    )

    return LaunchDescription(
        [
            controller_type_arg,
            start_rqt_plot_arg,
            start_rviz_arg,
            rviz_config_arg,
            start_desired_tcp_marker_arg,
            gazebo_control,
            rviz,
            desired_tcp_marker_node,
        ]
    )