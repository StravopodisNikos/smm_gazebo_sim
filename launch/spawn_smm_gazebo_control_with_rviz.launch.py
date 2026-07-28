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
    /smm_cartesian_controller/jacobian_condition
    /smm_cartesian_controller/jacobian_singular_values
    /smm_cartesian_controller/jacobian_column_norms

    RViz Cartesian goal marker:
    /smm_viz/desired_cartesian_position_marker

    RViz Jacobian condition marker:
    /smm_viz/jacobian_condition_marker
    /smm_viz/jacobian_condition_text
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

    default_world = os.path.join(
        smm_gazebo_sim_share,
        "worlds",
        "smm_empty_world.sdf",
    )

    default_controller_defaults_yaml = os.path.join(
        get_package_share_directory("smm_controllers"),
        "config",
        "controller_defaults.yaml",
    )

    controller_type_arg = DeclareLaunchArgument(
        "controller_type",
        default_value="joint_inverse_dynamics",
        description=(
            "Controller type: position, velocity, effort, joint_pd_effort, "
            "pd_gravity, joint_inverse_dynamics, cartesian_pd_gravity, "
            "cartesian_pose_pd_gravity, cartesian_inv_dyn, "
            "cartesian_robust_inv_dyn, cartesian_robust_adaptive_inv_dyn, "
            "cartesian_robust_impedance."
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

    start_jacobian_condition_marker_arg = DeclareLaunchArgument(
        "start_jacobian_condition_marker",
        default_value="true",
        description="Start RViz marker node for Jacobian condition number visualization.",
    )

    # Kinematic manipulability ellipsoid
    start_ellipsoid_viz_arg = DeclareLaunchArgument(
        "start_ellipsoid_viz",
        default_value="true",
        description="Start runtime manipulability ellipsoid visualization.",
    )

    ellipsoid_type_arg = DeclareLaunchArgument(
        "ellipsoid_type",
        default_value="kinematic",
        description="Ellipsoid type to visualize. Default: kinematic.",
    )

    show_kinematic_trans_ellipsoid_arg = DeclareLaunchArgument(
        "show_kinematic_trans_ellipsoid",
        default_value="true",
        description="Show translational kinematic manipulability ellipsoid.",
    )

    show_kinematic_rot_ellipsoid_arg = DeclareLaunchArgument(
        "show_kinematic_rot_ellipsoid",
        default_value="false",
        description="Show rotational kinematic manipulability ellipsoid.",
    )

    ellipsoid_frame_id_arg = DeclareLaunchArgument(
        "ellipsoid_frame_id",
        default_value="base_plate",
        description="Frame id for manipulability ellipsoid markers.",
    )

    # Parent ags
    data_dir_arg = DeclareLaunchArgument(
        "data_dir",
        default_value="/home/nikos/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml",
        description="Live generated synthesis YAML directory.",
    )

    xacro_path_arg = DeclareLaunchArgument(
        "xacro_path",
        default_value="urdf/6dof/smm_structure_anatomy_assembly_6dof.xacro",
        description="SMM xacro path, relative to smm_synthesis share or absolute.",
    )

    run_synthesis_arg = DeclareLaunchArgument(
        "run_synthesis",
        default_value="true",
        description="Run headless synthesis before spawning Gazebo/control.",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description=(
            "Gazebo Sim world SDF file. Default is the current empty world. "
            "For interaction tests, pass world:=smm_interaction_massage_test.sdf."
        ),
    )

    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="smm",
        description="Gazebo entity name.",
    )

    controller_defaults_yaml_arg = DeclareLaunchArgument(
        "controller_defaults_yaml",
        default_value=default_controller_defaults_yaml,
        description="YAML file containing default controller parameters.",
    )

    # Launch descriptions: Main Gazebo
    gazebo_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_control_launch),
        launch_arguments={
            "controller_type": LaunchConfiguration("controller_type"),
            "start_rqt_plot": LaunchConfiguration("start_rqt_plot"),
            "data_dir": LaunchConfiguration("data_dir"),
            "xacro_path": LaunchConfiguration("xacro_path"),
            "run_synthesis": LaunchConfiguration("run_synthesis"),
            "world": LaunchConfiguration("world"),
            "robot_name": LaunchConfiguration("robot_name"),
            "controller_defaults_yaml": LaunchConfiguration("controller_defaults_yaml"),
        }.items(),
    )

    # Launch description for ellipsoids
    ellipsoid_viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("smm_gazebo_sim"),
                "launch",
                "smm_runtime_ellipsoid_viz.launch.py"
            )
        ),
        launch_arguments={
            "start_ellipsoid_viz": LaunchConfiguration("start_ellipsoid_viz"),
            "ellipsoid_type": LaunchConfiguration("ellipsoid_type"),

            # Use the same live YAML directory already used by synthesis/control.
            "yaml_base_dir": LaunchConfiguration("data_dir"),

            # During Gazebo task execution, compute ellipsoids from real joint states.
            "joint_states_topic": "/joint_states",

            "frame_id": LaunchConfiguration("ellipsoid_frame_id"),
            "use_sim_time": "true",

            # Default desired ellipsoid: kinematic translational.
            "show_kinematic_trans_ellipsoid": LaunchConfiguration(
                "show_kinematic_trans_ellipsoid"
            ),
            "show_kinematic_rot_ellipsoid": LaunchConfiguration(
                "show_kinematic_rot_ellipsoid"
            ),

            "kinematic_trans_msg_topic":
                "/smm/kinematic_manipulability_ellipsoid_trans_ndof",
            "kinematic_rot_msg_topic":
                "/smm/kinematic_manipulability_ellipsoid_rot_ndof",

            "kinematic_trans_marker_topic":
                "/smm_viz/kinematic_manipulability_ellipsoid_trans_ndof",
            "kinematic_trans_axes_topic":
                "/smm_viz/kinematic_manipulability_ellipsoid_trans_axes_ndof",

            "kinematic_rot_marker_topic":
                "/smm_viz/kinematic_manipulability_ellipsoid_rot_ndof",
            "kinematic_rot_axes_topic":
                "/smm_viz/kinematic_manipulability_ellipsoid_rot_axes_ndof",
        }.items()
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

    jacobian_condition_marker_node = Node(
        condition=IfCondition(LaunchConfiguration("start_jacobian_condition_marker")),
        package="smm_viz_tools",
        executable="jacobian_condition_marker_node",
        name="jacobian_condition_marker_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "condition_topic": "/smm_cartesian_controller/jacobian_condition",
                "pose_topic": "/smm_cartesian_controller/current_cartesian_state",
                "marker_topic": "/smm_viz/jacobian_condition_marker",
                "text_topic": "/smm_viz/jacobian_condition_text",
                "fixed_frame": "world",
                "condition_good": 30.0,
                "condition_bad": 1000.0,
                "marker_scale": 0.045,
                "text_scale": 0.055,
                "z_offset": 0.10,
                "publish_rate": 20.0,
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
            start_jacobian_condition_marker_arg,

            data_dir_arg,
            xacro_path_arg,
            run_synthesis_arg,
            world_arg,
            robot_name_arg,
            controller_defaults_yaml_arg,

            start_ellipsoid_viz_arg,
            ellipsoid_type_arg,
            show_kinematic_trans_ellipsoid_arg,
            show_kinematic_rot_ellipsoid_arg,
            ellipsoid_frame_id_arg,

            gazebo_control,
            rviz,
            desired_tcp_marker_node,
            jacobian_condition_marker_node,
            ellipsoid_viz_launch,
        ]
    )