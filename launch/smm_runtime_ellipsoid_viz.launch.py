from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def bool_and(arg_a, arg_b):
    return IfCondition(
        PythonExpression([
            "'", LaunchConfiguration(arg_a), "'.lower() == 'true' and '",
            LaunchConfiguration(arg_b), "'.lower() == 'true'"
        ])
    )


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            "start_ellipsoid_viz",
            default_value="true",
            description="Start runtime ellipsoid computation and RViz visualization."
        ),

        DeclareLaunchArgument(
            "ellipsoid_type",
            default_value="kinematic",
            description="Ellipsoid source/type. Currently supported: kinematic."
        ),

        DeclareLaunchArgument(
            "yaml_base_dir",
            default_value="/home/nikos/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml",
            description="Live generated SMM synthesis YAML directory."
        ),

        DeclareLaunchArgument(
            "joint_states_topic",
            default_value="/joint_states",
            description="JointState topic used for online ellipsoid computation."
        ),

        DeclareLaunchArgument(
            "frame_id",
            default_value="base_plate",
            description="Frame used in the published ManipulabilityEllipsoid message."
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock."
        ),

        DeclareLaunchArgument(
            "kinematic_ellipsoid_executable",
            default_value="smm_kinematic_manipulability_ndof_node",
            description="Executable name of the smm_metrics kinematic ellipsoid node."
        ),

        DeclareLaunchArgument(
            "show_kinematic_trans_ellipsoid",
            default_value="true",
            description="Publish and visualize translational kinematic manipulability ellipsoid."
        ),

        DeclareLaunchArgument(
            "show_kinematic_rot_ellipsoid",
            default_value="false",
            description="Publish and visualize rotational kinematic manipulability ellipsoid."
        ),

        DeclareLaunchArgument(
            "kinematic_trans_msg_topic",
            default_value="/smm/kinematic_manipulability_ellipsoid_trans_ndof",
            description="ManipulabilityEllipsoid message topic for translational kinematic ellipsoid."
        ),

        DeclareLaunchArgument(
            "kinematic_rot_msg_topic",
            default_value="/smm/kinematic_manipulability_ellipsoid_rot_ndof",
            description="ManipulabilityEllipsoid message topic for rotational kinematic ellipsoid."
        ),

        DeclareLaunchArgument(
            "kinematic_trans_marker_topic",
            default_value="/smm_viz/kinematic_manipulability_ellipsoid_trans_ndof",
            description="RViz marker topic for translational kinematic ellipsoid body."
        ),

        DeclareLaunchArgument(
            "kinematic_trans_axes_topic",
            default_value="/smm_viz/kinematic_manipulability_ellipsoid_trans_axes_ndof",
            description="RViz MarkerArray topic for translational kinematic ellipsoid axes."
        ),

        DeclareLaunchArgument(
            "kinematic_rot_marker_topic",
            default_value="/smm_viz/kinematic_manipulability_ellipsoid_rot_ndof",
            description="RViz marker topic for rotational kinematic ellipsoid body."
        ),

        DeclareLaunchArgument(
            "kinematic_rot_axes_topic",
            default_value="/smm_viz/kinematic_manipulability_ellipsoid_rot_axes_ndof",
            description="RViz MarkerArray topic for rotational kinematic ellipsoid axes."
        ),

        # ---------------------------------------------------------------------
        # 1) Computation node: smm_metrics
        # ---------------------------------------------------------------------
        Node(
            package="smm_metrics",
            executable=LaunchConfiguration("kinematic_ellipsoid_executable"),
            name="smm_kinematic_manipulability_ndof",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_ellipsoid_viz")),
            parameters=[{
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"),
                    value_type=bool
                ),

                "yaml_base_dir": LaunchConfiguration("yaml_base_dir"),

                # Important:
                # The node calls this parameter joint_cmd_topic,
                # but for Gazebo task execution we feed it /joint_states.
                "joint_cmd_topic": LaunchConfiguration("joint_states_topic"),

                "output_topic_trans": LaunchConfiguration("kinematic_trans_msg_topic"),
                "output_topic_rot": LaunchConfiguration("kinematic_rot_msg_topic"),

                "publish_translational": ParameterValue(
                    LaunchConfiguration("show_kinematic_trans_ellipsoid"),
                    value_type=bool
                ),

                "publish_rotational": ParameterValue(
                    LaunchConfiguration("show_kinematic_rot_ellipsoid"),
                    value_type=bool
                ),

                "frame_id": LaunchConfiguration("frame_id"),
            }],
        ),

        # ---------------------------------------------------------------------
        # 2) Translational ellipsoid visualizer: smm_viz_tools
        # ---------------------------------------------------------------------
        Node(
            package="smm_viz_tools",
            executable="smm_ellipsoid_ndof_viz_node",
            name="smm_kinematic_trans_ellipsoid_viz",
            output="screen",
            condition=bool_and(
                "start_ellipsoid_viz",
                "show_kinematic_trans_ellipsoid"
            ),
            parameters=[{
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"),
                    value_type=bool
                ),

                "input_topic": LaunchConfiguration("kinematic_trans_msg_topic"),
                "ellipsoid_topic": LaunchConfiguration("kinematic_trans_marker_topic"),
                "axes_topic": LaunchConfiguration("kinematic_trans_axes_topic"),

                "marker_namespace": "kinematic_trans_ellipsoid_ndof",
                "axes_namespace": "kinematic_trans_axes_ndof",

                "ellipsoid_alpha": 0.25,
                "ellipsoid_color_r": 0.871,
                "ellipsoid_color_g": 0.520,
                "ellipsoid_color_b": 0.150,

                "axis_shaft_diameter": 0.01,
                "axis_head_diameter": 0.02,
                "axis_head_length": 0.03,
            }],
        ),

        # ---------------------------------------------------------------------
        # 3) Rotational ellipsoid visualizer: smm_viz_tools
        # ---------------------------------------------------------------------
        Node(
            package="smm_viz_tools",
            executable="smm_ellipsoid_ndof_viz_node",
            name="smm_kinematic_rot_ellipsoid_viz",
            output="screen",
            condition=bool_and(
                "start_ellipsoid_viz",
                "show_kinematic_rot_ellipsoid"
            ),
            parameters=[{
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"),
                    value_type=bool
                ),

                "input_topic": LaunchConfiguration("kinematic_rot_msg_topic"),
                "ellipsoid_topic": LaunchConfiguration("kinematic_rot_marker_topic"),
                "axes_topic": LaunchConfiguration("kinematic_rot_axes_topic"),

                "marker_namespace": "kinematic_rot_ellipsoid_ndof",
                "axes_namespace": "kinematic_rot_axes_ndof",

                "ellipsoid_alpha": 0.25,
                "ellipsoid_color_r": 0.2,
                "ellipsoid_color_g": 0.5,
                "ellipsoid_color_b": 0.9,

                "axis_shaft_diameter": 0.01,
                "axis_head_diameter": 0.02,
                "axis_head_length": 0.03,
            }],
        ),
    ])