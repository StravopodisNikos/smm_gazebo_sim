from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os
import subprocess
import yaml


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("smm_gazebo_sim")
    smm_synthesis_share = get_package_share_directory("smm_synthesis")
    smm_synthesis_parent = os.path.dirname(smm_synthesis_share)

    world = LaunchConfiguration("world").perform(context)
    xacro_path = LaunchConfiguration("xacro_path").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    controller_type = LaunchConfiguration("controller_type").perform(context)
    data_dir = LaunchConfiguration("data_dir").perform(context)
    start_rqt_plot = LaunchConfiguration("start_rqt_plot").perform(context).lower() == "true"
    controller_defaults_yaml = LaunchConfiguration("controller_defaults_yaml").perform(context)
    
    data_dir = os.path.expanduser(data_dir)
    controller_defaults_yaml = os.path.expanduser(controller_defaults_yaml)
    os.makedirs(data_dir, exist_ok=True)

    active_joint_names_yaml = os.path.join(data_dir, "active_joint_names.yaml")
    generated_control_xacro = os.path.join(data_dir, "generated_gz_ros2_control.xacro")
    generated_controller_yaml = os.path.join(data_dir, "generated_smm_controllers.yaml")

    valid_controller_types = [
        "position",
        "velocity",
        "effort",
        "joint_pd_effort",
        "pd_gravity",
        "joint_inverse_dynamics",
    ]

    if controller_type not in valid_controller_types:
        raise RuntimeError(
            f"Invalid controller_type='{controller_type}'. "
            f"Allowed values: {', '.join(valid_controller_types)}."
        )

    if not os.path.exists(active_joint_names_yaml):
        raise RuntimeError(
            "[spawn_smm_gazebo_control] active_joint_names.yaml not found:\n"
            f"  {active_joint_names_yaml}\n"
            "Run master_synthesis_ndof.launch.py first."
        )

    with open(active_joint_names_yaml, "r") as f:
        active_joint_data = yaml.safe_load(f)

    active_joint_names = active_joint_data.get("active_joint_names", [])

    if not active_joint_names:
        raise RuntimeError(
            "[spawn_smm_gazebo_control] active_joint_names.yaml contains no active joints."
        )

    dof = len(active_joint_names)

    generator_script = os.path.join(
        pkg_share,
        "..",
        "..",
        "lib",
        "smm_gazebo_sim",
        "generate_smm_gazebo_control_files.py",
    )
    generator_script = os.path.abspath(generator_script)

    if not os.path.exists(controller_defaults_yaml):
        raise RuntimeError(
        "[spawn_smm_gazebo_control] controller_defaults_yaml not found:\n"
        f"  {controller_defaults_yaml}"
    )

    subprocess.run(
        [
            generator_script,
            "--active-joint-names-yaml",
            active_joint_names_yaml,
            "--output-control-xacro",
            generated_control_xacro,
            "--output-controller-yaml",
            generated_controller_yaml,
            "--controller-type",
            controller_type,
            "--update-rate",
            "1000",
            "--controller-defaults-yaml",
            controller_defaults_yaml,
        ],
        check=True,
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

    set_gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            "/opt/ros/kilted/lib",
        ],
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro",
                " ",
                xacro_path,
                " ",
                "use_gz_ros2_control:=true",
                " ",
                "gz_ros2_control_xacro:=",
                generated_control_xacro,
            ]
        ),
        value_type=str,
    )

    gz_sim = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-r",
            world,
        ],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
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
                    robot_name,
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

    spawn_joint_state_broadcaster = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    smm_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "smm_joint_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    spawn_smm_joint_controller = TimerAction(
        period=9.0,
        actions=[
            smm_joint_controller_spawner
        ],
    )

    position_error_topics = [
        f"/smm_joint_controller/q_error_{i}/data"
        for i in range(dof)
    ]

    rqt_plot_error_state = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "rqt_plot",
            "rqt_plot",
            *position_error_topics,
        ],
        output="screen",
    )

    start_rqt_after_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=smm_joint_controller_spawner,
            on_exit=[rqt_plot_error_state],
        )
    )

    actions = [
        set_gz_resource_path,
        set_gz_plugin_path,
        gz_sim,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,
        spawn_joint_state_broadcaster,
        spawn_smm_joint_controller,
    ]

    if start_rqt_plot and controller_type == "joint_inverse_dynamics":
        actions.append(start_rqt_after_controller)

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory("smm_gazebo_sim")

    default_world = os.path.join(
        pkg_share,
        "worlds",
        "smm_empty_world.sdf",
    )

    default_xacro = os.path.expanduser(
        "~/ros2_ws/src/smm_class_pkgs/smm_synthesis/urdf/6dof/smm_structure_anatomy_assembly_6dof.xacro"
    )

    default_data_dir = os.path.expanduser(
        "~/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Gazebo Sim world SDF file.",
            ),
            DeclareLaunchArgument(
                "xacro_path",
                default_value=default_xacro,
                description="Absolute path to the SMM Xacro file.",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="smm",
                description="Gazebo entity name.",
            ),
            DeclareLaunchArgument(
                "data_dir",
                default_value=default_data_dir,
                description="Directory containing active_joint_names.yaml and generated runtime control files.",
            ),
            DeclareLaunchArgument(
                "controller_type",
                default_value="position",
                description="Controller type: position, velocity, effort, joint_pd_effort, pd_gravity, joint_inverse_dynamics.",
            ),
            DeclareLaunchArgument(
                "start_rqt_plot",
                default_value="true",
                description="Start rqt_plot automatically for N-DOF position error visualization.",
            ),
            DeclareLaunchArgument(
                "controller_defaults_yaml",
                default_value=os.path.join(
                    get_package_share_directory("smm_controllers"),
                    "config",
                    "controller_defaults.yaml",
                ),
                description="YAML file containing default parameters for SMM controllers.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

## How to run:
#ros2 launch smm_gazebo_sim spawn_smm_gazebo_control.launch.py \
#  controller_type:=joint_inverse_dynamics \
#  controller_defaults_yaml:=/path/to/my_experiment_params.yaml