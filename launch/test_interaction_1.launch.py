from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration

# Simple tsst for robust impedance with apply wrench only

def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    model = LaunchConfiguration("model").perform(context)
    link = LaunchConfiguration("link").perform(context)

    fixed_frame = LaunchConfiguration("fixed_frame").perform(context)
    external_wrench_topic = LaunchConfiguration("external_wrench_topic").perform(context)
    rate = LaunchConfiguration("rate").perform(context)

    force_x = LaunchConfiguration("force_x").perform(context)
    force_y = LaunchConfiguration("force_y").perform(context)
    force_z = LaunchConfiguration("force_z").perform(context)

    torque_x = LaunchConfiguration("torque_x").perform(context)
    torque_y = LaunchConfiguration("torque_y").perform(context)
    torque_z = LaunchConfiguration("torque_z").perform(context)

    scoped_link = f"{model}::{link}"

    gz_wrench_payload = (
        f'entity {{name: "{scoped_link}" type: LINK}} '
        f'wrench {{'
        f'force {{x: {force_x} y: {force_y} z: {force_z}}} '
        f'torque {{x: {torque_x} y: {torque_y} z: {torque_z}}}'
        f'}}'
    )

    gz_clear_payload = (
        f'name: "{scoped_link}" type: LINK'
    )

    ros_wrench_msg = (
        f"header:\n"
        f"  frame_id: {fixed_frame}\n"
        f"wrench:\n"
        f"  force:\n"
        f"    x: {force_x}\n"
        f"    y: {force_y}\n"
        f"    z: {force_z}\n"
        f"  torque:\n"
        f"    x: {torque_x}\n"
        f"    y: {torque_y}\n"
        f"    z: {torque_z}\n"
    )

    ros_zero_wrench_msg = (
        f"header:\n"
        f"  frame_id: {fixed_frame}\n"
        f"wrench:\n"
        f"  force:\n"
        f"    x: 0.0\n"
        f"    y: 0.0\n"
        f"    z: 0.0\n"
        f"  torque:\n"
        f"    x: 0.0\n"
        f"    y: 0.0\n"
        f"    z: 0.0\n"
    )

    print("[test_interaction_1] Applying Gazebo persistent wrench:")
    print(f"  world: {world}")
    print(f"  link:  {scoped_link}")
    print(f"  force: [{force_x}, {force_y}, {force_z}] N")
    print(f"  torque:[{torque_x}, {torque_y}, {torque_z}] Nm")

    print("[test_interaction_1] Publishing equivalent controller wrench:")
    print(f"  topic: {external_wrench_topic}")
    print(f"  frame: {fixed_frame}")

    apply_gz_wrench = ExecuteProcess(
        cmd=[
            "gz",
            "topic",
            "-t",
            f"/world/{world}/wrench/persistent",
            "-m",
            "gz.msgs.EntityWrench",
            "-p",
            gz_wrench_payload,
        ],
        output="screen",
    )

    publish_external_wrench = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "-r",
            rate,
            external_wrench_topic,
            "geometry_msgs/msg/WrenchStamped",
            ros_wrench_msg,
        ],
        output="screen",
    )

    clear_gz_wrench = ExecuteProcess(
        cmd=[
            "gz",
            "topic",
            "-t",
            f"/world/{world}/wrench/clear",
            "-m",
            "gz.msgs.Entity",
            "-p",
            gz_clear_payload,
        ],
        output="screen",
    )

    publish_zero_external_wrench = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            external_wrench_topic,
            "geometry_msgs/msg/WrenchStamped",
            ros_zero_wrench_msg,
        ],
        output="screen",
    )

    cleanup_on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                clear_gz_wrench,
                publish_zero_external_wrench,
            ]
        )
    )

    return [
        apply_gz_wrench,
        publish_external_wrench,
        cleanup_on_shutdown,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="smm_world",
                description="Gazebo world name.",
            ),

            DeclareLaunchArgument(
                "model",
                default_value="smm",
                description="Gazebo model name.",
            ),

            DeclareLaunchArgument(
                "link",
                # default is for 3Dof case
                # For 6DoF, first inspect: gz model -m smm --link
                default_value="active_module_b_2",
                description=(
                    "Gazebo exposed physical link where the wrench is applied. "
                    "For the current 3DoF model this is active_module_b_2. "
                    "For other DoF/anatomies, run `gz model -m smm --link` and pass "
                    "the terminal exposed physical link explicitly."
                ),
            ),

            DeclareLaunchArgument(
                "fixed_frame",
                default_value="world",
                description="Frame used by the controller external wrench message.",
            ),

            DeclareLaunchArgument(
                "external_wrench_topic",
                default_value="/smm_cartesian_controller/external_wrench",
                description="Robust impedance controller external wrench topic.",
            ),

            DeclareLaunchArgument(
                "rate",
                default_value="100",
                description="External wrench publishing rate in Hz.",
            ),

            DeclareLaunchArgument(
                "force_x",
                default_value="5.0",
                description="Force X in fixed/world frame, N.",
            ),

            DeclareLaunchArgument(
                "force_y",
                default_value="0.0",
                description="Force Y in fixed/world frame, N.",
            ),

            DeclareLaunchArgument(
                "force_z",
                default_value="0.0",
                description="Force Z in fixed/world frame, N.",
            ),

            DeclareLaunchArgument(
                "torque_x",
                default_value="0.0",
                description="Torque X in fixed/world frame, Nm.",
            ),

            DeclareLaunchArgument(
                "torque_y",
                default_value="0.0",
                description="Torque Y in fixed/world frame, Nm.",
            ),

            DeclareLaunchArgument(
                "torque_z",
                default_value="0.0",
                description="Torque Z in fixed/world frame, Nm.",
            ),

            OpaqueFunction(function=launch_setup),
        ]
    )