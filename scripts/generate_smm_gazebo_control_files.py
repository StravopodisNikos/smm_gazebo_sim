#!/usr/bin/env python3

import argparse
import os
import yaml


VALID_CONTROLLER_TYPES = ["position", "velocity", "effort"]


def load_active_joint_names(active_joint_names_yaml):
    with open(active_joint_names_yaml, "r") as f:
        data = yaml.safe_load(f)

    if "active_joint_names" not in data:
        raise RuntimeError(
            f"Missing key 'active_joint_names' in {active_joint_names_yaml}"
        )

    joint_names = data["active_joint_names"]

    if not isinstance(joint_names, list) or len(joint_names) == 0:
        raise RuntimeError("'active_joint_names' must be a non-empty list.")

    return joint_names


def controller_plugin_type(controller_type):
    if controller_type == "position":
        return "position_controllers/JointGroupPositionController"

    if controller_type == "velocity":
        return "velocity_controllers/JointGroupVelocityController"

    if controller_type == "effort":
        return "effort_controllers/JointGroupEffortController"

    raise RuntimeError(f"Unsupported controller type: {controller_type}")


def command_interface_name(controller_type):
    if controller_type == "position":
        return "position"

    if controller_type == "velocity":
        return "velocity"

    if controller_type == "effort":
        return "effort"

    raise RuntimeError(f"Unsupported controller type: {controller_type}")


def write_controller_yaml(output_file, joint_names, controller_type, update_rate):
    command_interface = command_interface_name(controller_type)

    data = {
        "controller_manager": {
            "ros__parameters": {
                "update_rate": update_rate,
                "joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster"
                },
                "smm_joint_controller": {
                    "type": controller_plugin_type(controller_type)
                },
            }
        },
        "smm_joint_controller": {
            "ros__parameters": {
                "joints": joint_names,
                "command_interfaces": [command_interface],
            }
        },
    }

    with open(output_file, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def write_gz_ros2_control_xacro(output_file, joint_names, controller_yaml):
    joint_blocks = []

    for joint_name in joint_names:
        joint_blocks.append(
            f"""      <joint name="{joint_name}">
        <command_interface name="position"/>
        <command_interface name="velocity"/>
        <command_interface name="effort"/>

        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>"""
        )

    joint_blocks_text = "\n\n".join(joint_blocks)

    content = f"""<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="smm_generated_gz_ros2_control">

    <ros2_control name="SMMGazeboSystem" type="system">
      <hardware>
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
      </hardware>

{joint_blocks_text}

    </ros2_control>

    <gazebo>
      <plugin
        filename="libgz_ros2_control-system.so"
        name="gz_ros2_control::GazeboSimROS2ControlPlugin">
        <parameters>{controller_yaml}</parameters>
      </plugin>
    </gazebo>

  </xacro:macro>

</robot>
"""

    with open(output_file, "w") as f:
        f.write(content)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--active-joint-names-yaml",
        required=True,
        help="Input YAML containing active_joint_names.",
    )

    parser.add_argument(
        "--output-control-xacro",
        required=True,
        help="Output generated gz_ros2_control xacro file.",
    )

    parser.add_argument(
        "--output-controller-yaml",
        required=True,
        help="Output generated ros2_control controller YAML file.",
    )

    parser.add_argument(
        "--controller-type",
        default="position",
        choices=VALID_CONTROLLER_TYPES,
        help="Default joint controller type: position, velocity, or effort.",
    )

    parser.add_argument(
        "--update-rate",
        type=int,
        default=1000,
        help="controller_manager update rate in Hz.",
    )

    args = parser.parse_args()

    joint_names = load_active_joint_names(args.active_joint_names_yaml)

    os.makedirs(os.path.dirname(args.output_control_xacro), exist_ok=True)
    os.makedirs(os.path.dirname(args.output_controller_yaml), exist_ok=True)

    write_controller_yaml(
        output_file=args.output_controller_yaml,
        joint_names=joint_names,
        controller_type=args.controller_type,
        update_rate=args.update_rate,
    )

    write_gz_ros2_control_xacro(
        output_file=args.output_control_xacro,
        joint_names=joint_names,
        controller_yaml=args.output_controller_yaml,
    )

    print("[generate_smm_gazebo_control_files] Generated files:")
    print(f"  controller type: {args.controller_type}")
    print(f"  control xacro:   {args.output_control_xacro}")
    print(f"  controller yaml: {args.output_controller_yaml}")
    print("  joints:")
    for joint_name in joint_names:
        print(f"    - {joint_name}")


if __name__ == "__main__":
    main()

# How to test:
# python3 scripts/generate_smm_gazebo_control_files.py \
#--active-joint-names-yaml ~/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml/active_joint_names.yaml \
#--output-control-xacro ~/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml/generated_gz_ros2_control.xacro \
#--output-controller-yaml ~/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml/generated_smm_controllers.yaml \
#--controller-type position