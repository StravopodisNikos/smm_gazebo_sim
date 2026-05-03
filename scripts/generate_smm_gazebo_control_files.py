#!/usr/bin/env python3

import argparse
import os
import yaml

from smm_gazebo_helpers import (
    VALID_CONTROLLER_TYPES,
    load_active_joint_names,
    load_controller_defaults,
    controller_plugin_type,
    command_interface_name,
    expand_to_dof,
    make_test_q_des,
)

def build_custom_controller_params(joint_names, controller_type, defaults):
    n = len(joint_names)

    if controller_type not in defaults:
        raise RuntimeError(
            f"Missing defaults for controller '{controller_type}' in controller defaults YAML."
        )

    cfg = defaults[controller_type]

    params = {
        "joints": joint_names,
    }

    if "kp" in cfg:
        params["kp"] = expand_to_dof(cfg["kp"], n, "kp")

    if "kd" in cfg:
        params["kd"] = expand_to_dof(cfg["kd"], n, "kd")

    if "hold_initial_position" in cfg:
        params["hold_initial_position"] = bool(cfg["hold_initial_position"])

    if controller_type == "pd_gravity":
        params["dynamics_data_dir"] = cfg["dynamics_data_dir"]
        params["gravity_representation"] = cfg.get("gravity_representation", "body")

    elif controller_type == "joint_inverse_dynamics":
        params["dynamics_data_dir"] = cfg["dynamics_data_dir"]
        params["dynamics_representation"] = cfg.get("dynamics_representation", "body")
        params["publish_error_state"] = bool(cfg.get("publish_error_state", True))

        if "q_des" in cfg:
            params["q_des"] = expand_to_dof(cfg["q_des"], n, "q_des")
        else:
            params["q_des"] = make_test_q_des(n, cfg)

        if "qdot_des" in cfg:
            params["qdot_des"] = expand_to_dof(cfg["qdot_des"], n, "qdot_des")
        else:
            params["qdot_des"] = [float(cfg.get("qdot_des_default_value", 0.0))] * n

        if "qddot_des" in cfg:
            params["qddot_des"] = expand_to_dof(cfg["qddot_des"], n, "qddot_des")
        else:
            params["qddot_des"] = [float(cfg.get("qddot_des_default_value", 0.0))] * n

    return params


def write_controller_yaml(
    output_file,
    joint_names,
    controller_type,
    update_rate,
    controller_defaults_yaml,
):
    command_interface = command_interface_name(controller_type)
    defaults = load_controller_defaults(controller_defaults_yaml)

    if controller_type in ["joint_pd_effort", "pd_gravity", "joint_inverse_dynamics"]:
        controller_params = build_custom_controller_params(
            joint_names=joint_names,
            controller_type=controller_type,
            defaults=defaults,
        )
    else:
        controller_params = {
            "joints": joint_names,
            "command_interfaces": [command_interface],
        }

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
            "ros__parameters": controller_params
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
        help="Controller type.",
    )

    parser.add_argument(
        "--update-rate",
        type=int,
        default=1000,
        help="controller_manager update rate in Hz.",
    )

    parser.add_argument(
        "--controller-defaults-yaml",
        required=True,
        help="YAML file containing default parameters for SMM controllers.",
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
        controller_defaults_yaml=os.path.expanduser(args.controller_defaults_yaml),
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
    print(f"  defaults yaml:   {args.controller_defaults_yaml}")
    print("  joints:")
    for joint_name in joint_names:
        print(f"    - {joint_name}")


if __name__ == "__main__":
    main()