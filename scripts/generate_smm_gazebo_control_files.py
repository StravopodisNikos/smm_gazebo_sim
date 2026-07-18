#!/usr/bin/env python3

import argparse
import os
import yaml

from smm_gazebo_helpers import (
    VALID_CONTROLLER_TYPES,
    load_active_joint_names,
    load_controller_defaults,
    controller_plugin_type,
    controller_runtime_name,
    command_interface_name,
    expand_to_dof,
    make_test_q_des,
)

def expand_optional_vector(cfg, key, n, default=None):
    value = cfg.get(key, default)

    if value is None:
        return None

    if isinstance(value, list) and len(value) == 0:
        return []

    return expand_to_dof(value, n, key)

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

    params["publish_desired_state"] = bool(cfg.get("publish_desired_state", True))

    if controller_type == "joint_pd_effort":
        params["publish_error_state"] = bool(cfg.get("publish_error_state", True))

        q_des_value = expand_optional_vector(cfg, "q_des", n, default=[])
        if q_des_value:
            params["q_des"] = q_des_value

        qdot_des_value = expand_optional_vector(cfg, "qdot_des", n, default=[])
        if qdot_des_value:
            params["qdot_des"] = qdot_des_value

    elif controller_type == "pd_gravity":
        params["dynamics_data_dir"] = cfg["dynamics_data_dir"]
        params["gravity_representation"] = cfg.get("gravity_representation", "body")
        params["publish_error_state"] = bool(cfg.get("publish_error_state", True))

        q_des_value = expand_optional_vector(cfg, "q_des", n, default=[])
        if q_des_value:
            params["q_des"] = q_des_value

        qdot_des_value = expand_optional_vector(cfg, "qdot_des", n, default=[])
        if qdot_des_value:
            params["qdot_des"] = qdot_des_value

    elif controller_type == "joint_inverse_dynamics":
        params["dynamics_data_dir"] = cfg["dynamics_data_dir"]
        params["dynamics_representation"] = cfg.get("dynamics_representation", "body")
        params["publish_error_state"] = bool(cfg.get("publish_error_state", True))
        params["trajectory_interpolation_mode"] = cfg.get("trajectory_interpolation_mode", "sync_cubic")
        
        q_des_value = expand_optional_vector(cfg, "q_des", n, default=None)
        if q_des_value is None:
            params["q_des"] = make_test_q_des(n, cfg)
        elif q_des_value:
            params["q_des"] = q_des_value

        qdot_des_value = expand_optional_vector(cfg, "qdot_des", n, default=None)
        if qdot_des_value is None:
            params["qdot_des"] = [float(cfg.get("qdot_des_default_value", 0.0))] * n
        elif qdot_des_value:
            params["qdot_des"] = qdot_des_value

        qddot_des_value = expand_optional_vector(cfg, "qddot_des", n, default=None)
        if qddot_des_value is None:
            params["qddot_des"] = [float(cfg.get("qddot_des_default_value", 0.0))] * n
        elif qddot_des_value:
            params["qddot_des"] = qddot_des_value

    elif controller_type == "cartesian_pd_gravity":
        params["kinematics_data_dir"] = cfg["kinematics_data_dir"]
        params["dynamics_data_dir"] = cfg["dynamics_data_dir"]
        params["gravity_representation"] = cfg.get("gravity_representation", "body")
        params["fixed_frame"] = cfg.get("fixed_frame", "world")

        params["publish_error_state"] = bool(cfg.get("publish_error_state", True))
        params["publish_desired_state"] = bool(cfg.get("publish_desired_state", True))
        params["publish_current_state"] = bool(cfg.get("publish_current_state", True))

        if "kp_cartesian" not in cfg:
            raise RuntimeError("cartesian_pd_gravity requires 'kp_cartesian'.")

        if "kd_cartesian" not in cfg:
            raise RuntimeError("cartesian_pd_gravity requires 'kd_cartesian'.")

        params["kp_cartesian"] = expand_to_dof(
            cfg["kp_cartesian"],
            3,
            "kp_cartesian",
        )

        params["kd_cartesian"] = expand_to_dof(
            cfg["kd_cartesian"],
            3,
            "kd_cartesian",
        )

        x_des_value = expand_optional_vector(cfg, "x_des", 3, default=[])

        if x_des_value is not None and len(x_des_value) > 0:
            params["x_des"] = x_des_value

        orientation_des_value = expand_optional_vector(cfg, "orientation_des", 4, default=[])

        if orientation_des_value is not None and len(orientation_des_value) > 0:
            params["orientation_des"] = orientation_des_value

    elif controller_type == "cartesian_pose_pd_gravity":
        params["kinematics_data_dir"] = cfg["kinematics_data_dir"]
        params["dynamics_data_dir"] = cfg["dynamics_data_dir"]
        params["gravity_representation"] = cfg.get("gravity_representation", "body")
        params["fixed_frame"] = cfg.get("fixed_frame", "world")

        params["publish_error_state"] = bool(cfg.get("publish_error_state", True))
        params["publish_desired_state"] = bool(cfg.get("publish_desired_state", True))
        params["publish_current_state"] = bool(cfg.get("publish_current_state", True))
        params["publish_full_debug_state"] = bool(cfg.get("publish_full_debug_state", True))

        params["kp_position"] = expand_to_dof(
            cfg["kp_position"],
            3,
            "kp_position",
        )

        params["kd_position"] = expand_to_dof(
            cfg["kd_position"],
            3,
            "kd_position",
        )

        params["kp_orientation"] = expand_to_dof(
            cfg["kp_orientation"],
            3,
            "kp_orientation",
        )

        params["kd_orientation"] = expand_to_dof(
            cfg["kd_orientation"],
            3,
            "kd_orientation",
        )

        x_des_value = expand_optional_vector(cfg, "x_des", 3, default=[])

        if x_des_value is not None and len(x_des_value) > 0:
            params["x_des"] = x_des_value

        orientation_des_value = expand_optional_vector(cfg, "orientation_des", 4, default=[])

        if orientation_des_value is not None and len(orientation_des_value) > 0:
            params["orientation_des"] = orientation_des_value

    elif controller_type == "cartesian_inv_dyn":
        params["kinematics_data_dir"] = cfg["kinematics_data_dir"]
        params["dynamics_data_dir"] = cfg["dynamics_data_dir"]
        params["gravity_representation"] = cfg.get("gravity_representation", "body")
        params["fixed_frame"] = cfg.get("fixed_frame", "world")

        params["publish_error_state"] = bool(cfg.get("publish_error_state", True))
        params["publish_desired_state"] = bool(cfg.get("publish_desired_state", True))
        params["publish_current_state"] = bool(cfg.get("publish_current_state", True))
        params["publish_full_debug_state"] = bool(cfg.get("publish_full_debug_state", True))

        params["hold_initial_position"] = bool(cfg.get("hold_initial_position", True))

        params["kp_position"] = expand_to_dof(cfg["kp_position"], 3, "kp_position")
        params["kd_position"] = expand_to_dof(cfg["kd_position"], 3, "kd_position")
        params["kp_orientation"] = expand_to_dof(cfg["kp_orientation"], 3, "kp_orientation")
        params["kd_orientation"] = expand_to_dof(cfg["kd_orientation"], 3, "kd_orientation")

        params["task_acceleration_scale"] = float(
            cfg.get("task_acceleration_scale", 1.0)
        )
        params["condition_soft_limit"] = float(
            cfg.get("condition_soft_limit", 500.0)
        )
        params["condition_hard_limit"] = float(
            cfg.get("condition_hard_limit", 2000.0)
        )
        params["orientation_condition_scaling"] = bool(
            cfg.get("orientation_condition_scaling", True)
        )

        params["operational_dynamics_method"] = cfg.get(
            "operational_dynamics_method",
            "exact_with_damped_fallback",
        )

        params["operational_damping"] = float(
            cfg.get("operational_damping", 0.001)
        )

        x_des_value = expand_optional_vector(cfg, "x_des", 3, default=[])
        if x_des_value is not None and len(x_des_value) > 0:
            params["x_des"] = x_des_value

        xdot_des_value = expand_optional_vector(cfg, "xdot_des", 3, default=[])
        if xdot_des_value is not None and len(xdot_des_value) > 0:
            params["xdot_des"] = xdot_des_value

        orientation_des_value = expand_optional_vector(
            cfg, "orientation_des", 4, default=[]
        )
        if orientation_des_value is not None and len(orientation_des_value) > 0:
            params["orientation_des"] = orientation_des_value

    elif controller_type == "cartesian_robust_inv_dyn":
        params["kinematics_data_dir"] = cfg["kinematics_data_dir"]
        params["dynamics_data_dir"] = cfg["dynamics_data_dir"]
        params["gravity_representation"] = cfg.get("gravity_representation", "body")
        params["fixed_frame"] = cfg.get("fixed_frame", "world")

        params["publish_error_state"] = bool(cfg.get("publish_error_state", True))
        params["publish_desired_state"] = bool(cfg.get("publish_desired_state", True))
        params["publish_current_state"] = bool(cfg.get("publish_current_state", True))
        params["publish_full_debug_state"] = bool(cfg.get("publish_full_debug_state", True))

        params["hold_initial_position"] = bool(cfg.get("hold_initial_position", True))

        params["kp_position"] = expand_to_dof(cfg["kp_position"], 3, "kp_position")
        params["kd_position"] = expand_to_dof(cfg["kd_position"], 3, "kd_position")

        params["kp_orientation"] = expand_to_dof(
            cfg["kp_orientation"],
            3,
            "kp_orientation",
        )

        params["kd_orientation"] = expand_to_dof(
            cfg["kd_orientation"],
            3,
            "kd_orientation",
        )

        params["lambda_position"] = expand_to_dof(
            cfg["lambda_position"],
            3,
            "lambda_position",
        )

        params["lambda_orientation"] = expand_to_dof(
            cfg["lambda_orientation"],
            3,
            "lambda_orientation",
        )

        params["k1_position"] = expand_to_dof(
            cfg["k1_position"],
            3,
            "k1_position",
        )

        params["k1_orientation"] = expand_to_dof(
            cfg["k1_orientation"],
            3,
            "k1_orientation",
        )

        params["k2_position"] = expand_to_dof(
            cfg["k2_position"],
            3,
            "k2_position",
        )

        params["k2_orientation"] = expand_to_dof(
            cfg["k2_orientation"],
            3,
            "k2_orientation",
        )

        params["tanh_kappa"] = float(cfg.get("tanh_kappa", 5.0))

        params["operational_dynamics_method"] = cfg.get(
            "operational_dynamics_method",
            "exact_with_damped_fallback",
        )

        params["operational_damping"] = float(
            cfg.get("operational_damping", 0.05)
        )

        params["task_acceleration_scale"] = float(
            cfg.get("task_acceleration_scale", 0.5)
        )

        params["condition_soft_limit"] = float(
            cfg.get("condition_soft_limit", 500.0)
        )

        params["condition_hard_limit"] = float(
            cfg.get("condition_hard_limit", 2000.0)
        )

        params["orientation_condition_scaling"] = bool(
            cfg.get("orientation_condition_scaling", True)
        )

        params["effort_limit"] = float(
            cfg.get("effort_limit", 80.0)
        )

        x_des_value = expand_optional_vector(cfg, "x_des", 3, default=[])
        if x_des_value is not None and len(x_des_value) > 0:
            params["x_des"] = x_des_value

        xdot_des_value = expand_optional_vector(cfg, "xdot_des", 3, default=[])
        if xdot_des_value is not None and len(xdot_des_value) > 0:
            params["xdot_des"] = xdot_des_value

        orientation_des_value = expand_optional_vector(
            cfg,
            "orientation_des",
            4,
            default=[],
        )

        if orientation_des_value is not None and len(orientation_des_value) > 0:
            params["orientation_des"] = orientation_des_value

    return params


def write_controller_yaml(
    output_file,
    joint_names,
    controller_type,
    update_rate,
    controller_defaults_yaml,
):
    command_interface = command_interface_name(controller_type)
    controller_name = controller_runtime_name(controller_type)
    defaults = load_controller_defaults(controller_defaults_yaml)

    if controller_type in [
        "joint_pd_effort",
        "pd_gravity",
        "joint_inverse_dynamics",
        "cartesian_pd_gravity",
        "cartesian_pose_pd_gravity",
        "cartesian_inv_dyn",
        "cartesian_robust_inv_dyn",
    ]:
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
                controller_name: {
                    "type": controller_plugin_type(controller_type)
                },
            }
        },
        controller_name: {
            "ros__parameters": controller_params
        },
    }

    with open(output_file, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)

    print("[write_controller_yaml] Controller YAML written:")
    print(f"  output_file:      {output_file}")
    print(f"  controller_type:  {controller_type}")
    print(f"  controller_name:  {controller_name}")
    print(f"  command_interface:{command_interface}")


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