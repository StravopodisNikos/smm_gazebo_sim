#!/usr/bin/env python3

import os
import yaml


VALID_CONTROLLER_TYPES = [
    "position",
    "velocity",
    "effort",
    "joint_pd_effort",
    "pd_gravity",
    "joint_inverse_dynamics",
    "cartesian_pd_gravity",
    "cartesian_pose_pd_gravity",
    "cartesian_inv_dyn",
    "cartesian_robust_inv_dyn",
]


def load_yaml(path):
    path = os.path.expanduser(path)

    if not os.path.exists(path):
        raise RuntimeError(f"YAML file not found: {path}")

    with open(path, "r") as f:
        data = yaml.safe_load(f)

    return data if data is not None else {}


def ensure_directory(path):
    path = os.path.expanduser(path)
    os.makedirs(path, exist_ok=True)
    return path


def validate_controller_type(controller_type):
    if controller_type not in VALID_CONTROLLER_TYPES:
        raise RuntimeError(
            f"Invalid controller_type='{controller_type}'. "
            f"Allowed values: {', '.join(VALID_CONTROLLER_TYPES)}."
        )


def load_active_joint_names(active_joint_names_yaml):
    data = load_yaml(active_joint_names_yaml)

    if "active_joint_names" not in data:
        raise RuntimeError(
            f"Missing key 'active_joint_names' in {active_joint_names_yaml}"
        )

    joint_names = data["active_joint_names"]

    if not isinstance(joint_names, list) or len(joint_names) == 0:
        raise RuntimeError("'active_joint_names' must be a non-empty list.")

    return joint_names


def load_controller_defaults(defaults_yaml):
    data = load_yaml(defaults_yaml)

    if "controllers" not in data:
        raise RuntimeError(
            f"Missing top-level key 'controllers' in {defaults_yaml}"
        )

    return data["controllers"]


def controller_plugin_type(controller_type):
    plugin_map = {
        "position": "position_controllers/JointGroupPositionController",
        "velocity": "velocity_controllers/JointGroupVelocityController",
        "effort": "effort_controllers/JointGroupEffortController",
        "joint_pd_effort": "smm_controllers/JointPDEffortController",
        "pd_gravity": "smm_controllers/PDGravityController",
        "joint_inverse_dynamics": "smm_controllers/InverseDynamicsJointController",
        "cartesian_pd_gravity": "smm_controllers/CartesianPDGravityController",
        "cartesian_pose_pd_gravity": "smm_controllers/CartesianPosePDGravityController",
        "cartesian_inv_dyn": "smm_controllers/CartesianInvDynController",
        "cartesian_robust_inv_dyn": "smm_controllers/CartesianRobustInvDynController",
    }

    validate_controller_type(controller_type)
    return plugin_map[controller_type]

def controller_runtime_name(controller_type):
    if controller_type.startswith("cartesian_"):
        return "smm_cartesian_controller"

    return "smm_joint_controller"

def command_interface_name(controller_type):
    if controller_type == "position":
        return "position"

    if controller_type == "velocity":
        return "velocity"

    if controller_type in [
        "effort",
        "joint_pd_effort",
        "pd_gravity",
        "joint_inverse_dynamics",
        "cartesian_pd_gravity",
        "cartesian_pose_pd_gravity",
        "cartesian_inv_dyn",
        "cartesian_robust_inv_dyn",
    ]:
        return "effort"

    raise RuntimeError(f"Unsupported controller type: {controller_type}")


def expand_to_dof(value, n, field_name):
    if isinstance(value, list):
        if len(value) != n:
            raise RuntimeError(
                f"Parameter '{field_name}' must have length {n}, got {len(value)}."
            )
        return [float(v) for v in value]

    return [float(value)] * n


def make_zero_vector(n):
    return [0.0] * n


def make_test_q_des(n, cfg):
    default_value = float(cfg.get("q_des_default_value", 0.0))
    joint_index = int(cfg.get("q_des_test_joint_index", -1))
    joint_value = float(cfg.get("q_des_test_joint_value", default_value))

    q_des = [default_value] * n

    if 0 <= joint_index < n:
        q_des[joint_index] = joint_value

    return q_des


def build_rqt_position_error_topics(dof, controller_name="smm_joint_controller"):
    return [
        f"/{controller_name}/q_error_{i}/data"
        for i in range(dof)
    ]


def runtime_control_paths(data_dir):
    data_dir = ensure_directory(data_dir)

    return {
        "active_joint_names_yaml": os.path.join(data_dir, "active_joint_names.yaml"),
        "generated_control_xacro": os.path.join(data_dir, "generated_gz_ros2_control.xacro"),
        "generated_controller_yaml": os.path.join(data_dir, "generated_smm_controllers.yaml"),
    }


def require_file(path, hint=None):
    path = os.path.expanduser(path)

    if not os.path.exists(path):
        msg = f"Required file not found:\n  {path}"
        if hint:
            msg += f"\n{hint}"
        raise RuntimeError(msg)

    return path