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

def expand_joint_limit_vector(value, n, key):
    """
    Expands a joint-limit parameter to length n.

    Accepted forms:
      [] or None      -> omit the key
      scalar          -> [scalar] * n
      [scalar]        -> [scalar] * n
      [v1, ..., vn]   -> unchanged after validation

    This is mainly used for velocity limits.
    """    
    if value is None:
        return None

    if isinstance(value, list):
        if len(value) == 0:
            return None

        if len(value) == 1:
            return [float(value[0])] * n

        return expand_to_dof(value, n, key)

    return [float(value)] * n

def expand_first_rest_vector(value, n, key):
    """
    Expands a joint-limit parameter to length n.

    Accepted forms:
      [] or None        -> omit the key
      scalar            -> [scalar] * n
      [scalar]          -> [scalar] * n
      [first, rest]     -> [first, rest, rest, ..., rest]
      [v1, ..., vn]     -> unchanged after validation

    This is mainly used for effort limits:
      [80.0, 55.0] -> [80.0, 55.0, 55.0, ...]
    """    
    if value is None:
        return None

    if isinstance(value, list):
        if len(value) == 0:
            return None

        if len(value) == 1:
            return [float(value[0])] * n

        if len(value) == 2:
            return [float(value[0])] + [float(value[1])] * (n - 1)

        return expand_to_dof(value, n, key)

    return [float(value)] * n

def copy_if_present(params, cfg, key):
    """
    Copies a config key only when it exists and is not an empty list.

    This prevents invalid YAML entries such as:
      x_des:
      orientation_des:
      joint_velocity_limits:
    """
    if key not in cfg:
        return

    value = cfg[key]

    if isinstance(value, list) and len(value) == 0:
        return

    if value is None:
        return

    params[key] = value

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

        params["effort_limit"] = float(
            cfg.get("effort_limit", 80.0)
        )

        joint_effort_limits_value = expand_first_rest_vector(
            cfg.get("joint_effort_limits", None),
            n,
            "joint_effort_limits",
        )

        if joint_effort_limits_value is not None:
            params["joint_effort_limits"] = joint_effort_limits_value

        params["operational_dynamics_method"] = cfg.get(
            "operational_dynamics_method",
            "exact_with_damped_fallback",
        )

        params["operational_damping"] = float(
            cfg.get("operational_damping", 0.001)
        )

        params["enforce_velocity_limits"] = bool(
            cfg.get("enforce_velocity_limits", True)
        )
        params["default_velocity_limit"] = float(
            cfg.get("default_velocity_limit", 4.0841)
        )
        params["velocity_soft_margin"] = float(
            cfg.get("velocity_soft_margin", 0.25)
        )
        params["velocity_brake_gain"] = float(
            cfg.get("velocity_brake_gain", 15.0)
        )

        joint_velocity_limits_value = expand_joint_limit_vector(
            cfg.get("joint_velocity_limits", None),
            n,
            "joint_velocity_limits",
        )

        if joint_velocity_limits_value is not None:
            params["joint_velocity_limits"] = joint_velocity_limits_value

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

        joint_effort_limits_value = expand_first_rest_vector(
            cfg.get("joint_effort_limits", None),
            n,
            "joint_effort_limits",
        )

        if joint_effort_limits_value is not None:
            params["joint_effort_limits"] = joint_effort_limits_value

        params["enforce_velocity_limits"] = bool(
            cfg.get("enforce_velocity_limits", True)
        )
        params["default_velocity_limit"] = float(
            cfg.get("default_velocity_limit", 4.0841)
        )
        params["velocity_soft_margin"] = float(
            cfg.get("velocity_soft_margin", 0.25)
        )
        params["velocity_brake_gain"] = float(
            cfg.get("velocity_brake_gain", 15.0)
        )

        joint_velocity_limits_value = expand_joint_limit_vector(
            cfg.get("joint_velocity_limits", None),
            n,
            "joint_velocity_limits",
        )

        if joint_velocity_limits_value is not None:
            params["joint_velocity_limits"] = joint_velocity_limits_value

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

    elif controller_type == "cartesian_robust_adaptive_inv_dyn":
        n = len(joint_names)

        params = {
            "joints": joint_names,

            # ---------------------------------------------------------------
            # Operational acceleration reference:
            # a_ref = scale * (Kp*e + Kd*e_dot)
            # ---------------------------------------------------------------
            "kp_position": expand_to_dof(
                cfg["kp_position"],
                3,
                "kp_position",
            ),
            "kd_position": expand_to_dof(
                cfg["kd_position"],
                3,
                "kd_position",
            ),
            "kp_orientation": expand_to_dof(
                cfg["kp_orientation"],
                3,
                "kp_orientation",
            ),
            "kd_orientation": expand_to_dof(
                cfg["kd_orientation"],
                3,
                "kd_orientation",
            ),

            # ---------------------------------------------------------------
            # Robust/adaptive sliding variable:
            # s = e_dot + Lambda*e
            # ---------------------------------------------------------------
            "lambda_position": expand_to_dof(
                cfg["lambda_position"],
                3,
                "lambda_position",
            ),
            "lambda_orientation": expand_to_dof(
                cfg["lambda_orientation"],
                3,
                "lambda_orientation",
            ),

            # ---------------------------------------------------------------
            # Fixed robust RIDOSC term:
            # F_rob = K1*s + K2*tanh(kappa*s)
            # ---------------------------------------------------------------
            "k1_position": expand_to_dof(
                cfg["k1_position"],
                3,
                "k1_position",
            ),
            "k1_orientation": expand_to_dof(
                cfg["k1_orientation"],
                3,
                "k1_orientation",
            ),
            "k2_position": expand_to_dof(
                cfg["k2_position"],
                3,
                "k2_position",
            ),
            "k2_orientation": expand_to_dof(
                cfg["k2_orientation"],
                3,
                "k2_orientation",
            ),
            "tanh_kappa": float(cfg.get("tanh_kappa", 5.0)),

            # ---------------------------------------------------------------
            # Adaptive robust term:
            # rho_hat_dot = gamma*max(|s|-deadzone,0) - leakage*rho_hat
            # F_ad = rho_hat*tanh(kappa*s)
            # ---------------------------------------------------------------
            "adaptive_enabled": bool(cfg.get("adaptive_enabled", False)),
            "adaptive_gain_position": expand_to_dof(
                cfg["adaptive_gain_position"],
                3,
                "adaptive_gain_position",
            ),
            "adaptive_gain_orientation": expand_to_dof(
                cfg["adaptive_gain_orientation"],
                3,
                "adaptive_gain_orientation",
            ),
            "adaptive_leakage_position": expand_to_dof(
                cfg["adaptive_leakage_position"],
                3,
                "adaptive_leakage_position",
            ),
            "adaptive_leakage_orientation": expand_to_dof(
                cfg["adaptive_leakage_orientation"],
                3,
                "adaptive_leakage_orientation",
            ),
            "adaptive_rho_initial_position": expand_to_dof(
                cfg["adaptive_rho_initial_position"],
                3,
                "adaptive_rho_initial_position",
            ),
            "adaptive_rho_initial_orientation": expand_to_dof(
                cfg["adaptive_rho_initial_orientation"],
                3,
                "adaptive_rho_initial_orientation",
            ),
            "adaptive_rho_min_position": expand_to_dof(
                cfg["adaptive_rho_min_position"],
                3,
                "adaptive_rho_min_position",
            ),
            "adaptive_rho_min_orientation": expand_to_dof(
                cfg["adaptive_rho_min_orientation"],
                3,
                "adaptive_rho_min_orientation",
            ),
            "adaptive_rho_max_position": expand_to_dof(
                cfg["adaptive_rho_max_position"],
                3,
                "adaptive_rho_max_position",
            ),
            "adaptive_rho_max_orientation": expand_to_dof(
                cfg["adaptive_rho_max_orientation"],
                3,
                "adaptive_rho_max_orientation",
            ),
            "adaptive_deadzone": float(cfg.get("adaptive_deadzone", 0.001)),

            # ---------------------------------------------------------------
            # Reference handling
            # ---------------------------------------------------------------
            "hold_initial_position": bool(cfg.get("hold_initial_position", True)),

            # ---------------------------------------------------------------
            # Model data paths
            # These are loaded from controller_defaults.yaml.
            # Do not use data_dir here because build_custom_controller_params()
            # does not receive data_dir.
            # ---------------------------------------------------------------
            "kinematics_data_dir": cfg["kinematics_data_dir"],
            "dynamics_data_dir": cfg["dynamics_data_dir"],
            "gravity_representation": cfg.get("gravity_representation", "body"),
            "fixed_frame": cfg.get("fixed_frame", "world"),

            # ---------------------------------------------------------------
            # Operational-space dynamics
            # ---------------------------------------------------------------
            "operational_dynamics_method": cfg.get(
                "operational_dynamics_method",
                "exact_with_damped_fallback",
            ),
            "operational_damping": float(cfg.get("operational_damping", 0.05)),
            "task_acceleration_scale": float(
                cfg.get("task_acceleration_scale", 1.0)
            ),

            # ---------------------------------------------------------------
            # Jacobian conditioning safety
            # ---------------------------------------------------------------
            "condition_soft_limit": float(cfg.get("condition_soft_limit", 500.0)),
            "condition_hard_limit": float(cfg.get("condition_hard_limit", 2000.0)),
            "orientation_condition_scaling": bool(
                cfg.get("orientation_condition_scaling", False)
            ),

            # ---------------------------------------------------------------
            # Effort and velocity safety
            # ---------------------------------------------------------------
            "effort_limit": float(cfg.get("effort_limit", 80.0)),
            "enforce_velocity_limits": bool(
                cfg.get("enforce_velocity_limits", True)
            ),
            "default_velocity_limit": float(
                cfg.get("default_velocity_limit", 4.0841)
            ),
            "velocity_soft_margin": float(
                cfg.get("velocity_soft_margin", 0.25)
            ),
            "velocity_brake_gain": float(
                cfg.get("velocity_brake_gain", 15.0)
            ),

            # ---------------------------------------------------------------
            # Debug publishing
            # ---------------------------------------------------------------
            "publish_error_state": bool(cfg.get("publish_error_state", True)),
            "publish_desired_state": bool(cfg.get("publish_desired_state", True)),
            "publish_current_state": bool(cfg.get("publish_current_state", True)),
            "publish_full_debug_state": bool(
                cfg.get("publish_full_debug_state", True)
            ),
        }

        # Optional desired references.
        # Empty lists are intentionally omitted to avoid invalid ROS 2 parameter YAML.
        copy_if_present(params, cfg, "x_des")
        copy_if_present(params, cfg, "xdot_des")
        copy_if_present(params, cfg, "orientation_des")

        # Per-joint effort limits.
        # Example:
        #   [80.0, 55.0] -> [80.0, 55.0, 55.0, ...]
        joint_effort_limits_value = expand_first_rest_vector(
            cfg.get("joint_effort_limits", None),
            n,
            "joint_effort_limits",
        )

        if joint_effort_limits_value is not None:
            params["joint_effort_limits"] = joint_effort_limits_value

        # Per-joint velocity limits.
        # Example:
        #   [4.0841] -> [4.0841, 4.0841, ...]
        joint_velocity_limits_value = expand_joint_limit_vector(
            cfg.get("joint_velocity_limits", None),
            n,
            "joint_velocity_limits",
        )

        if joint_velocity_limits_value is not None:
            params["joint_velocity_limits"] = joint_velocity_limits_value

    # Add impedance
    elif controller_type == "cartesian_robust_impedance":
        # Robust Impedance Controller, RIC.
        # It inherits most RIDOSC infrastructure but replaces the acceleration
        # reference with:
        #
        #   a_imp = M_imp^{-1}(D_imp * e_dot + K_imp * e)
        #
        # External TCP wrench is not included yet in this first version.

        params["impedance_mass_position"] = expand_to_dof(
            cfg["impedance_mass_position"],
            3,
            "impedance_mass_position",
        )

        params["impedance_damping_position"] = expand_to_dof(
            cfg["impedance_damping_position"],
            3,
            "impedance_damping_position",
        )

        params["impedance_stiffness_position"] = expand_to_dof(
            cfg["impedance_stiffness_position"],
            3,
            "impedance_stiffness_position",
        )

        params["impedance_mass_orientation"] = expand_to_dof(
            cfg["impedance_mass_orientation"],
            3,
            "impedance_mass_orientation",
        )

        params["impedance_damping_orientation"] = expand_to_dof(
            cfg["impedance_damping_orientation"],
            3,
            "impedance_damping_orientation",
        )

        params["impedance_stiffness_orientation"] = expand_to_dof(
            cfg["impedance_stiffness_orientation"],
            3,
            "impedance_stiffness_orientation",
        )

        # Compatibility with copied RIDOSC parameter parser.
        # RIC does not use kp/kd in the command law anymore, but the current
        # C++ on_configure() still reads and validates them.
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

        params["tanh_kappa"] = float(cfg["tanh_kappa"])

        params["hold_initial_position"] = bool(cfg["hold_initial_position"])

        params["use_external_wrench"] = bool(
            cfg.get("use_external_wrench", True)
        )

        params["subtract_external_wrench_from_command"] = bool(
            cfg.get("subtract_external_wrench_from_command", True)
        )

        params["require_external_wrench_frame_match"] = bool(
            cfg.get("require_external_wrench_frame_match", True)
        )

        params["external_wrench_sign"] = float(
            cfg.get("external_wrench_sign", 1.0)
        )

        params["external_wrench_timeout"] = float(
            cfg.get("external_wrench_timeout", 0.2)
        )

        params["external_wrench_filter_alpha"] = float(
            cfg.get("external_wrench_filter_alpha", 0.2)
        )

        params["external_wrench_deadband_force"] = float(
            cfg.get("external_wrench_deadband_force", 0.2)
        )

        params["external_wrench_deadband_torque"] = float(
            cfg.get("external_wrench_deadband_torque", 0.02)
        )

        params["external_wrench_limit_force"] = float(
            cfg.get("external_wrench_limit_force", 80.0)
        )

        params["external_wrench_limit_torque"] = float(
            cfg.get("external_wrench_limit_torque", 10.0)
        )

        params["desired_wrench_position"] = expand_to_dof(
            cfg["desired_wrench_position"],
            3,
            "desired_wrench_position",
        )

        params["desired_wrench_orientation"] = expand_to_dof(
            cfg["desired_wrench_orientation"],
            3,
            "desired_wrench_orientation",
        )

        # Optional desired Cartesian references.
        # Do not emit null YAML fields.
        copy_if_present(params, cfg, "x_des")
        copy_if_present(params, cfg, "xdot_des")
        copy_if_present(params, cfg, "orientation_des")

        params["kinematics_data_dir"] = cfg["kinematics_data_dir"]
        params["dynamics_data_dir"] = cfg["dynamics_data_dir"]

        params["gravity_representation"] = cfg["gravity_representation"]
        params["fixed_frame"] = cfg["fixed_frame"]

        params["operational_dynamics_method"] = cfg["operational_dynamics_method"]
        params["operational_damping"] = float(cfg["operational_damping"])

        params["task_acceleration_scale"] = float(cfg["task_acceleration_scale"])

        params["condition_soft_limit"] = float(cfg["condition_soft_limit"])
        params["condition_hard_limit"] = float(cfg["condition_hard_limit"])
        params["orientation_condition_scaling"] = bool(
            cfg["orientation_condition_scaling"])

        params["effort_limit"] = float(cfg["effort_limit"])

        joint_effort_limits = expand_first_rest_vector(
            cfg.get("joint_effort_limits"),
            n,
            "joint_effort_limits")
        if joint_effort_limits is not None:
            params["joint_effort_limits"] = joint_effort_limits

        params["enforce_velocity_limits"] = bool(
            cfg.get("enforce_velocity_limits", True))

        params["default_velocity_limit"] = float(
            cfg.get("default_velocity_limit", 4.0841))

        params["velocity_soft_margin"] = float(
            cfg.get("velocity_soft_margin", 0.25))

        params["velocity_brake_gain"] = float(
            cfg.get("velocity_brake_gain", 15.0))

        joint_velocity_limits = expand_joint_limit_vector(
            cfg.get("joint_velocity_limits"),
            n,
            "joint_velocity_limits")
        if joint_velocity_limits is not None:
            params["joint_velocity_limits"] = joint_velocity_limits

        params["publish_error_state"] = bool(cfg["publish_error_state"])
        params["publish_desired_state"] = bool(cfg["publish_desired_state"])
        params["publish_current_state"] = bool(cfg["publish_current_state"])
        params["publish_full_debug_state"] = bool(cfg["publish_full_debug_state"])

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
        "cartesian_robust_adaptive_inv_dyn",
        "cartesian_robust_impedance",
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