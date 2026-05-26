"""Numeric safety helpers for QArm joint commands.

These functions do not talk to hardware. They only validate command values
before another part of the program sends those values to a physical or virtual
QArm.
"""

import numpy as np


JOINT_NAMES = ["base", "shoulder", "elbow", "wrist"]

LIMITS_MIN = np.array(
    [
        -17 * np.pi / 18,
        -17 * np.pi / 36,
        -19 * np.pi / 36,
        -8 * np.pi / 9,
    ],
    dtype=np.float64,
)

LIMITS_MAX = np.array(
    [
        17 * np.pi / 18,
        17 * np.pi / 36,
        15 * np.pi / 36,
        8 * np.pi / 9,
    ],
    dtype=np.float64,
)


def validate_joint_index(joint_index: int) -> None:
    """Make sure code is commanding one of the four arm joints."""

    # Protects against typos like joint 4 or "base" being sent as a command.
    if not isinstance(joint_index, int):
        raise TypeError("joint_index must be an integer from 0 to 3.")

    if joint_index < 0 or joint_index >= len(JOINT_NAMES):
        raise ValueError("joint_index must be between 0 and 3.")


def validate_delta(delta_rad: float, max_delta_rad: float = 0.05) -> None:
    """Limit relative moves so one command cannot jump too far at once."""

    # Protects against NaN/infinity and accidental large relative moves.
    if not np.isfinite(delta_rad):
        raise ValueError("delta_rad must be a finite number.")

    if max_delta_rad <= 0 or not np.isfinite(max_delta_rad):
        raise ValueError("max_delta_rad must be a positive finite number.")

    if abs(delta_rad) > max_delta_rad:
        raise ValueError(
            f"delta_rad {delta_rad:.4f} is larger than the allowed "
            f"step of +/-{max_delta_rad:.4f} rad."
        )


def check_joint_target(joint_index: int, target_rad: float) -> None:
    """Check that one joint target is inside its allowed travel range."""

    validate_joint_index(joint_index)

    # Protects the robot from undefined numeric targets.
    if not np.isfinite(target_rad):
        joint_name = JOINT_NAMES[joint_index]
        raise ValueError(f"{joint_name} target must be a finite number.")

    min_rad = LIMITS_MIN[joint_index]
    max_rad = LIMITS_MAX[joint_index]

    if target_rad < min_rad or target_rad > max_rad:
        joint_name = JOINT_NAMES[joint_index]
        raise ValueError(
            f"{joint_name} target {target_rad:.4f} rad is outside the "
            f"allowed range [{min_rad:.4f}, {max_rad:.4f}] rad."
        )


def check_joint_vector(joints: np.ndarray) -> None:
    """Check a full four-joint pose before it is sent to the arm."""

    joints = np.asarray(joints, dtype=np.float64)

    # Protects against sending too many, too few, or nested joint values.
    if joints.shape != (4,):
        raise ValueError("joints must be a numpy array or sequence with 4 values.")

    ensure_finite_array(joints, "joints")

    for joint_index, target_rad in enumerate(joints):
        check_joint_target(joint_index, float(target_rad))


def is_valid_joint_vector(joints: np.ndarray) -> bool:
    """Return True when a four-joint pose passes all safety checks."""

    try:
        check_joint_vector(joints)
    except (TypeError, ValueError):
        return False

    return True


def clamp_gripper(gripper_value: float) -> float:
    """Clamp gripper commands to the same safe range used by qarm.py."""

    # The gripper is allowed to clamp because qarm.py already uses this range.
    if not np.isfinite(gripper_value):
        raise ValueError("gripper_value must be a finite number.")

    return float(np.clip(gripper_value, 0.1, 0.9))


def ensure_finite_array(values: np.ndarray, name: str = "array") -> None:
    """Reject NaN or infinite values before they reach robot commands."""

    values = np.asarray(values)

    # Protects against invalid math results being forwarded to hardware.
    if not np.all(np.isfinite(values)):
        raise ValueError(f"{name} must contain only finite numbers.")
