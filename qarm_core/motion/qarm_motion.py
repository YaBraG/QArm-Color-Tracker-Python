"""High-level motion helpers for an already-open QArm-like object."""

import numpy as np

from qarm_core import config
from qarm_core.safety.qarm_safety import (
    check_joint_target,
    check_joint_vector,
    clamp_gripper,
    validate_delta,
    validate_joint_index,
)


class QArmMotionController:
    """Wrap a physical or virtual QArm object with safer motion helpers."""

    def __init__(self, qarm):
        self.qarm = qarm

    def read_pose(self) -> np.ndarray:
        """Read and return the current four arm joint positions."""

        self.qarm.read_std()
        return self.qarm.measJointPosition[:4].copy()

    def read_gripper(self) -> float:
        """Read and return the current gripper position."""

        self.qarm.read_std()
        return float(self.qarm.measJointPosition[4])

    def write_pose(
        self,
        joints,
        gripper=None,
        check_limits: bool = True,
    ) -> None:
        """Write a full four-joint pose and optional gripper command."""

        joints = np.asarray(joints, dtype=np.float64)

        if check_limits:
            check_joint_vector(joints)

        if gripper is None:
            gripper_value = float(config.GRIPPER_OPEN[0])
        else:
            gripper_value = clamp_gripper(float(gripper))

        self.qarm.write_position(
            phiCMD=joints,
            gprCMD=np.array([gripper_value], dtype=np.float64),
        )

    def move_joint_absolute(self, joint_index, target_rad, gripper=None) -> None:
        """Move one joint to an absolute target while preserving the others."""

        validate_joint_index(joint_index)
        check_joint_target(joint_index, float(target_rad))

        joints = self.read_pose()
        joints[joint_index] = float(target_rad)
        self.write_pose(joints, gripper=gripper)

    def move_joint_relative(
        self,
        joint_index,
        delta_rad,
        max_delta_rad: float = 0.05,
        gripper=None,
    ) -> None:
        """Move one joint by a small relative amount."""

        validate_joint_index(joint_index)
        validate_delta(float(delta_rad), max_delta_rad=max_delta_rad)

        joints = self.read_pose()
        target_rad = float(joints[joint_index]) + float(delta_rad)
        check_joint_target(joint_index, target_rad)

        joints[joint_index] = target_rad
        self.write_pose(joints, gripper=gripper)

    def go_home(self) -> None:
        """Move the arm to the shared home pose."""

        self.write_pose(config.HOME_POSE)

    def go_sleep(self) -> None:
        """Move the arm to the shared sleep pose."""

        self.write_pose(config.SLEEP_POSE)

    def open_gripper(self) -> None:
        """Open the gripper while keeping the current arm pose."""

        self.write_pose(self.read_pose(), gripper=float(config.GRIPPER_OPEN[0]))

    def close_gripper(self) -> None:
        """Close the gripper while keeping the current arm pose."""

        self.write_pose(self.read_pose(), gripper=float(config.GRIPPER_CLOSED[0]))

    def set_led(self, color) -> None:
        """Set the QArm base LED color."""

        self.qarm.write_led(np.array(color, dtype=np.float64))

    def set_led_green(self) -> None:
        self.set_led(config.LED_GREEN)

    def set_led_red(self) -> None:
        self.set_led(config.LED_RED)

    def set_led_blue(self) -> None:
        self.set_led(config.LED_BLUE)

    def set_led_off(self) -> None:
        self.set_led(config.LED_OFF)

