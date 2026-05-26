import time
import numpy as np
from quanser.hardware import HIL, HILError, MAX_STRING_LENGTH


class QArm:
    """
    Minimal low-level QArm controller using quanser.hardware.HIL directly.

    Joint channel map:
        1000 = base / joint 0
        1001 = shoulder / joint 1
        1002 = elbow / joint 2
        1003 = wrist / joint 3
        1004 = gripper

    Units:
        Joint commands are in radians.
    """

    JOINT_CHANNELS = np.array([1000, 1001, 1002, 1003], dtype=np.int32)
    ALL_POSITION_CHANNELS = np.array([1000, 1001, 1002, 1003, 1004], dtype=np.int32)

    def __init__(self, device_id="0"):
        self.device_id = device_id
        self.card = HIL()
        self.is_open = False

    def open(self):
        """
        Open QArm hardware and configure joints in position mode.
        """
        try:
            self.card.open("qarm_usb", self.device_id)

            board_options = (
                "j0_mode=0;j1_mode=0;j2_mode=0;j3_mode=0;"
                "gripper_mode=0;"
                "j0_profile_config=0;j0_profile_velocity=0.25;j0_profile_acceleration=0.25;"
                "j1_profile_config=0;j1_profile_velocity=0.25;j1_profile_acceleration=0.25;"
                "j2_profile_config=0;j2_profile_velocity=0.25;j2_profile_acceleration=0.25;"
                "j3_profile_config=0;j3_profile_velocity=0.25;j3_profile_acceleration=0.25;"
            )

            self.card.set_card_specific_options(board_options, MAX_STRING_LENGTH)

            # PID properties copied from Quanser PAL QArm wrapper.
            pid_property_channels = np.array([
                128, 129, 130, 131,
                133, 134, 135, 136,
                138, 139, 140, 141
            ], dtype=np.int32)

            pid_property_values = np.array([
                8.89, 8.89, 8.89, 8.89,
                0.012, 0.012, 0.012, 0.012,
                10.23, 10.23, 10.23, 10.23
            ], dtype=np.float64)

            self.card.set_double_property(
                pid_property_channels,
                len(pid_property_channels),
                pid_property_values
            )

            self.is_open = True
            print("QArm opened successfully.")

        except HILError as ex:
            print("Could not open QArm.")
            print(ex.get_error_message())
            self.is_open = False

    def close(self):
        """
        Close QArm connection.
        """
        if self.is_open:
            self.card.close()
            self.is_open = False
            print("QArm closed.")

    def read_joints(self):
        """
        Read the four arm joint positions.

        Returns:
            numpy array [base, shoulder, elbow, wrist]
        """
        positions = np.zeros(4, dtype=np.float64)

        self.card.read(
            None, 0,
            None, 0,
            None, 0,
            self.JOINT_CHANNELS, len(self.JOINT_CHANNELS),
            None,
            None,
            None,
            positions
        )

        return positions

    def read_all_positions(self):
        """
        Read four arm joints plus gripper.

        Returns:
            numpy array [base, shoulder, elbow, wrist, gripper]
        """
        positions = np.zeros(5, dtype=np.float64)

        self.card.read(
            None, 0,
            None, 0,
            None, 0,
            self.ALL_POSITION_CHANNELS, len(self.ALL_POSITION_CHANNELS),
            None,
            None,
            None,
            positions
        )

        return positions

    def move_joint_relative(self, joint_index, delta_rad, wait_time=2.0):
        """
        Move only one QArm joint by writing only that joint's HIL channel.

        joint_index:
            0 = base
            1 = shoulder
            2 = elbow
            3 = wrist

        delta_rad:
            Relative movement in radians.
        """
        if joint_index < 0 or joint_index > 3:
            raise ValueError("joint_index must be 0, 1, 2, or 3.")

        # Safety: block large accidental movements
        if abs(delta_rad) > 1.00:
            raise ValueError("delta_rad too large. Use <= 0.05 rad for safety.")

        current = self.read_joints()
        target = current[joint_index] + delta_rad

        channel = np.array([1000 + joint_index], dtype=np.int32)
        command = np.array([target], dtype=np.float64)

        print("Current joints:", current)
        print(f"Moving joint {joint_index}")
        print("Command channel:", channel)
        print("Command value:", command)

        self.card.write(
            None, 0,
            None, 0,
            None, 0,
            channel, 1,
            None,
            None,
            None,
            command
        )

        time.sleep(wait_time)

        after = self.read_joints()
        print("After move:", after)

        return after


if __name__ == "__main__":
    qarm = QArm()

    try:
        qarm.open()

        if not qarm.is_open:
            raise RuntimeError("QArm did not open.")

        print("Initial positions:", qarm.read_all_positions())

        # Test move:
        # Move base joint only by +0.03 rad ≈ +1.7 degrees.
        qarm.move_joint_relative(joint_index=0, delta_rad=0.5, wait_time=2.0)

    finally:
        qarm.close()