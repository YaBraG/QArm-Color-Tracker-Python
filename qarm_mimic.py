import time
from typing import Optional

import numpy as np

from qarm import QArm
from virtual_qarm import VirtualQArm


class QArmMimic:
    """
    Library for keeping a QLabs Virtual QArm synchronized with a physical QArm.

    Main idea:
        - physical QArm provides measured joint positions
        - virtual QArm receives those joint positions
        - main.py can use this as a background/helper library later

    Files used:
        qarm.py          -> physical QArm
        virtual_qarm.py  -> QLabs virtual QArm
    """

    HOME_POSE = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
    GRIPPER_OPEN = np.array([0.1], dtype=np.float64)

    LED_OFF = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    LED_RED = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    LED_GREEN = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    LED_BLUE = np.array([0.0, 0.0, 1.0], dtype=np.float64)

    def __init__(self, sample_time=0.05, record_positions=True):
        """
        Args:
            sample_time:
                Time between mimic updates.
                0.05 seconds = 20 Hz.

            record_positions:
                If True, saves physical joint positions into self.position_log.
        """
        self.sample_time = sample_time
        self.record_positions = record_positions

        self.physical: Optional[QArm] = None
        self.virtual: Optional[VirtualQArm] = None

        self.is_open = False
        self.is_unlocked = False
        self.position_log: list[dict[str, object]] = []

    def open(self):
        """
        Open physical and virtual QArms.
        """
        print("Opening physical QArm...")
        physical = QArm(hardware=1)

        if not physical.status:
            raise RuntimeError("Physical QArm did not open.")

        print("Opening virtual QArm...")
        virtual = VirtualQArm()

        if not virtual.status:
            physical.terminate()
            raise RuntimeError("Virtual QArm did not open. Check that QLabs is open.")

        self.physical = physical
        self.virtual = virtual
        self.is_open = True

        print("Physical and virtual QArms opened.")

    def close(self):
        """
        Close both QArms safely.
        """
        physical = self.physical
        virtual = self.virtual

        if physical is not None and physical.status:
            if self.is_unlocked:
                self.lock_physical_position_mode()

            physical.terminate()

        if virtual is not None and virtual.status:
            virtual.terminate()

        self.physical = None
        self.virtual = None
        self.is_open = False
        self.is_unlocked = False

        print("QArm mimic closed.")

    def home_both(self, wait_time=2.0):
        """
        Move physical and virtual QArms to home pose.
        """
        physical = self._get_physical()
        virtual = self._get_virtual()

        print("Moving physical QArm to home...")
        self.lock_physical_position_mode()
        physical.write_position(
            phiCMD=self.HOME_POSE,
            gprCMD=self.GRIPPER_OPEN
        )

        print("Moving virtual QArm to home...")
        virtual.write_position(
            phiCMD=self.HOME_POSE,
            gprCMD=self.GRIPPER_OPEN
        )

        time.sleep(wait_time)

        physical.read_std()
        virtual.read_std()

        print("Physical home position:", physical.measJointPosition[:5])
        print("Virtual home position:", virtual.measJointPosition[:5])

    def set_leds(self, led_color):
        """
        Set both QArm base LEDs.

        Args:
            led_color:
                np.array([red, green, blue])
        """
        physical = self._get_physical()
        virtual = self._get_virtual()

        led_color = np.array(led_color, dtype=np.float64)

        physical.write_led(led_color)
        virtual.write_led(led_color)

    def set_green_ready(self):
        """
        Convenience function: set both LEDs green.
        """
        self.set_leds(self.LED_GREEN)

    def lock_physical_position_mode(self):
        """
        Put physical QArm back into position mode.
        """
        board_options = (
            "j0_mode=0;j1_mode=0;j2_mode=0;j3_mode=0;"
            "gripper_mode=0;"
            "j0_profile_config=0;j0_profile_velocity=0.25;j0_profile_acceleration=0.25;"
            "j1_profile_config=0;j1_profile_velocity=0.25;j1_profile_acceleration=0.25;"
            "j2_profile_config=0;j2_profile_velocity=0.25;j2_profile_acceleration=0.25;"
            "j3_profile_config=0;j3_profile_velocity=0.25;j3_profile_acceleration=0.25;"
        )

        self._set_physical_board_options(board_options)
        self.is_unlocked = False

    def unlock_physical_free_movement(self):
        """
        Put physical QArm joints into PWM mode with zero PWM.

        This allows the physical QArm to be moved by hand while encoders are read.
        """
        physical = self._get_physical()

        board_options = (
            "j0_mode=1;j1_mode=1;j2_mode=1;j3_mode=1;"
            "gripper_mode=0;"
        )
        self._set_physical_board_options(board_options)

        pwm_channels = np.array([11000, 11001, 11002, 11003], dtype=np.int32)
        zero_pwm = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

        physical.card.write(
            None, 0,
            None, 0,
            None, 0,
            pwm_channels, len(pwm_channels),
            None,
            None,
            None,
            zero_pwm
        )

        self.is_unlocked = True

    def read_physical_pose(self):
        """
        Read physical QArm measured joint and gripper positions.

        Returns:
            joints:
                np.array([base, shoulder, elbow, wrist])

            gripper:
                np.array([gripper])
        """
        physical = self._get_physical()

        physical.read_std()

        joints = physical.measJointPosition[:4].copy()
        gripper = np.array([physical.measJointPosition[4]], dtype=np.float64)

        return joints, gripper

    def physical_to_virtual(self, physical_joints):
        """
        Convert physical QArm joint values into virtual QArm joint values.

        Right now this is 1:1.
        If the virtual arm later needs sign flips or offsets, change them here.
        """
        return physical_joints.copy()

    def send_to_virtual(self, joints, gripper=None):
        """
        Send joint/gripper command to virtual QArm.
        """
        virtual = self._get_virtual()

        joints = np.array(joints, dtype=np.float64)

        if gripper is None:
            gripper = self.GRIPPER_OPEN
        else:
            gripper = np.array(gripper, dtype=np.float64)

        virtual.write_position(
            phiCMD=joints,
            gprCMD=gripper
        )

    def step(self):
        """
        One mimic update.

        Read physical QArm, convert pose, send to virtual QArm.
        """
        self._require_open()

        if self.is_unlocked:
            # Keep PWM at zero while free-movement mimic is active.
            self.unlock_physical_free_movement()

        physical_joints, physical_gripper = self.read_physical_pose()
        virtual_joints = self.physical_to_virtual(physical_joints)

        self.send_to_virtual(virtual_joints, physical_gripper)

        if self.record_positions:
            self.position_log.append({
                "time": time.time(),
                "physical_joints": physical_joints.copy(),
                "virtual_joints": virtual_joints.copy(),
                "gripper": physical_gripper.copy()
            })

        return physical_joints, virtual_joints, physical_gripper

    def run_free_mimic(self):
        """
        Run mimic loop until Ctrl+C.

        Physical QArm is unlocked for hand movement.
        Virtual QArm follows the measured physical joint positions.
        """
        self._require_open()

        self.unlock_physical_free_movement()

        print("Free mimic running.")
        print("Move the physical QArm by hand.")
        print("Press Ctrl+C to stop.")

        try:
            while True:
                physical_joints, virtual_joints, gripper = self.step()

                print("Physical:", physical_joints, "Virtual:", virtual_joints)

                time.sleep(self.sample_time)

        except KeyboardInterrupt:
            print("\nStopping free mimic.")

        finally:
            self.lock_physical_position_mode()

    def _set_physical_board_options(self, board_options):
        """
        Set card-specific options on the physical QArm.

        Keeping this as a named helper avoids duplicated HIL calls and keeps
        the type checker from treating self.physical as possibly None.
        """
        physical = self._get_physical()
        physical.card.set_card_specific_options(board_options, 2048)

    def _require_open(self):
        """
        Internal safety check.
        """
        if not self.is_open or self.physical is None or self.virtual is None:
            raise RuntimeError("QArmMimic is not open. Call mimic.open() first.")

    def _get_physical(self):
        """
        Return the physical QArm after checking that it exists.

        This method is mainly here so VS Code/Pylance knows the return value
        is a QArm and not None.
        """
        self._require_open()
        assert self.physical is not None
        return self.physical

    def _get_virtual(self):
        """
        Return the virtual QArm after checking that it exists.

        This method is mainly here so VS Code/Pylance knows the return value
        is a VirtualQArm and not None.
        """
        self._require_open()
        assert self.virtual is not None
        return self.virtual

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
