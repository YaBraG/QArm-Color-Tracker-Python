import time
import numpy as np

from qarm import QArm
from virtual_qarm import VirtualQArm


HOME_POSE = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
GRIPPER_OPEN = np.array([0.1], dtype=np.float64)
LED_GREEN = np.array([0.0, 1.0, 0.0], dtype=np.float64)


def set_physical_pwm_mode_zero(qarm):
    """
    Put physical QArm joints into PWM mode and command 0 PWM.

    This is intended to stop active position-hold so the arm can be moved by hand.
    """
    board_options = (
        "j0_mode=1;j1_mode=1;j2_mode=1;j3_mode=1;"
        "gripper_mode=0;"
    )

    qarm.card.set_card_specific_options(board_options, 2048)

    pwm_channels = np.array([11000, 11001, 11002, 11003], dtype=np.int32)
    zero_pwm = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    qarm.card.write(
        None, 0,
        None, 0,
        None, 0,
        pwm_channels, len(pwm_channels),
        None,
        None,
        None,
        zero_pwm
    )


def set_physical_position_mode(qarm):
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

    qarm.card.set_card_specific_options(board_options, 2048)


def move_to_home(qarm, label):
    """
    Move a QArm object to home position using the existing qarm.py write_position method.
    """
    print(f"Moving {label} to home...")
    qarm.write_position(phiCMD=HOME_POSE, gprCMD=GRIPPER_OPEN)
    time.sleep(2.0)
    qarm.read_std()
    print(f"{label} position:", qarm.measJointPosition[:5])


def set_led_green(qarm, label):
    """
    Turn QArm base LED green.
    """
    print(f"Setting {label} LED green...")
    qarm.write_led(LED_GREEN)


def main():
    physical = None
    virtual = None

    try:
        print("Opening physical QArm...")
        physical = QArm(hardware=1)

        if not physical.status:
            print("ERROR: Physical QArm did not open.")
            return

        print("Opening virtual QArm...")
        virtual = VirtualQArm()

        if not virtual.status:
            print("ERROR: Virtual QArm did not open.")
            return

        # Start both in position mode
        set_physical_position_mode(physical)

        # Move both to home
        move_to_home(physical, "physical QArm")
        move_to_home(virtual, "virtual QArm")

        # Turn LEDs green
        set_led_green(physical, "physical QArm")
        set_led_green(virtual, "virtual QArm")

        answer = input("Ready? Type y to unlock physical joints and start mimic: ").strip().lower()

        if answer != "y":
            print("Cancelled.")
            return

        print("Unlocking physical joints...")
        set_physical_pwm_mode_zero(physical)

        print("Mimic running. Move the physical arm by hand.")
        print("Press Ctrl+C to stop.")

        while True:
            # Keep physical joints unpowered/free
            set_physical_pwm_mode_zero(physical)

            # Read physical encoder positions
            physical.read_std()
            physical_joints = physical.measJointPosition[:4].copy()
            physical_gripper = np.array([physical.measJointPosition[4]], dtype=np.float64)

            # For now, direct 1:1 mapping
            virtual_joints = physical_joints.copy()

            # Send physical position to virtual QArm
            virtual.write_position(phiCMD=virtual_joints, gprCMD=physical_gripper)

            print("Physical:", physical_joints, "Gripper:", physical_gripper)

            time.sleep(0.05)  # 20 Hz

    except KeyboardInterrupt:
        print("\nStopping mimic.")

    finally:
        if physical is not None and physical.status:
            print("Returning physical QArm to position mode.")
            set_physical_position_mode(physical)
            physical.terminate()

        if virtual is not None and virtual.status:
            virtual.terminate()


if __name__ == "__main__":
    main()