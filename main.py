from qarm import QArm


def main():
    qarm = QArm()

    try:
        qarm.open()

        if not qarm.is_open:
            print("QArm failed to open.")
            return

        print("Initial positions:")
        print(qarm.read_all_positions())

        # Move base joint only
        qarm.move_joint_relative(joint_index=0, delta_rad=0.03, wait_time=2.0)

        # Read final positions
        print("Final positions:")
        print(qarm.read_all_positions())

    finally:
        qarm.close()


if __name__ == "__main__":
    main()