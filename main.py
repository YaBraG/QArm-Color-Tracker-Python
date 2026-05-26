from qarm_mimic import QArmMimic


def main():
    mimic = QArmMimic(sample_time=0.05, record_positions=True)

    try:
        mimic.open()

        mimic.home_both()
        mimic.set_green_ready()

        answer = input("Ready? Type y to unlock physical joints and start mimic: ").strip().lower()

        if answer != "y":
            print("Cancelled.")
            return

        mimic.run_free_mimic()

    finally:
        mimic.close()


if __name__ == "__main__":
    main()