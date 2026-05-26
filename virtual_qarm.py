import numpy as np
from qarm import QArm


class VirtualQArm(QArm):
    """
    Small wrapper around the existing QArm class for QLabs Virtual QArm.

    This uses the same QArm HIL interface, but connects to QLabs through TCP/IP:
        0@tcpip://localhost:18900?nagle='off'
    """

    def __init__(self, readMode=0, frequency=500, hilPort=18900):
        super().__init__(
            hardware=0,
            readMode=readMode,
            frequency=frequency,
            deviceId=0,
            hilPort=hilPort
        )
        
    def read_joints(self):
        """
        Read virtual QArm joint positions.

        Returns:
            np.array([base, shoulder, elbow, wrist])
        """
        self.read_std()
        return self.measJointPosition[:4].copy()

    def read_all_positions(self):
        """
        Read virtual QArm joints plus gripper.

        Returns:
            np.array([base, shoulder, elbow, wrist, gripper])
        """
        self.read_std()
        return self.measJointPosition[:5].copy()

    def write_joints(self, joints):
        """
        Write only the 4 arm joints to the virtual QArm.
        Gripper is left at a safe default.
        """
        joints = np.array(joints, dtype=np.float64)

        if joints.shape[0] != 4:
            raise ValueError("joints must contain exactly 4 values.")

        # Safe/default gripper command.
        gripper = np.array([0.1], dtype=np.float64)

        self.write_position(phiCMD=joints, gprCMD=gripper)

    def close(self):
        """
        Alias for terminate(), so main.py can use physical and virtual similarly.
        """
        self.terminate()