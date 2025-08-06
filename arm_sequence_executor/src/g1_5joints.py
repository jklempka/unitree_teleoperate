from typing import List

# -----------------------------------------------------------------------------
# Constants / Joint indices
# -----------------------------------------------------------------------------
class G1JointIndex:
    """Enumerates the joint indices for a 23-DoF Unitree G1 arms subset.

    Only 5 joints per arm are valid for the 23-DoF configuration. Wrist Pitch/Yaw
    are not used; we retain constants commented for reference.
    """

    # Left arm
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    # LeftWristPitch = 20  # invalid
    # LeftWristYaw = 21    # invalid

    # Right arm
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    # RightWristPitch = 27 # invalid
    # RightWristYaw = 28   # invalid

    kNotUsedJoint = 29  # sentinel used by SDK for enable/disable

    @classmethod
    def left_arm_indices(cls) -> List[int]:
        return [
            cls.LeftShoulderPitch,
            cls.LeftShoulderRoll,
            cls.LeftShoulderYaw,
            cls.LeftElbow,
            cls.LeftWristRoll,
        ]

    @classmethod
    def right_arm_indices(cls) -> List[int]:
        return [
            cls.RightShoulderPitch,
            cls.RightShoulderRoll,
            cls.RightShoulderYaw,
            cls.RightElbow,
            cls.RightWristRoll,
        ]
