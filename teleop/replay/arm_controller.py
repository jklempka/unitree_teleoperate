"""Arm controller used for replaying recorded episodes.

This module provides a thin wrapper around the Unitree DDS publisher to send
joint commands to either the real robot or a local simulation.  The controller
mirrors the behaviour of :class:`teleop.robot_control.robot_arm.G1_29_ArmController`
by configuring appropriate stiffness and damping gains for the arm joints and
selecting the correct DDS topic for motion or debug modes.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Protocol, Sequence

import numpy as np

try:  # pragma: no cover - optional dependency
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as hg_LowCmd
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as hg_LowState
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.utils.crc import CRC
except ModuleNotFoundError:  # pragma: no cover - allow import without sdk
    ChannelFactoryInitialize = (
        ChannelPublisher
    ) = ChannelSubscriber = hg_LowCmd = hg_LowState = unitree_hg_msg_dds__LowCmd_ = CRC = None  # type: ignore


kTopicLowCommand_Debug = "rt/lowcmd"
kTopicLowCommand_Motion = "rt/arm_sdk"
kTopicLowState = "rt/lowstate"

KP_LOW = 80.0
KD_LOW = 3.0
KP_WRIST = 40.0
KD_WRIST = 1.5


class ArmController(Protocol):
    """Protocol for classes capable of moving both arms."""

    def send_joint_positions(self, positions: Sequence[float]) -> None:
        """Command both arms to move to ``positions``."""

    def get_joint_positions(self) -> Sequence[float]:
        """Return the current joint angles for both arms."""


class G1ArmJointIndex(IntEnum):
    """Indices for the G1 arm joints used when publishing commands."""

    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristYaw = 21

    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28


LEFT_ARM_JOINTS = [
    G1ArmJointIndex.kLeftShoulderPitch,
    G1ArmJointIndex.kLeftShoulderRoll,
    G1ArmJointIndex.kLeftShoulderYaw,
    G1ArmJointIndex.kLeftElbow,
    G1ArmJointIndex.kLeftWristRoll,
    G1ArmJointIndex.kLeftWristPitch,
    G1ArmJointIndex.kLeftWristYaw,
]

RIGHT_ARM_JOINTS = [
    G1ArmJointIndex.kRightShoulderPitch,
    G1ArmJointIndex.kRightShoulderRoll,
    G1ArmJointIndex.kRightShoulderYaw,
    G1ArmJointIndex.kRightElbow,
    G1ArmJointIndex.kRightWristRoll,
    G1ArmJointIndex.kRightWristPitch,
    G1ArmJointIndex.kRightWristYaw,
]


DUAL_ARM_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS


@dataclass
class DDSArmController:
    """Send joint commands over DDS to control both robot arms."""

    motion_mode: bool = False
    simulation_mode: bool = False

    def __post_init__(self) -> None:
        ChannelFactoryInitialize(1 if self.simulation_mode else 0)
        topic = kTopicLowCommand_Motion if self.motion_mode else kTopicLowCommand_Debug
        self.publisher = ChannelPublisher(topic, hg_LowCmd)
        self.publisher.Init()
        self.subscriber = ChannelSubscriber(kTopicLowState, hg_LowState)
        self.subscriber.Init()
        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = 0

        wrist_joints = {
            G1ArmJointIndex.kLeftWristRoll,
            G1ArmJointIndex.kLeftWristPitch,
            G1ArmJointIndex.kLeftWristYaw,
            G1ArmJointIndex.kRightWristRoll,
            G1ArmJointIndex.kRightWristPitch,
            G1ArmJointIndex.kRightWristYaw,
        }

        for joint in G1ArmJointIndex:
            cmd = self.msg.motor_cmd[joint]
            cmd.mode = 1
            if joint in wrist_joints:
                cmd.kp = KP_WRIST
                cmd.kd = KD_WRIST
            else:
                cmd.kp = KP_LOW
                cmd.kd = KD_LOW
            cmd.q = 0.0
            cmd.dq = 0.0
            cmd.tau = 0.0

    def send_joint_positions(self, positions: Sequence[float]) -> None:
        qpos = np.asarray(positions, dtype=float)
        for idx, joint in enumerate(DUAL_ARM_JOINTS):
            cmd = self.msg.motor_cmd[joint]
            cmd.q = float(qpos[idx])
            cmd.dq = 0.0
            cmd.tau = 0.0
        self.msg.crc = self.crc.Crc(self.msg)
        self.publisher.Write(self.msg)

    def get_joint_positions(self) -> Sequence[float]:
        """Return the most recently published joint positions."""
        msg = self.subscriber.Read()
        if msg is None:
            return np.zeros(len(DUAL_ARM_JOINTS), dtype=float)
        return [float(msg.motor_state[j].q) for j in DUAL_ARM_JOINTS]

