import sys
from pathlib import Path
from unittest.mock import ANY, MagicMock, patch

sys.path.append(str(Path(__file__).resolve().parents[3]))

from teleop.replay.arm_controller import (
    DDSArmController,
    G1ArmJointIndex,
    DUAL_ARM_JOINTS,
    KP_LOW,
    KP_WRIST,
    kTopicLowCommand_Debug,
)


def test_controller_initializes_and_sends() -> None:
    with (
        patch("teleop.replay.arm_controller.ChannelFactoryInitialize") as cf_init,
        patch("teleop.replay.arm_controller.ChannelPublisher") as pub_cls,
        patch("teleop.replay.arm_controller.ChannelSubscriber") as sub_cls,
        patch("teleop.replay.arm_controller.unitree_hg_msg_dds__LowCmd_") as msg_cls,
        patch("teleop.replay.arm_controller.hg_LowState") as lowstate_cls,
        patch("teleop.replay.arm_controller.CRC") as crc_cls,
    ):

        publisher = MagicMock()
        pub_cls.return_value = publisher

        msg = MagicMock()
        msg.motor_cmd = {joint: MagicMock() for joint in G1ArmJointIndex}
        msg_cls.return_value = msg

        crc = MagicMock()
        crc.Crc.return_value = 0
        crc_cls.return_value = crc

        lowstate = MagicMock()
        lowstate.motor_state = {joint: MagicMock(q=float(joint.value)) for joint in G1ArmJointIndex}
        subscriber = MagicMock()
        subscriber.Read.return_value = lowstate
        sub_cls.return_value = subscriber

        controller = DDSArmController()
        pub_cls.assert_called_once_with(kTopicLowCommand_Debug, ANY)
        cf_init.assert_called()

        shoulder = msg.motor_cmd[G1ArmJointIndex.kLeftShoulderPitch]
        wrist = msg.motor_cmd[G1ArmJointIndex.kLeftWristRoll]
        assert shoulder.kp == KP_LOW
        assert wrist.kp == KP_WRIST

        controller.send_joint_positions([0.1] * len(DUAL_ARM_JOINTS))
        publisher.Write.assert_called_once()
        for joint in DUAL_ARM_JOINTS:
            assert msg.motor_cmd[joint].q == 0.1
            assert msg.motor_cmd[joint].tau == 0.0

        # test get_joint_positions reads from subscriber
        positions = controller.get_joint_positions()
        assert positions == [float(j.value) for j in DUAL_ARM_JOINTS]


def test_motion_mode_topic() -> None:
    with (
        patch("teleop.replay.arm_controller.ChannelFactoryInitialize"),
        patch("teleop.replay.arm_controller.ChannelPublisher") as pub_cls,
        patch("teleop.replay.arm_controller.ChannelSubscriber"),
        patch("teleop.replay.arm_controller.unitree_hg_msg_dds__LowCmd_") as msg_cls,
        patch("teleop.replay.arm_controller.CRC") as crc_cls,
    ):

        publisher = MagicMock()
        pub_cls.return_value = publisher

        msg = MagicMock()
        msg.motor_cmd = {joint: MagicMock() for joint in G1ArmJointIndex}
        msg_cls.return_value = msg

        crc = MagicMock()
        crc_cls.return_value = crc

        DDSArmController(motion_mode=True)
        pub_cls.assert_called_once_with("rt/arm_sdk", ANY)

