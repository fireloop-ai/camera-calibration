import pathlib

import numpy as np
import pinocchio as pin

from lerobot.robots.so_follower import (
    SO101Follower,
    SO101FollowerConfig,
)

PORT = "/dev/ttyACM0"

# Path to the SO-101 URDF (relative to this file)
URDF_PATH = str(pathlib.Path(__file__).resolve().parent.parent / "calibration" / "so101_updated.urdf")

# robot connection
config = SO101FollowerConfig(port=PORT)

robot = SO101Follower(config)

robot.connect()
robot.bus.disable_torque()
# pinocchio model
model = pin.buildModelFromUrdf(URDF_PATH)
data = model.createData()

# IMPORTANT:
# update this frame name after checking URDF
EE_FRAME = "gripper"


def get_joint_state():
    obs = robot.get_observation()

    q_deg = [
        obs["shoulder_pan.pos"],
        obs["shoulder_lift.pos"],
        obs["elbow_flex.pos"],
        obs["wrist_flex.pos"],
        obs["wrist_roll.pos"],
        obs["gripper.pos"],
    ]

    q_rad = np.deg2rad(q_deg)

    return q_rad


def get_T_base_gripper():

    q = get_joint_state()

    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    frame_id = model.getFrameId(EE_FRAME)

    T = data.oMf[frame_id]

    M = np.eye(4)
    M[:3, :3] = T.rotation
    M[:3, 3] = T.translation

    return M


if __name__ == "__main__":

    T = get_T_base_gripper()

    np.set_printoptions(suppress=True)

    print(T)