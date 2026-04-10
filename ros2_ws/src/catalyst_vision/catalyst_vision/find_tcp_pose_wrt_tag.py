#!/usr/bin/env python3
"""Compute H_TCP_TO (gripper pose w.r.t. AprilTag) from poses in link_base.

Given:
  - TCP (gripper) pose in link_base
  - AprilTag pose in link_base

Computes:
  H_TCP_TO = inv(H_tag_in_base) @ H_tcp_in_base

and updates gripper_pose.json with the result.
"""

import json
import os

import numpy as np
from scipy.spatial.transform import Rotation as R


def pose_to_H(position, quat):
    """Convert position [x,y,z] and quaternion [qx,qy,qz,qw] to 4x4 matrix."""
    H = np.eye(4)
    H[:3, :3] = R.from_quat(quat).as_matrix()
    H[:3, 3] = position
    return H


# --- AprilTag pose in link_base (tag_liquid_handler, id=14) ---
tag_pos = [-0.073853, 0.475906, 0.024722]
tag_quat = [0.019612, -0.441784, 0.896861, 0.009113]  # qx, qy, qz, qw

H_tag_base = pose_to_H(tag_pos, tag_quat)

# --- Stand 1: TCP (gripper) pick/place pose in link_base ---
tcp1_pos = [-0.141756, 0.714322, 0.25341]
tcp1_quat = [0.692021, 0.721858, 0.002903, 0.004445]  # qx, qy, qz, qw

# --- Stand 2: TCP (gripper) pick/place pose in link_base ---
tcp2_pos = [0.014501, 0.713115, 0.255184]
tcp2_quat = [0.694224, 0.719742, 0.003223, 0.00381]  # qx, qy, qz, qw

H_tcp1_base = pose_to_H(tcp1_pos, tcp1_quat)
H_tcp2_base = pose_to_H(tcp2_pos, tcp2_quat)

# H_TCP_TO = inv(H_tag_base) @ H_tcp_base
H_TCP_TO_stand1 = np.linalg.inv(H_tag_base) @ H_tcp1_base
H_TCP_TO_stand2 = np.linalg.inv(H_tag_base) @ H_tcp2_base

print("H_TCP_TO (stand 1):")
print(np.array2string(H_TCP_TO_stand1, precision=6, suppress_small=True))
print("\nH_TCP_TO (stand 2):")
print(np.array2string(H_TCP_TO_stand2, precision=6, suppress_small=True))

# --- Update gripper_pose.json ---
config_path = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    '..', 'catalyst_execute', 'config', 'gripper_pose.json'
)
config_path = os.path.normpath(config_path)

data = {
    "H_TCP_TO_stand1": H_TCP_TO_stand1.tolist(),
    "H_TCP_TO_stand2": H_TCP_TO_stand2.tolist(),
}

with open(config_path, 'w') as f:
    json.dump(data, f, indent=4)

print(f"\nUpdated {config_path}")
