import numpy as np


def quat_to_rotation_matrix(qx, qy, qz, qw):
    """Convert quaternion to 3x3 rotation matrix."""
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ])


def rotation_matrix_to_quat(R):
    """Convert 3x3 rotation matrix to quaternion (qx, qy, qz, qw)."""
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def pose_to_homogeneous(position, orientation):
    """Convert position/orientation dicts to 4x4 homogeneous matrix.

    Args:
        position: dict with keys 'x', 'y', 'z'
        orientation: dict with keys 'qx', 'qy', 'qz', 'qw'
    """
    H = np.eye(4)
    H[:3, :3] = quat_to_rotation_matrix(
        orientation['qx'], orientation['qy'],
        orientation['qz'], orientation['qw']
    )
    H[0, 3] = position['x']
    H[1, 3] = position['y']
    H[2, 3] = position['z']
    return H


def homogeneous_to_pos_quat(H):
    """Convert 4x4 homogeneous matrix to (x, y, z, qx, qy, qz, qw)."""
    pos = H[:3, 3]
    qx, qy, qz, qw = rotation_matrix_to_quat(H[:3, :3])
    return pos[0], pos[1], pos[2], qx, qy, qz, qw


def compute_pose_from_tag(tag_pose, H_TCP_TO):
    """Compute target TCP pose from an AprilTag detection.

    Args:
        tag_pose: dict with 'position' and 'orientation' sub-dicts
        H_TCP_TO: 4x4 numpy array — calibrated transform from TCP to target object

    Returns:
        4x4 homogeneous matrix of the target pose in base frame
    """
    H_tag = pose_to_homogeneous(tag_pose['position'], tag_pose['orientation'])
    return H_tag @ H_TCP_TO


def compute_approach_pose(H_target, tag_position, pre_height_m, approach_tag_y_offset_m=-0.05):
    """Approach pick/place in link_base: same x, z, and orientation as pre-pick/pre-place.

    ``pre_height_m`` is added to target Z (same as ``pre_pick`` / ``pre_place`` height).
    ``y = tag_position['y'] + approach_tag_y_offset_m`` (default −0.05 m = 5 cm less than tag y).

    Args:
        H_target: 4x4 pick or place TCP in base
        tag_position: dict with 'x', 'y', 'z' (AprilTag in base)
        pre_height_m: added to target z (m), same as compute_poses ``pre_height``
        approach_tag_y_offset_m: added to tag y for approach y (m)

    Returns:
        4x4 homogeneous matrix of the approach pose
    """
    H = H_target.copy()
    ty = float(tag_position['y'])
    H[0, 3] = H_target[0, 3]
    H[1, 3] = ty + float(approach_tag_y_offset_m)
    H[2, 3] = H_target[2, 3] + float(pre_height_m)
    return H
