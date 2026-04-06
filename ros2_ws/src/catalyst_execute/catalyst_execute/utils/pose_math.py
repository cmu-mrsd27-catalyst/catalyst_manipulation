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


def compute_approach_pose(H_target, H_tag):
    """Compute approach pose: same orientation as target, pulled back along tag Z to 5cm ahead of tag.

    Args:
        H_target: 4x4 homogeneous matrix of the target pose
        H_tag: 4x4 homogeneous matrix of the AprilTag pose

    Returns:
        4x4 homogeneous matrix of the approach pose
    """
    tag_pos = H_tag[:3, 3]
    tag_z = H_tag[:3, 2]  # tag Z axis (points outward)
    target_pos = H_target[:3, 3]

    # Project target onto tag Z axis (distance from tag along tag Z)
    depth = np.dot(target_pos - tag_pos, tag_z)
    # Pull back to 5cm ahead of tag
    offset = 0.05 - depth
    approach_pos = target_pos + offset * tag_z

    H_approach = H_target.copy()
    H_approach[:3, 3] = approach_pos
    return H_approach
