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


def _parse_tool_axis_toward_tag(name):
    """Map YAML string → (column_index 0..2, sign ±1)."""
    if name is None:
        return 2, 1.0
    key = str(name).strip().lower().replace('-', '_')
    table = {
        'plus_z': (2, 1.0),
        'tcp_plus_z': (2, 1.0),
        'z': (2, 1.0),
        'minus_z': (2, -1.0),
        'tcp_minus_z': (2, -1.0),
        'neg_z': (2, -1.0),
        'plus_x': (0, 1.0),
        'tcp_plus_x': (0, 1.0),
        'x': (0, 1.0),
        'minus_x': (0, -1.0),
        'tcp_minus_x': (0, -1.0),
        'neg_x': (0, -1.0),
        'plus_y': (1, 1.0),
        'tcp_plus_y': (1, 1.0),
        'y': (1, 1.0),
        'minus_y': (1, -1.0),
        'tcp_minus_y': (1, -1.0),
        'neg_y': (1, -1.0),
    }
    return table.get(key, (2, 1.0))


def _orthonormal_rotation_column_k_along_u(column_k, u_unit, world_up):
    """Right-handed ``R`` (3×3) with ``R[:, column_k] == u_unit`` (unit)."""
    u = np.asarray(u_unit, dtype=np.float64)
    u = u / float(np.linalg.norm(u))
    up = np.asarray(world_up, dtype=np.float64)
    un = float(np.linalg.norm(up))
    if un > 1e-9:
        up = up / un
    else:
        up = np.array([0.0, 0.0, 1.0])
    others = [a for a in (0, 1, 2) if a != column_k]
    i, j = others[0], others[1]
    R = np.zeros((3, 3), dtype=np.float64)
    R[:, column_k] = u
    w = np.cross(up, u)
    if float(np.linalg.norm(w)) < 1e-6:
        w = np.cross(np.array([0.0, 1.0, 0.0]), u)
    w = w / float(np.linalg.norm(w))
    R[:, i] = w
    R[:, j] = np.cross(R[:, column_k], R[:, i])
    nj = float(np.linalg.norm(R[:, j]))
    if nj < 1e-9:
        raise ValueError('head_on_tcp_command_for_tag: degenerate look-at basis')
    R[:, j] = R[:, j] / nj
    R[:, i] = np.cross(R[:, j], R[:, column_k])
    ni = float(np.linalg.norm(R[:, i]))
    if ni < 1e-9:
        raise ValueError('head_on_tcp_command_for_tag: degenerate look-at basis')
    R[:, i] = R[:, i] / ni
    if float(np.linalg.det(R)) < 0.0:
        R[:, j] = -R[:, j]
    return R


def _rotation_about_local_x_deg(degrees):
    """Rotation matrix (TCP frame): right-handed +X axis by ``degrees``."""
    th = np.radians(float(degrees))
    c, s = float(np.cos(th)), float(np.sin(th))
    return np.array(
        [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]],
        dtype=np.float64,
    )


def head_on_tcp_command_for_tag(
        tag_pose,
        distance_m,
        axis_sign=-1.0,
        world_up=(0.0, 0.0, 1.0),
        tool_axis_toward_tag='plus_x',
        post_rotate_tcp_x_deg=0.0,
):
    """TCP pose in ``link_base`` for an approximately head-on view of the tag.

    **Position:** tag origin plus ``axis_sign * distance_m`` times the tag Z
    axis expressed in base (third column of the tag rotation). AprilTag / TF
    conventions differ; flip ``axis_sign`` in YAML if the arm moves the
    wrong way through the table.

    **Orientation:** the **link_tcp** axis given by ``tool_axis_toward_tag``
    (e.g. ``plus_x`` = +X_tcp, ``plus_z`` = +Z_tcp) points **from TCP toward
    the tag**. Pick the axis that matches your camera / tool convention; if
    the arm stands off at the right distance but appears to look *past* the tag
    (90° off), try ``plus_x`` or ``minus_z`` instead of ``plus_z``.

    Args:
        tag_pose: ``{'position': {x,y,z}, 'orientation': {qx,qy,qz,qw}}`` in
            ``link_base`` (same as ``/world_model`` ``pose_in_base``).
        distance_m: Standoff distance (m), e.g. ``0.4`` for ~40 cm.
        axis_sign: +1 or −1 to choose which side of the tag plane along local Z.
        world_up: Reference up vector in ``link_base`` for the look-at basis.
        tool_axis_toward_tag: ``plus_x`` | ``minus_x`` | ``plus_y`` | ``minus_y``
            | ``plus_z`` | ``minus_z`` (aliases: ``tcp_plus_x``, …).
        post_rotate_tcp_x_deg: Extra rotation **in the TCP frame** after the
            look-at basis, about **local +X_tcp** (degrees). Use ``180`` when
            the view is head-on but the camera appears flipped 180° about
            ``link_tcp`` X (e.g. upside-down); ``0`` disables.

    Returns:
        Dict with ``x, y, z, qx, qy, qz, qw`` suitable for ``/cartesian_command``
        (add ``speed`` at the call site).
    """
    pos = tag_pose['position']
    ori = tag_pose['orientation']
    p_tag = np.array(
        [float(pos['x']), float(pos['y']), float(pos['z'])], dtype=np.float64)
    H = pose_to_homogeneous(pos, ori)
    z_axis = H[:3, 2].copy()
    zn = float(np.linalg.norm(z_axis))
    if zn < 1e-9:
        raise ValueError('head_on_tcp_command_for_tag: invalid tag orientation')
    z_axis /= zn
    p_tcp = p_tag + float(axis_sign) * float(distance_m) * z_axis
    fwd = p_tag - p_tcp
    fn = float(np.linalg.norm(fwd))
    if fn < 1e-9:
        fwd = -z_axis * float(np.sign(axis_sign) or 1.0)
        fn = float(np.linalg.norm(fwd))
    fwd = fwd / fn
    upw = np.array(world_up, dtype=np.float64)
    unn = float(np.linalg.norm(upw))
    if unn > 1e-9:
        upw = upw / unn
    else:
        upw = np.array([0.0, 0.0, 1.0])
    col_k, col_sign = _parse_tool_axis_toward_tag(tool_axis_toward_tag)
    u = col_sign * fwd
    R = _orthonormal_rotation_column_k_along_u(col_k, u, upw)
    prx = float(post_rotate_tcp_x_deg)
    if abs(prx) > 1e-6:
        R = R @ _rotation_about_local_x_deg(prx)
    qx, qy, qz, qw = rotation_matrix_to_quat(R)
    return {
        'x': float(p_tcp[0]),
        'y': float(p_tcp[1]),
        'z': float(p_tcp[2]),
        'qx': float(qx),
        'qy': float(qy),
        'qz': float(qz),
        'qw': float(qw),
    }


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
