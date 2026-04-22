"""Helpers for robot-side container pick/place actions (hardcoded poses in link_base)."""

import math


def workspace_box_restore_from_cfg(cfg: dict) -> dict:
    """Geometry to re-add workspace_box after remove (default = planning_scene_static_objects)."""
    default = {
        'frame_id': 'link_base',
        'position': [0.27, 0.0, 0.04],
        'dimensions': [0.14, 0.10, 0.08],
        'orientation': [0.0, 0.0, 0.0, 1.0],
    }
    raw = cfg.get('workspace_box_restore_collision')
    if not raw:
        return default
    out = dict(default)
    out['frame_id'] = raw.get('frame_id', default['frame_id'])
    pos = raw.get('position', default['position'])
    if isinstance(pos, dict):
        out['position'] = [
            float(pos['x']), float(pos['y']), float(pos['z'])]
    else:
        out['position'] = [float(x) for x in pos]
    dim = raw.get('dimensions', default['dimensions'])
    if isinstance(dim, dict):
        out['dimensions'] = [
            float(dim['x']), float(dim['y']), float(dim['z'])]
    else:
        out['dimensions'] = [float(x) for x in dim]
    ori = raw.get('orientation', default['orientation'])
    if isinstance(ori, dict):
        out['orientation'] = [
            float(ori['qx']), float(ori['qy']),
            float(ori['qz']), float(ori['qw'])]
    else:
        out['orientation'] = [float(x) for x in ori]
    return out


def parse_container_joint_transit(cfg: dict):
    """Optional fixed joint waypoints: home → down_right → pre (pick/place).

    YAML block ``container_transit_joint_waypoints`` with ``enabled: true`` and
    six-element ``joints_down_right`` / ``joints_pre_container`` (degrees).
    Returns None if disabled or invalid (caller falls back to Cartesian).
    """
    raw = cfg.get('container_transit_joint_waypoints')
    if not isinstance(raw, dict) or not raw.get('enabled', False):
        return None
    speed = float(raw.get('speed', 0.1))
    jd = raw.get('joints_down_right')
    jp = raw.get('joints_pre_container')
    if not (isinstance(jd, (list, tuple)) and len(jd) == 6
            and isinstance(jp, (list, tuple)) and len(jp) == 6):
        return None
    return {
        'speed': speed,
        'joints_down_right': [float(x) for x in jd],
        'joints_pre_container': [float(x) for x in jp],
    }


def pose_cfg_to_cartesian_dict(pose_cfg: dict) -> dict:
    """Build /cartesian_command JSON from a pose block in catalyst_execute_params.yaml."""
    return {
        'x': float(pose_cfg['x']),
        'y': float(pose_cfg['y']),
        'z': float(pose_cfg['z']),
        'qx': float(pose_cfg['qx']),
        'qy': float(pose_cfg['qy']),
        'qz': float(pose_cfg['qz']),
        'qw': float(pose_cfg['qw']),
        'speed': float(pose_cfg.get('speed', 0.1)),
    }


def apply_pre_place_planar_offsets(
        pose_cfg: dict, offset_x_m: float, offset_y_m: float) -> dict:
    """Copy pose block with x += offset_x_m, y += offset_y_m (same as /compute_poses pre_place_offset)."""
    out = dict(pose_cfg)
    out['x'] = float(pose_cfg['x']) + float(offset_x_m)
    out['y'] = float(pose_cfg['y']) + float(offset_y_m)
    return out


def _quat_to_rpy_xyz_deg(qx: float, qy: float, qz: float, qw: float):
    """Quaternion (qx,qy,qz,qw) → intrinsic XYZ Euler in degrees (xArm-style RPY)."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def sdk_z_delta_position_move_cmd(
        position_mm_rpy_deg,
        z_delta_mm: float,
        speed_mm_s: float = 30.0,
        tolerance_mm: float = 1.0,
        max_corrections: int = 10,
) -> dict:
    """get_position() list [x,y,z mm, roll,pitch,yaw deg] → position_move with Z += z_delta_mm."""
    x, y, z, roll, pitch, yaw = position_mm_rpy_deg[:6]
    return {
        'action': 'position_move',
        'x': float(x),
        'y': float(y),
        'z': float(z) + float(z_delta_mm),
        'roll': float(roll),
        'pitch': float(pitch),
        'yaw': float(yaw),
        'speed': float(speed_mm_s),
        'tolerance': float(tolerance_mm),
        'max_corrections': int(max_corrections),
    }


def cartesian_dict_to_sdk_position_move(
        cart: dict,
        speed_mm_s: float = 30.0,
        tolerance_mm: float = 1.0,
        max_corrections: int = 10,
) -> dict:
    """link_base pose (m + quat) → /sdk_control position_move (mm + deg RPY)."""
    qx = float(cart['qx'])
    qy = float(cart['qy'])
    qz = float(cart['qz'])
    qw = float(cart['qw'])
    roll, pitch, yaw = _quat_to_rpy_xyz_deg(qx, qy, qz, qw)
    return {
        'action': 'position_move',
        'x': float(cart['x']) * 1000.0,
        'y': float(cart['y']) * 1000.0,
        'z': float(cart['z']) * 1000.0,
        'roll': roll,
        'pitch': pitch,
        'yaw': yaw,
        'speed': float(speed_mm_s),
        'tolerance': float(tolerance_mm),
        'max_corrections': int(max_corrections),
    }
