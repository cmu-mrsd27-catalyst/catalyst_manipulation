# catalyst_vision

AprilTag pipeline and point cloud preprocessing for the Catalyst Manipulator (RealSense, **`apriltag_ros`**, optional filtering).

## Launch

**AprilTag + RealSense** (used by real-hardware `demo.launch.py`):

- File: `launch/detection_launch.py`
- Includes **`realsense2_camera`** (`rs_launch.py`) with point cloud + aligned depth enabled
- Runs AprilTag continuous detector with `config/apriltag_config.yaml`

```bash
ros2 launch catalyst_vision detection_launch.py
```

## Config files (`config/`)

| File | Purpose |
|------|---------|
| `apriltag_config.yaml` | Tag family, size, topics, frame overrides for `apriltag_ros` |
| `pointcloud_filter.yaml` | Defaults for statistical outlier filter node (see below) |
| `capture_images.yaml` | Defaults for saving color/depth snapshots |

Installed via `setup.py` glob `config/*.yaml`.

## Nodes / executables

| Executable | Role |
|------------|------|
| `pointcloud_filter` | Subscribe raw `PointCloud2`, voxel + statistical outlier removal, publish filtered cloud (OctoMap input). Loads **`pointcloud_filter.yaml`**; ROS `-p` overrides still apply. |
| `capture_images` | Save aligned color/depth PNGs on Enter; loads **`capture_images.yaml`**. |
| `detection_client` | Client / helper for detections (see source). |
| `pointcloud_filter` params in **demo.launch.py** may override YAML when passed as node parameters. |

## Dependencies

`package.xml`: `rclpy`, `sensor_msgs`, `realsense2_camera`, `apriltag_ros`, `tf2_ros`, `python3-scipy`, **`ament_index_python`**, **`python3-yaml`**, **`cv_bridge`** (for capture tool).

## See also

- [catalyst_bringup README](../catalyst_bringup/README.md) — starts vision in real `demo.launch.py`
- [catalyst_world_model README](../catalyst_world_model/README.md) — consumes detections in `/world_model`
- [Repository README](../../../README.md)
