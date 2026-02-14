#!/usr/bin/env python3

import os
import sys
import time
import yaml
from dataclasses import dataclass, fields
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# -------------------- AX-18A Control Table --------------------
ADDR_CW_ANGLE_LIMIT     = 6    # 2 bytes
ADDR_CCW_ANGLE_LIMIT    = 8    # 2 bytes
ADDR_TORQUE_ENABLE      = 24   # 1 byte
ADDR_GOAL_POSITION      = 30   # 2 bytes
ADDR_MOVING_SPEED       = 32   # 2 bytes
ADDR_TORQUE_LIMIT       = 34   # 2 bytes

ADDR_PRESENT_POSITION   = 36   # 2 bytes
ADDR_PRESENT_SPEED      = 38   # 2 bytes
ADDR_PRESENT_LOAD       = 40   # 2 bytes

TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0

# Default path to the config file (relative to this package)
DEFAULT_CONFIG_PATH = os.path.join(
    os.path.dirname(__file__), "..", "config", "gripper_params.yaml"
)

def _dxl_ok(result, err, ph, ctx=""):
    if result != COMM_SUCCESS:
        raise RuntimeError(f"{ctx} COMM failed: {ph.getTxRxResult(result)}")
    if err != 0:
        raise RuntimeError(f"{ctx} DXL error: {ph.getRxPacketError(err)}")


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _mag10(raw):
    """Strip direction bit, keep magnitude 0..1023."""
    return raw & 1023


def _decode_signed_10bit_with_dir(raw):
    """
    AX speed/load encoding:
      bit10 (1024) = direction
      bits0..9     = magnitude (0..1023)
    Return signed integer in [-1023, +1023].
    """
    direction = -1 if (raw & 1024) else 1
    magnitude = raw & 1023
    return direction * magnitude


def _wheel_speed_value(speed, direction):
    """
    Moving Speed encoding for wheel mode:
      0..1023      = one direction
      1024..2047   = opposite direction (bit10 set)
    direction: +1 or -1
    """
    speed = _clamp(int(speed), 0, 1023)
    return speed if direction >= 0 else speed + 1024


# -------------------- Config --------------------
@dataclass
class GripperConfig:
    devicename: str = "/dev/ttyUSB0"
    baudrate: int = 1_000_000
    protocol_version: float = 1.0
    dxl_id: int = 7

    pos_min: int = 30
    pos_max: int = 720

    open_pos: int = 30
    open_speed: int = 300
    open_tol: int = 5

    close_speed: int = 200
    close_dir: int = 1
    close_stop_margin: int = 15
    load_threshold: int = 80
    consecutive_hits: int = 3
    poll_dt: float = 0.02

    hold_torque_limit: int = 300
    hold_speed: int = 30

    grasp_detect_threshold: int = 50

    @classmethod
    def from_yaml(cls, yaml_path: str) -> "GripperConfig":
        """Load config from a YAML file. Only keys matching dataclass fields are used."""
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)
        valid_keys = {field.name for field in fields(cls)}
        filtered = {k: v for k, v in data.items() if k in valid_keys}
        return cls(**filtered)


# -------------------- Controller --------------------
class AX18AGripper:
    def __init__(self, cfg: GripperConfig):
        self.cfg = cfg
        self.port = PortHandler(cfg.devicename)
        self.ph = PacketHandler(cfg.protocol_version)
        self.connected = False

    # ---------- Connection ----------
    def connect(self):
        if not self.port.openPort():
            raise RuntimeError(f"Failed to open port {self.cfg.devicename}")
        if not self.port.setBaudRate(self.cfg.baudrate):
            raise RuntimeError(f"Failed to set baudrate {self.cfg.baudrate}")
        self.connected = True

    def close_port(self):
        if self.connected:
            self.port.closePort()
            self.connected = False

    # ---------- Low-level IO ----------
    def write_u8(self, addr, val, ctx="WRITE"):
        r, e = self.ph.write1ByteTxRx(self.port, self.cfg.dxl_id, addr, int(val))
        _dxl_ok(r, e, self.ph, ctx)

    def write_u16(self, addr, val, ctx="WRITE"):
        r, e = self.ph.write2ByteTxRx(self.port, self.cfg.dxl_id, addr, int(val))
        _dxl_ok(r, e, self.ph, ctx)

    def read_u16(self, addr, ctx="READ"):
        v, r, e = self.ph.read2ByteTxRx(self.port, self.cfg.dxl_id, addr)
        _dxl_ok(r, e, self.ph, ctx)
        return int(v)

    # ---------- Mode switching ----------
    def set_joint_mode(self):
        self.write_u16(ADDR_CW_ANGLE_LIMIT, self.cfg.pos_min, "SET CW Angle Limit")
        self.write_u16(ADDR_CCW_ANGLE_LIMIT, self.cfg.pos_max, "SET CCW Angle Limit")

    def set_wheel_mode(self):
        self.write_u16(ADDR_CW_ANGLE_LIMIT, 0, "SET CW Angle Limit (wheel)")
        self.write_u16(ADDR_CCW_ANGLE_LIMIT, 0, "SET CCW Angle Limit (wheel)")

    def enable_torque(self, enable=True):
        self.write_u8(
            ADDR_TORQUE_ENABLE,
            TORQUE_ENABLE if enable else TORQUE_DISABLE,
            "Torque Enable",
        )

    # ---------- State read ----------
    def read_state(self):
        pos = self.read_u16(ADDR_PRESENT_POSITION, "READ Present Position")
        raw_speed = self.read_u16(ADDR_PRESENT_SPEED, "READ Present Speed")
        raw_load = self.read_u16(ADDR_PRESENT_LOAD, "READ Present Load")
        vel = _decode_signed_10bit_with_dir(raw_speed)
        load = _decode_signed_10bit_with_dir(raw_load)
        return pos, vel, load

    # ---------- PUBLIC API ----------

    def get_data(self):
        """Returns dict with position, velocity, and torque of the gripper."""
        pos, vel, load = self.read_state()
        return {"position": pos, "velocity": vel, "torque": load}

    def is_grasping(self, torque_threshold=None):
        """Returns True if the gripper is currently holding an object.

        Detects grasping by checking if the load opposes the closing direction
        with magnitude above threshold.
        """
        threshold = torque_threshold if torque_threshold is not None else self.cfg.grasp_detect_threshold
        raw_load = self.read_u16(ADDR_PRESENT_LOAD, "READ Present Load")
        signed_load = _decode_signed_10bit_with_dir(raw_load)
        # Load opposing close_dir means object is pushing back
        opposing_load = -signed_load if self.cfg.close_dir >= 0 else signed_load
        return opposing_load >= threshold

    def open_gripper(self, target_pos=None):
        """Move gripper to open position. Blocks until position is reached."""
        goal = self.cfg.open_pos if target_pos is None else int(target_pos)
        goal = _clamp(goal, self.cfg.pos_min, self.cfg.pos_max)

        # Disable torque first so mode/speed changes don't cause uncontrolled motion
        self.enable_torque(False)
        self.set_joint_mode()
        self.write_u16(ADDR_MOVING_SPEED, _clamp(self.cfg.open_speed, 0, 1023), "SET Moving Speed (open)")
        self.write_u16(ADDR_TORQUE_LIMIT, 1023, "SET Torque Limit (open)")
        self.write_u16(ADDR_GOAL_POSITION, goal, "SET Goal Position (open)")
        # Enable torque last — motor starts moving only after all registers are configured
        self.enable_torque(True)

        while True:
            pos = self.read_u16(ADDR_PRESENT_POSITION, "READ Present Position")
            if abs(pos - goal) <= self.cfg.open_tol:
                return
            time.sleep(0.05)

    def close_gripper(self):
        """
        Wheel-mode closing until:
          (a) load >= threshold for N consecutive samples (contact detected)
              => switch to position control, hold with torque. Returns True.
          (b) position reaches close limit (no object)
              => just stop. Returns False.
        """
        self.set_wheel_mode()
        self.write_u16(ADDR_TORQUE_LIMIT, 1023, "SET Torque Limit (close approach)")
        self.enable_torque(True)

        mv = _wheel_speed_value(self.cfg.close_speed, self.cfg.close_dir)
        self.write_u16(ADDR_MOVING_SPEED, mv, "SET Moving Speed (close)")

        hits = 0
        contact_detected = False
        stop_at = self.cfg.pos_max - max(0, int(self.cfg.close_stop_margin))

        while True:
            pos = self.read_u16(ADDR_PRESENT_POSITION, "READ Present Position")
            print(f"State during close: {self.get_data()}")

            # Position limit reached — no object encountered
            if self.cfg.close_dir >= 0 and pos >= stop_at:
                print(f"Close limit reached: pos={pos} >= stop_at={stop_at}. No object.")
                break
            if self.cfg.close_dir < 0 and pos <= (self.cfg.pos_min + self.cfg.close_stop_margin):
                print("Close limit reached (min). No object.")
                break

            # Check load for contact — object pushes back against closing direction
            raw_load = self.read_u16(ADDR_PRESENT_LOAD, "READ Present Load")
            signed_load = _decode_signed_10bit_with_dir(raw_load)
            # Opposing load: positive when object resists closing
            opposing_load = -signed_load if self.cfg.close_dir >= 0 else signed_load

            if opposing_load >= self.cfg.load_threshold:
                hits += 1
            else:
                hits = 0

            if hits >= self.cfg.consecutive_hits:
                print("Contact detected (load threshold).")
                contact_detected = True
                break

            time.sleep(self.cfg.poll_dt)

        if contact_detected:
            # Stay in wheel mode — keep pushing against the object with limited
            # torque. No mode switch needed, so no stale-goal-position jerk.
            self.write_u16(ADDR_TORQUE_LIMIT, _clamp(self.cfg.hold_torque_limit, 0, 1023), "SET Torque Limit (hold)")
            mv = _wheel_speed_value(self.cfg.hold_speed, self.cfg.close_dir)
            self.write_u16(ADDR_MOVING_SPEED, mv, "SET Moving Speed (hold)")
        else:
            # No object — stop and disable torque so gripper is backdrivable
            self.write_u16(ADDR_MOVING_SPEED, 0, "STOP Moving Speed")
            self.enable_torque(False)

        return contact_detected

    # ---------- RELEASE ----------
    def release(self):
        """Stop motion and disable torque (backdrivable)."""
        self.write_u16(ADDR_MOVING_SPEED, 0, "STOP Moving Speed (release)")
        self.enable_torque(False)


# -------------------- Example usage --------------------
if __name__ == "__main__":
    # Use config path from command line arg, or fall back to default
    config_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CONFIG_PATH
    config_path = os.path.abspath(config_path)

    print(f"Loading config from: {config_path}")
    cfg = GripperConfig.from_yaml(config_path)
    print(f"Config: {cfg}")

    g = AX18AGripper(cfg)

    try:
        g.connect()

        print("Closing...")
        contact = g.close_gripper()
        print(f"Contact detected: {contact}")
        time.sleep(0.3)
        print(f"State after close: {g.get_data()}")
        print(f"Is grasping: {g.is_grasping()}")

        if contact:
            print("Holding. Press ENTER to release and open...")
            input()
        
        print("Opening...")
        g.open_gripper()
        time.sleep(0.3)
        print(f"State after open: {g.get_data()}")

        print("Done.")

    finally:
        try:
            g.release()
        except Exception:
            pass
        g.close_port()
