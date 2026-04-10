#!/usr/bin/env python3
"""ROS 2 service node exposing xArm SDK primitives via /sdk_control.

Provides low-level SDK operations that bypass ros2_control:
  - connect: Connect to xArm SDK + OnRobot F/T sensor
  - velocity_control: Cartesian velocity mode with F/T thresholds
    (simple admittance, corner registration, or force descent)
  - position_move: SDK position mode move with verification loop
  - bias_ft: Re-bias the F/T sensor
  - cleanup: Stop velocity, disconnect SDK, close F/T reader
  - restart_controllers: Kill and restart ros2_control_node

The node maintains persistent SDK and F/T sensor connections across calls.
Typical call sequence: connect → velocity_control → position_move → cleanup → restart_controllers

Usage:
  ros2 run catalyst_execute sdk_control_service --ros-args \
    -p robot_ip:=192.168.1.212 -p ft_sensor_ip:=192.168.2.1
"""

import json
import os
import time

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

from catalyst_interfaces.srv import JsonCommand
from xarm.wrapper import XArmAPI

from catalyst_execute.sdk_admittance import OnRobotFTReader
from catalyst_execute.utils.execute_config import section
from catalyst_execute.utils.ros2_control_manager import restart_ros2_control

# xArm mode/state constants
XARM_MODE_POSITION = 0
XARM_MODE_CART_VELOCITY = 5
XARM_STATE_START = 0
XARM_STATE_STOP = 4


class SdkControlService(Node):
    def __init__(self):
        super().__init__('sdk_control_service')

        _cfg = section('sdk_control_service')
        self.declare_parameter(
            'robot_ip', _cfg.get('robot_ip', '192.168.1.212'))
        self.declare_parameter(
            'ft_sensor_ip', _cfg.get('ft_sensor_ip', '192.168.2.1'))
        self.declare_parameter('rate', float(_cfg.get('rate', 100.0)))

        self._sdk_service_name = _cfg.get('service_name', '/sdk_control')

        self._arm = None
        self._ft_reader = None

        self._cb_group = ReentrantCallbackGroup()
        # One client for the whole process — restart_ros2_control uses it each place.
        self._robot_description_client = self.create_client(
            GetParameters, '/robot_state_publisher/get_parameters',
            callback_group=self._cb_group,
        )
        self.create_service(
            JsonCommand, self._sdk_service_name, self._handle_request,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            f'SDK control service ready on {self._sdk_service_name}')

    # ── Request dispatcher ──

    def _handle_request(self, request, response):
        try:
            cmd = json.loads(request.command)
        except json.JSONDecodeError as e:
            response.response = json.dumps({
                'success': False, 'message': f'Invalid JSON: {e}'
            })
            return response

        action = cmd.get('action', '')
        self.get_logger().info(f'SDK control action: {action}')

        handlers = {
            'connect': self._handle_connect,
            'velocity_control': self._handle_velocity_control,
            'position_move': self._handle_position_move,
            'bias_ft': self._handle_bias_ft,
            'cleanup': self._handle_cleanup,
            'restart_controllers': self._handle_restart_controllers,
            'get_position': self._handle_get_position,
        }

        handler = handlers.get(action)
        if handler is None:
            response.response = json.dumps({
                'success': False,
                'message': f'Unknown action: {action}. '
                           f'Valid: {", ".join(handlers.keys())}'
            })
            return response

        result = handler(cmd)
        response.response = json.dumps(result)
        return response

    # ── Action handlers ──

    def _handle_connect(self, cmd):
        """Connect to xArm SDK and F/T sensor. Enter velocity mode.

        Params:
            robot_ip (optional): override node parameter
            ft_sensor_ip (optional): override node parameter
        """
        if self._arm is not None or self._ft_reader is not None:
            self.get_logger().warn(
                'Stale SDK/F-T session before connect — running cleanup first')
            self._handle_cleanup({})

        robot_ip = cmd.get('robot_ip', self.get_parameter('robot_ip').value)
        ft_ip = cmd.get('ft_sensor_ip', self.get_parameter('ft_sensor_ip').value)

        # Connect F/T sensor
        try:
            self.get_logger().info(f'Connecting to F/T sensor at {ft_ip}...')
            self._ft_reader = OnRobotFTReader(ft_ip)
            self._ft_reader.start()
            self._ft_reader.bias()
            self.get_logger().info('F/T sensor connected and biased.')
        except Exception as e:
            return {'success': False, 'message': f'F/T sensor connection failed: {e}'}

        # Connect xArm SDK
        try:
            self.get_logger().info(f'Connecting to xArm at {robot_ip}...')
            self._arm = XArmAPI(robot_ip, protocol=3)
            time.sleep(0.5)
            if self._arm.error_code != 0:
                self._arm.clean_error()
            if self._arm.warn_code != 0:
                self._arm.clean_warn()
            self._arm.motion_enable(True)
            self._arm.set_collision_sensitivity(0)
            self._arm.set_mode(XARM_MODE_CART_VELOCITY)
            self._arm.set_state(XARM_STATE_START)
            time.sleep(0.5)
        except Exception as e:
            return {'success': False, 'message': f'xArm connection failed: {e}'}

        code, pose = self._arm.get_position()
        if code != 0:
            return {'success': False, 'message': f'Cannot read arm position (code={code})'}

        self.get_logger().info(
            f'Connected. Mode={self._arm.mode}, '
            f'Pos: X={pose[0]:.1f}, Y={pose[1]:.1f}, Z={pose[2]:.1f} mm'
        )
        return {
            'success': True,
            'message': 'SDK and F/T sensor connected',
            'position': pose[:6],
        }

    def _handle_velocity_control(self, cmd):
        """Run cartesian velocity control loop with F/T feedback.

        Params:
            mode: "simple_admittance" | "corner_registration" | "force_descent"
            rate (optional): control loop Hz (default: node param)
            log_path (optional): CSV path for F/T logging

        Mode "simple_admittance" params:
            ref_velocity: m/s, descent speed in tool -X (default: -0.005)
            force_threshold: N, stop when |Fx| exceeds (default: 5.0)
            max_distance: m, safety limit (default: 0.1)
            linear_gain: admittance gain (default: 0.005)
            force_deadzone: N (default: 1.0)
            compliant_axes: [x,y,z,rx,ry,rz] mask (default: [0,1,1,0,0,0])
            max_linear_vel: m/s (default: 0.05)
            timeout_sec: optional wall-clock limit (loop exits, success false)

        Mode "corner_registration" params:
            ref_velocity: m/s, descent speed (default: -0.005)
            contact_force_threshold: N, |Fx| to detect surface (default: 1.0)
            corner_force_threshold: N, |Fy|/|Fz| to detect corner (default: 1.0)
            search_speed: m/s, lateral search speed (default: 0.003)
            max_distance: m (default: 0.1)

        Mode "force_descent" params:
            descent_speed: mm/s, tool -X velocity (default: -5.0)
            force_threshold: N, stop when |Fx| exceeds (default: 5.0)
            rebias: bool, re-bias F/T before descent (default: true)
        """
        if self._arm is None or self._ft_reader is None:
            return {'success': False, 'message': 'Not connected. Call "connect" first.'}

        # Ensure we're in velocity mode
        if self._arm.mode != XARM_MODE_CART_VELOCITY:
            self._arm.set_state(XARM_STATE_STOP)
            time.sleep(0.3)
            self._arm.set_mode(XARM_MODE_CART_VELOCITY)
            self._arm.set_state(XARM_STATE_START)
            time.sleep(0.3)

        mode = cmd.get('mode', 'simple_admittance')
        if mode == 'simple_admittance':
            return self._run_simple_admittance(cmd)
        elif mode == 'corner_registration':
            return self._run_corner_registration(cmd)
        elif mode == 'force_descent':
            return self._run_force_descent(cmd)
        else:
            return {'success': False, 'message': f'Unknown velocity mode: {mode}'}

    def _handle_position_move(self, cmd):
        """SDK position mode move with verification loop.

        Params:
            x, y, z: target position in mm (absolute, base frame)
            roll, pitch, yaw: target orientation in degrees (optional, keeps current if omitted)
            speed: mm/s (default: 5)
            tolerance: mm, position verification threshold (default: 0.5)
            max_corrections: int (default: 10)
        """
        if self._arm is None:
            return {'success': False, 'message': 'Not connected. Call "connect" first.'}

        # Switch to position mode
        self._arm.set_state(XARM_STATE_STOP)
        time.sleep(0.5)
        self._arm.set_mode(XARM_MODE_POSITION)
        self._arm.set_state(XARM_STATE_START)
        time.sleep(0.5)

        # Read current pose for defaults
        code, current_pose = self._arm.get_position()
        if code != 0:
            return {'success': False, 'message': f'Cannot read position (code={code})'}

        target_x = cmd.get('x', current_pose[0])
        target_y = cmd.get('y', current_pose[1])
        target_z = cmd.get('z', current_pose[2])
        target_roll = cmd.get('roll', current_pose[3])
        target_pitch = cmd.get('pitch', current_pose[4])
        target_yaw = cmd.get('yaw', current_pose[5])
        speed = cmd.get('speed', 5)
        tolerance = cmd.get('tolerance', 0.5)
        max_corrections = cmd.get('max_corrections', 10)

        self.get_logger().info(
            f'Position move: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f}) mm, speed={speed}'
        )

        # Move
        self._arm.set_position(
            x=target_x, y=target_y, z=target_z,
            roll=target_roll, pitch=target_pitch, yaw=target_yaw,
            speed=speed, wait=True
        )
        time.sleep(0.3)

        # Verification loop
        for i in range(max_corrections):
            code_v, verify_pose = self._arm.get_position()
            if code_v != 0:
                break
            dx = target_x - verify_pose[0]
            dy = target_y - verify_pose[1]
            dz = target_z - verify_pose[2]
            self.get_logger().info(
                f'Verify {i+1}: dx={dx:+.2f}, dy={dy:+.2f}, dz={dz:+.2f} mm'
            )
            if abs(dx) < tolerance and abs(dy) < tolerance and abs(dz) < tolerance:
                self.get_logger().info('Position within tolerance.')
                return {
                    'success': True,
                    'message': 'Position reached',
                    'position': verify_pose[:6],
                }
            # Re-send
            self._arm.set_position(
                x=target_x, y=target_y, z=target_z,
                roll=target_roll, pitch=target_pitch, yaw=target_yaw,
                speed=speed, wait=True
            )
            time.sleep(0.3)

        # Read final position
        code_f, final_pose = self._arm.get_position()
        pos = final_pose[:6] if code_f == 0 else []
        return {
            'success': False,
            'message': f'Position not reached within tolerance after {max_corrections} attempts',
            'position': pos,
        }

    def _handle_bias_ft(self, cmd):
        """Re-bias the F/T sensor."""
        if self._ft_reader is None:
            return {'success': False, 'message': 'F/T sensor not connected.'}
        self._ft_reader.bias()
        time.sleep(0.3)
        return {'success': True, 'message': 'F/T sensor biased'}

    def _handle_get_position(self, cmd):
        """Read current SDK position."""
        if self._arm is None:
            return {'success': False, 'message': 'Not connected.'}
        code, pose = self._arm.get_position()
        if code != 0:
            return {'success': False, 'message': f'Read failed (code={code})'}
        return {'success': True, 'message': 'OK', 'position': pose[:6]}

    def _handle_cleanup(self, cmd):
        """Stop velocity, disconnect SDK, close F/T reader."""
        if self._arm:
            try:
                self._arm.vc_set_cartesian_velocity(
                    [0, 0, 0, 0, 0, 0], is_tool_coord=True
                )
                time.sleep(0.3)
                self._arm.set_state(XARM_STATE_STOP)
                time.sleep(0.5)
                self._arm.set_collision_sensitivity(3)
                self.get_logger().info(
                    f'SDK stopped: mode={self._arm.mode}, state={self._arm.state}'
                )
                self._arm.disconnect()
                self.get_logger().info('SDK disconnected.')
            except Exception as e:
                self.get_logger().warn(f'SDK cleanup error: {e}')
            self._arm = None

        if self._ft_reader:
            self._ft_reader.close()
            self._ft_reader = None

        return {'success': True, 'message': 'SDK and F/T sensor cleaned up'}

    def _handle_restart_controllers(self, cmd):
        """Restart ros2_control_node and re-spawn controllers."""
        success = restart_ros2_control(
            self, self.get_logger(), self._robot_description_client)
        if success:
            return {'success': True, 'message': 'Controllers restarted'}
        return {'success': False, 'message': 'Controller restart failed'}

    # ── Velocity control modes ──

    def _run_simple_admittance(self, cmd):
        """Admittance descent: push in tool -X with F/T compliance."""
        rate = cmd.get('rate', self.get_parameter('rate').value)
        dt = 1.0 / rate

        ref_vx = cmd.get('ref_velocity', -0.005)
        place_force = cmd.get('force_threshold', 5.0)
        max_dist = cmd.get('max_distance', 0.1)
        linear_gain = cmd.get('linear_gain', 0.005)
        force_dz = cmd.get('force_deadzone', 1.0)
        compliant = np.array(cmd.get('compliant_axes', [0, 1, 1, 0, 0, 0]), dtype=float)
        max_lin_vel = cmd.get('max_linear_vel', 0.05)
        timeout_sec = cmd.get('timeout_sec')

        code, start_pose = self._arm.get_position()
        if code != 0:
            return {'success': False, 'message': f'Cannot read position (code={code})'}

        self.get_logger().info(
            f'Simple admittance: ref_vx={ref_vx*1000:.1f} mm/s, '
            f'stop at {place_force:.1f} N or {max_dist*1000:.0f} mm'
        )

        cycle_count = 0
        reason = 'Interrupted'
        place_success = False
        t_loop_start = time.monotonic()

        while rclpy.ok():
            if timeout_sec is not None and (time.monotonic() - t_loop_start) > float(timeout_sec):
                reason = f'Timeout after {float(timeout_sec):.1f}s'
                break
            loop_start = time.monotonic()
            ft = self._ft_reader.get_ft()

            if abs(ft[0]) > place_force:
                reason = f'Force threshold: Fx={ft[0]:+.2f} N > {place_force} N'
                place_success = True
                break

            for i in range(3):
                if abs(ft[i]) < force_dz:
                    ft[i] = 0.0

            vel = np.zeros(6)
            vel[0:3] = linear_gain * ft[0:3]
            vel *= compliant
            vel[0] += ref_vx

            lin_speed = np.linalg.norm(vel[0:3])
            if lin_speed > max_lin_vel:
                vel[0:3] *= max_lin_vel / lin_speed

            code, actual_pose = self._arm.get_position()
            if code == 0:
                actual_travel = np.linalg.norm(
                    np.array(actual_pose[0:3]) - np.array(start_pose[0:3])
                ) / 1000.0
                if actual_travel > max_dist:
                    reason = f'Max distance: {actual_travel*1000:.1f} mm'
                    break

            vel_cmd = [vel[0]*1000, vel[1]*1000, vel[2]*1000, 0, 0, 0]
            ret = self._arm.vc_set_cartesian_velocity(vel_cmd, is_tool_coord=True, duration=-1)
            if ret != 0:
                self.get_logger().warn(f'vc_set ret={ret}')
                if self._arm.error_code != 0:
                    reason = f'xArm error: {self._arm.error_code}'
                    break

            cycle_count += 1
            if cycle_count % int(rate * 2) == 0:
                travel_mm = actual_travel * 1000.0 if code == 0 else -1
                self.get_logger().info(
                    f'F/T: [{ft[0]:+6.2f}, {ft[1]:+6.2f}, {ft[2]:+6.2f}] N  '
                    f'Travel: {travel_mm:.1f} mm'
                )

            elapsed = time.monotonic() - loop_start
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)

        self._arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0], is_tool_coord=True)
        self.get_logger().info(f'Simple admittance stopped: {reason}')

        code_f, final_pose = self._arm.get_position()
        return {
            'success': place_success,
            'message': reason,
            'position': final_pose[:6] if code_f == 0 else [],
        }

    def _run_corner_registration(self, cmd):
        """Corner registration: DESCEND → SEARCH_Y → SEARCH_Z → DONE."""
        rate = cmd.get('rate', self.get_parameter('rate').value)
        dt = 1.0 / rate

        ref_vx = cmd.get('ref_velocity', -0.005)
        max_dist = cmd.get('max_distance', 0.1)
        contact_force = cmd.get('contact_force_threshold', 1.0)
        corner_force = cmd.get('corner_force_threshold', 1.0)
        search_speed = cmd.get('search_speed', 0.003)
        log_path = cmd.get('log_path', '')

        code, start_pose = self._arm.get_position()
        if code != 0:
            return {'success': False, 'message': f'Cannot read position (code={code})'}

        phase = 'DESCEND'
        corner_pose = None

        self.get_logger().info(
            f'Corner registration: descend at {ref_vx*1000:.1f} mm/s, '
            f'contact at {contact_force:.1f} N, corner at {corner_force:.1f} N, '
            f'search speed {search_speed*1000:.1f} mm/s'
        )

        cycle_count = 0
        reason = 'Interrupted'

        log_file = None
        if log_path:
            log_file = open(log_path, 'w')
            log_file.write('time,fx,fy,fz,tx,ty,tz,vx,vy,vz,travel_mm,pos_x,pos_y,pos_z,phase\n')

        t_start = time.monotonic()

        while rclpy.ok():
            loop_start = time.monotonic()
            t_elapsed = time.monotonic() - t_start

            ft = self._ft_reader.get_ft()
            vel_cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            if phase == 'DESCEND':
                vel_cmd[0] = ref_vx * 1000  # mm/s
                if abs(ft[0]) > contact_force:
                    self.get_logger().info(
                        f'Contact detected! |Fx|={abs(ft[0]):.2f} N > {contact_force} N'
                    )
                    phase = 'SEARCH_Y'
                    self.get_logger().info('Searching left (Y) for corner...')

            elif phase == 'SEARCH_Y':
                vel_cmd[1] = -search_speed * 1000  # mm/s
                if abs(ft[1]) > corner_force:
                    self.get_logger().info(
                        f'Corner Y found! |Fy|={abs(ft[1]):.2f} N > {corner_force} N'
                    )
                    phase = 'SEARCH_Z'
                    self.get_logger().info('Searching forward (Z) for corner...')

            elif phase == 'SEARCH_Z':
                vel_cmd[2] = search_speed * 1000  # mm/s
                if abs(ft[2]) > corner_force:
                    code_c, pose_c = self._arm.get_position()
                    if code_c == 0:
                        corner_pose = pose_c
                    self.get_logger().info(
                        f'Corner Z found! |Fz|={abs(ft[2]):.2f} N > {corner_force} N'
                    )
                    if corner_pose:
                        self.get_logger().info(
                            f'Corner (mm): X={corner_pose[0]:.2f}, '
                            f'Y={corner_pose[1]:.2f}, Z={corner_pose[2]:.2f}'
                        )
                    phase = 'DONE'
                    reason = 'Corner found'
                    break

            # Check travel distance
            code, actual_pose = self._arm.get_position()
            actual_travel = 0.0
            if code == 0:
                actual_travel = np.linalg.norm(
                    np.array(actual_pose[0:3]) - np.array(start_pose[0:3])
                ) / 1000.0
                if actual_travel > max_dist:
                    reason = f'Max distance: {actual_travel*1000:.1f} mm'
                    break

            ret = self._arm.vc_set_cartesian_velocity(vel_cmd, is_tool_coord=True, duration=-1)
            if ret != 0:
                self.get_logger().warn(f'vc_set ret={ret}')
                if self._arm.error_code != 0:
                    reason = f'xArm error: {self._arm.error_code}'
                    break

            # Log F/T data
            if log_file:
                raw_ft = self._ft_reader.get_ft()
                px = actual_pose[0] if code == 0 else 0.0
                py = actual_pose[1] if code == 0 else 0.0
                pz = actual_pose[2] if code == 0 else 0.0
                log_file.write(
                    f'{t_elapsed:.4f},'
                    f'{raw_ft[0]:.4f},{raw_ft[1]:.4f},{raw_ft[2]:.4f},'
                    f'{raw_ft[3]:.5f},{raw_ft[4]:.5f},{raw_ft[5]:.5f},'
                    f'{vel_cmd[0]:.2f},{vel_cmd[1]:.2f},{vel_cmd[2]:.2f},'
                    f'{actual_travel*1000:.1f},'
                    f'{px:.2f},{py:.2f},{pz:.2f},'
                    f'{phase}\n'
                )
                log_file.flush()

            cycle_count += 1
            if cycle_count % int(rate) == 0:
                raw_ft = self._ft_reader.get_ft()
                px = actual_pose[0] if code == 0 else 0.0
                py = actual_pose[1] if code == 0 else 0.0
                pz = actual_pose[2] if code == 0 else 0.0
                self.get_logger().info(
                    f'[{phase}] F:[{raw_ft[0]:+6.2f},{raw_ft[1]:+6.2f},{raw_ft[2]:+6.2f}] N  '
                    f'Pos:({px:.1f},{py:.1f},{pz:.1f})'
                )

            elapsed = time.monotonic() - loop_start
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)

        self._arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0], is_tool_coord=True)

        if log_file:
            log_file.close()
            self.get_logger().info(f'F/T log saved to {log_path}')

        self.get_logger().info(f'Corner registration stopped: {reason}')

        result = {
            'success': corner_pose is not None,
            'message': reason,
        }
        if corner_pose:
            result['corner_pose'] = corner_pose[:6]
        return result

    def _run_force_descent(self, cmd):
        """Simple force-controlled descent in tool -X until force threshold."""
        descent_speed = cmd.get('descent_speed', -5.0)  # mm/s
        force_threshold = cmd.get('force_threshold', 5.0)  # N
        rebias = cmd.get('rebias', True)

        if rebias:
            self._ft_reader.bias()
            time.sleep(0.2)

        self.get_logger().info(
            f'Force descent: {descent_speed} mm/s, stop at |Fx| > {force_threshold} N'
        )

        while rclpy.ok():
            ft = self._ft_reader.get_ft()
            if abs(ft[0]) > force_threshold:
                self.get_logger().info(
                    f'Plate seated! |Fx|={abs(ft[0]):.2f} N > {force_threshold} N'
                )
                break
            self._arm.vc_set_cartesian_velocity(
                [descent_speed, 0, 0, 0, 0, 0], is_tool_coord=True, duration=-1
            )
            time.sleep(0.01)

        self._arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0], is_tool_coord=True)
        time.sleep(0.3)

        code, final_pose = self._arm.get_position()
        return {
            'success': True,
            'message': f'Force descent complete',
            'position': final_pose[:6] if code == 0 else [],
        }


def main():
    rclpy.init()
    node = SdkControlService()
    n_threads = max(8, (os.cpu_count() or 4) * 2)
    executor = MultiThreadedExecutor(num_threads=n_threads)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
        node._handle_cleanup({})
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
