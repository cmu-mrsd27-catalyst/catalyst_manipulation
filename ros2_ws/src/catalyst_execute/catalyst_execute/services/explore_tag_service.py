#!/usr/bin/env python3
"""ROS 2 service node exposing AprilTag exploration via /explore_tag.

Wraps the TagExplorer class as a JsonCommand service. Sweeps joint1
through a range while checking /world_model for a target AprilTag.

Usage:
  ros2 run catalyst_execute explore_tag_service
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from catalyst_interfaces.srv import JsonCommand
from catalyst_execute.exploration import TagExplorer

JOINT_SERVICE = '/joint_command'
EXPLORE_TAG_SERVICE = '/explore_tag'


class ExploreTagService(Node):
    def __init__(self):
        super().__init__('explore_tag_service')

        # World model subscription
        self._world_model = None
        self._world_model_ts = None
        self.create_subscription(String, '/world_model', self._world_model_cb, 10)

        # Joint command client (used by TagExplorer)
        self._joint_client = self.create_client(JsonCommand, JOINT_SERVICE)

        self.create_service(JsonCommand, EXPLORE_TAG_SERVICE, self._handle_request)
        self.get_logger().info(f'Explore tag service ready on {EXPLORE_TAG_SERVICE}')

    def _world_model_cb(self, msg: String):
        data = json.loads(msg.data)
        self._world_model_ts = data.get('timestamp')
        self._world_model = data

    def _get_world_model(self):
        return (self._world_model_ts, self._world_model)

    def _handle_request(self, request, response):
        try:
            cmd = json.loads(request.command)
        except json.JSONDecodeError as e:
            response.response = json.dumps({
                'success': False, 'message': f'Invalid JSON: {e}'
            })
            return response

        # Wait for joint command service
        if not self._joint_client.wait_for_service(timeout_sec=5.0):
            response.response = json.dumps({
                'success': False, 'message': f'{JOINT_SERVICE} service not available'
            })
            return response

        tag_frame = cmd.get('tag_frame', 'tag_liquid_handler')
        sweep_start = cmd.get('sweep_start')
        sweep_end = cmd.get('sweep_end')
        sweep_step = cmd.get('sweep_step')
        move_speed = cmd.get('move_speed', 0.3)

        kwargs = {
            'tag_frame': tag_frame,
            'move_speed': move_speed,
        }
        if sweep_start is not None:
            kwargs['sweep_start'] = sweep_start
        if sweep_end is not None:
            kwargs['sweep_end'] = sweep_end
        if sweep_step is not None:
            kwargs['sweep_step'] = sweep_step

        explorer = TagExplorer(
            node=self,
            joint_client=self._joint_client,
            world_model_getter=self._get_world_model,
            **kwargs,
        )

        tag_pose, j1_angle = explorer.search()

        if tag_pose is not None:
            response.response = json.dumps({
                'success': True,
                'message': f'Tag found at j1={j1_angle:.1f} deg',
                'tag_pose': tag_pose,
                'j1_angle': j1_angle,
            })
        else:
            response.response = json.dumps({
                'success': False,
                'message': f'Tag "{tag_frame}" not found in sweep range',
            })

        return response


def main():
    rclpy.init()
    node = ExploreTagService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
