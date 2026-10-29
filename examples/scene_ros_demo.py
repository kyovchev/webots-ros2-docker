#!/usr/bin/env python3
"""Demo for the ROS 2 side of the scene manipulator (services + topic).

Run inside the container with the workspace sourced:

    python3 scene_ros_demo.py TARGET_BOX

Reads the pose via the get service, moves the object via the set service,
spins it via the topic, then restores the original pose.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from webots_simulation_interfaces.srv import GetObjectPose, SetObjectPose


class SceneDemo(Node):
    def __init__(self):
        super().__init__('scene_demo')
        self.pub = self.create_publisher(PoseStamped, '/scene/object_pose_cmd', 10)
        self.get_cli = self.create_client(GetObjectPose, '/scene/get_object_pose')
        self.set_cli = self.create_client(SetObjectPose, '/scene/set_object_pose')

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            raise RuntimeError('service call timed out (is the simulation running?)')
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        return response

    def get_pose(self, name):
        return self._call(self.get_cli, GetObjectPose.Request(name=name)).pose

    def set_pose(self, name, pose):
        self._call(self.set_cli, SetObjectPose.Request(name=name, pose=pose))


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else 'TARGET_BOX'
    rclpy.init()
    node = SceneDemo()
    start = None
    try:
        for client in (node.get_cli, node.set_cli):
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f'{client.srv_name} is not available')

        start = node.get_pose(name)
        p = start.position
        print(f'{name} starts at ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')

        # 1. Service: move 0.3 m along +x. The orientation is left all-zero,
        #    which means "keep the current orientation".
        target = Pose()
        target.position.x, target.position.y, target.position.z = p.x + 0.3, p.y, p.z
        node.set_pose(name, target)
        print('Moved +0.3 m in x via the service')
        time.sleep(1.0)

        # 2. Topic: fire-and-forget stream of poses, spinning about z.
        #    The DEF name goes in header.frame_id.
        deadline = time.monotonic() + 5.0
        while node.pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.1)

        msg = PoseStamped()
        msg.header.frame_id = name
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p.x + 0.3, p.y, p.z
        print('Spinning via the topic for 4 s ...')
        t0 = time.monotonic()
        while (t := time.monotonic() - t0) < 4.0:
            yaw = 2 * math.pi * t / 4.0
            msg.pose.orientation.z = math.sin(yaw / 2)
            msg.pose.orientation.w = math.cos(yaw / 2)
            node.pub.publish(msg)
            time.sleep(0.05)
    finally:
        if start is not None:
            try:
                node.set_pose(name, start)
                print('Restored the original pose.')
            except RuntimeError as exc:
                print(f'Could not restore the pose: {exc}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except RuntimeError as exc:
        sys.exit(str(exc))
