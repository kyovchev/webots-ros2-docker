#!/usr/bin/env python3
"""
Pick-and-place action server.

Action  : my_robot_pick_and_place/action/PickAndPlace  (empty goal – just a trigger)
Subs    : /detected_object       (geometry_msgs/PoseStamped) – latest object pose
          /is_moving             (std_msgs/Bool)              – Commander motion state
Pubs    : /vacuum_cum/turn_on    (std_msgs/Bool)              – vacuum gripper
          /status_pick_and_place (std_msgs/String)            – human-readable status
          /pose_command          (my_robot_interfaces/PoseCommand) – to Commander

Node parameters (set via YAML or CLI):
  hover_height                  – metres above pick/place Z    (default 0.04)
  detected_timeout              – seconds to wait for detection (default 5.0)
  vacuum_delay                  – seconds after gripper toggle  (default 0.5)
  move_timeout                  – seconds to wait for motion    (default 15.0)

Example launch override:
  ros2 run my_robot_pick_and_place pick_and_place_server.py \
    --ros-args -p place_pose.x:=0.5 -p place_pose.y:=-0.1 -p hover_height:=0.15
"""

import copy
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from my_robot_interfaces.msg import PoseCommand

from my_robot_interfaces.action import PickAndPlace


# ── status helper ─────────────────────────────────────────────────────────────

def _status(node: "PickAndPlaceServer", text: str, goal_handle=None) -> None:
    """Log, publish to /status_pick_and_place, and optionally send action feedback."""
    node.get_logger().info(text)
    node.status_pub_.publish(String(data=text))
    if goal_handle is not None:
        fb = PickAndPlace.Feedback()
        fb.status = text
        goal_handle.publish_feedback(fb)


# ── server node ───────────────────────────────────────────────────────────────

class PickAndPlaceServer(Node):

    def __init__(self):
        super().__init__("pick_and_place_server")

        # ── parameters ────────────────────────────────────────────────
        self.declare_parameter("hover_height",     0.04)
        self.declare_parameter("detected_timeout", 5.0)
        self.declare_parameter("vacuum_delay",     0.5)
        self.declare_parameter("move_timeout",     15.0)

        # ── state ─────────────────────────────────────────────────────
        self.detected_object_: PoseStamped | None = None
        self.is_moving_: bool = False

        cb = ReentrantCallbackGroup()

        # ── subscribers ───────────────────────────────────────────────
        self.create_subscription(
            PoseStamped, "/detected_object",
            self._detected_object_cb, 10, callback_group=cb,
        )
        self.create_subscription(
            Bool, "/is_moving",
            self._is_moving_cb, 10, callback_group=cb,
        )

        # ── publishers ────────────────────────────────────────────────
        self.vacuum_pub_   = self.create_publisher(Bool,        "/my_robot/vacuum_cup/turn_on", 10)
        self.status_pub_   = self.create_publisher(String,      "/status_pick_and_place", 10)
        self.pose_cmd_pub_ = self.create_publisher(PoseCommand, "/pose_command", 10)

        # ── action server ─────────────────────────────────────────────
        self._action_server = ActionServer(
            self,
            PickAndPlace,
            "pick_and_place",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=cb,
        )

        _status(self, "Pick-and-place server ready.")

    # ── topic callbacks ───────────────────────────────────────────────────────

    def _detected_object_cb(self, msg: PoseStamped) -> None:
        self.detected_object_ = msg

    def _is_moving_cb(self, msg: Bool) -> None:
        self.is_moving_ = msg.data

    # ── action callbacks ──────────────────────────────────────────────────────

    def _goal_cb(self, _goal_request):
        self.get_logger().info("Pick-and-place goal received.")
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle):
        self.get_logger().info("Cancel requested.")
        return CancelResponse.ACCEPT

    # ── main execute ──────────────────────────────────────────────────────────

    def _execute_cb(self, goal_handle):
        try:
            # Read parameters fresh on every execution so they can be changed
            # at runtime without restarting the node.
            place_pose   = self._read_place_pose()
            hover_height = self.get_parameter("hover_height").value
            det_timeout  = self.get_parameter("detected_timeout").value

            # 1. Wait for a valid object detection ─────────────────────────
            _status(self, "Waiting for object detection…", goal_handle)
            deadline = time.time() + det_timeout
            while self.detected_object_ is None:
                self._check_cancel(goal_handle)
                if time.time() > deadline:
                    raise RuntimeError(
                        f"No object detected on /detected_object "
                        f"within {det_timeout:.1f} s."
                    )
                time.sleep(0.05)

            pick_pose: PoseStamped = self.detected_object_
            _status(
                self,
                f"Object detected at "
                f"({pick_pose.pose.position.x:.3f}, "
                f"{pick_pose.pose.position.y:.3f}, "
                f"{pick_pose.pose.position.z:.3f}).",
                goal_handle,
            )

            pick_pose = self._pose_to_cmd(pick_pose)

            hover_pick  = self._offset_z(pick_pose,  hover_height)
            hover_place = self._offset_z(place_pose, hover_height)

            # 2. Ensure gripper is off ─────────────────────────────────────
            self._set_vacuum(False)

            # 3. Hover above pick ──────────────────────────────────────────
            _status(self, "Moving to hover above pick pose…", goal_handle)
            self._check_cancel(goal_handle)
            self._move_and_wait(hover_pick, goal_handle)

            # 4. Descend to pick ───────────────────────────────────────────
            _status(self, "Descending to pick pose…", goal_handle)
            self._check_cancel(goal_handle)
            self._move_and_wait(pick_pose, goal_handle)

            # 5. Activate vacuum ───────────────────────────────────────────
            _status(self, "Activating vacuum gripper…", goal_handle)
            self._set_vacuum(True)

            # 6. Retreat to hover ──────────────────────────────────────────
            _status(self, "Retreating from pick pose…", goal_handle)
            self._check_cancel(goal_handle)
            self._move_and_wait(hover_pick, goal_handle)

            # 7. Hover above place ─────────────────────────────────────────
            _status(self, "Moving to hover above place pose…", goal_handle)
            self._check_cancel(goal_handle)
            self._move_and_wait(hover_place, goal_handle)

            # 8. Descend to place ──────────────────────────────────────────
            _status(self, "Descending to place pose…", goal_handle)
            self._check_cancel(goal_handle)
            self._move_and_wait(place_pose, goal_handle)
            time.sleep(1)

            # 9. Release vacuum ────────────────────────────────────────────
            _status(self, "Releasing vacuum gripper…", goal_handle)
            self._set_vacuum(False)

            # 10. Retreat to hover ─────────────────────────────────────────
            _status(self, "Retreating from place pose…", goal_handle)
            self._move_and_wait(hover_place, goal_handle)

            _status(self, "Pick and place complete.", goal_handle)
            goal_handle.succeed()
            return PickAndPlace.Result(success=True, message="Success.")

        except InterruptedError as exc:
            _status(self, f"Cancelled: {exc}")
            self._set_vacuum(False)
            goal_handle.canceled()
            return PickAndPlace.Result(success=False, message=str(exc))

        # except Exception as exc:
        #     _status(self, f"Error: {exc}")
        #     self._set_vacuum(False)
        #     goal_handle.abort()
        #     return PickAndPlace.Result(success=False, message=str(exc))

    # ── motion helpers ────────────────────────────────────────────────────────

    def _move_and_wait(self, pose: PoseCommand, goal_handle) -> None:
        """Publish a PoseCommand, then block until is_moving goes False."""
        self.pose_cmd_pub_.publish(pose)

        move_timeout = self.get_parameter("move_timeout").value

        # Give Commander up to 1 s to start moving before we check for completion.
        deadline_start = time.time() + 3.0
        while not self.is_moving_ and time.time() < deadline_start:
            self._check_cancel(goal_handle)
            time.sleep(0.02)

        # Wait for motion to finish.
        deadline_done = time.time() + move_timeout
        while self.is_moving_:
            self._check_cancel(goal_handle)
            if time.time() > deadline_done:
                raise RuntimeError(
                    f"Motion did not finish within {move_timeout:.1f} s."
                )
            time.sleep(0.05)

    def _set_vacuum(self, on: bool) -> None:
        self.vacuum_pub_.publish(Bool(data=on))
        time.sleep(self.get_parameter("vacuum_delay").value)

    # ── utilities ─────────────────────────────────────────────────────────────

    def _read_place_pose(self) -> PoseCommand:
        """Build a PoseCommand from the node's place_pose.* parameters."""
        cmd = PoseCommand()
        cmd.x              = 0.0
        cmd.y              = 0.12
        cmd.z              = 0.003
        cmd.roll           = 3.14
        cmd.pitch          = 0.0
        cmd.yaw            = 0.0
        cmd.cartesian_path = False
        return cmd

    @staticmethod
    def _pose_to_cmd(pose: PoseStamped) -> PoseCommand:
        """Convert a PoseStamped to the PoseCommand that Commander expects."""
        cmd = PoseCommand()
        cmd.x              = pose.pose.position.x
        cmd.y              = pose.pose.position.y
        cmd.z              = pose.pose.position.z - 0.004
        cmd.roll           = 3.14
        cmd.pitch          = 0.0
        cmd.yaw            = 0.0
        cmd.cartesian_path = False
        return cmd

    @staticmethod
    def _offset_z(pose: PoseCommand, dz: float) -> PoseCommand:
        p = copy.deepcopy(pose)
        p.z += dz
        return p

    @staticmethod
    def _check_cancel(goal_handle) -> None:
        if goal_handle.is_cancel_requested:
            raise InterruptedError("Goal cancelled by client.")


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    server = PickAndPlaceServer()
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    try:
        executor.spin()
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()