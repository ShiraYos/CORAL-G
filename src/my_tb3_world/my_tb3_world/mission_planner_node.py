#!/usr/bin/env python3

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_msgs.msg import String


def yaw_to_quaternion(yaw):
    half_yaw = yaw * 0.5
    return {'z': math.sin(half_yaw), 'w': math.cos(half_yaw)}


def _finite_float(value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(value)
    return number


class MissionPlannerNode(BasicNavigator):
    def __init__(self):
        super().__init__(node_name='mission_planner_node')

        self.declare_parameter('goal_timeout_sec', 120.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('localizer', 'slam_toolbox')  # 'amcl' when using pre-built map
        self.declare_parameter('collection_radius_m', 0.2)
        self.declare_parameter('default_collection_material', 'unknown')

        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.collection_radius_m = float(self.get_parameter('collection_radius_m').value)
        self.default_collection_material = self.get_parameter(
            'default_collection_material'
        ).value
        localizer = self.get_parameter('localizer').value
        self.active_goal_started_at = None
        self.goal_handle = None
        self._pending_goal = None  # (mode, x, y, yaw) — queued while cancel is in flight
        self._active_goal_intent = None

        self.twin_state = None

        self.create_subscription(String, '/next_cell_goal', self._goal_cb, 10)
        self.create_subscription(String, '/twin_state', self._twin_state_cb, 10)
        self.collection_pub = self.create_publisher(String, '/collection_event', 10)

        self._initial_pose = self._make_pose(
            float(self.get_parameter('initial_x').value),
            float(self.get_parameter('initial_y').value),
            float(self.get_parameter('initial_yaw').value),
        )
        # Send once immediately (may be dropped if AMCL isn't up yet)
        self.setInitialPose(self._initial_pose)
        self.get_logger().info(f'Waiting for Nav2 (localizer={localizer})...')
        self.waitUntilNav2Active(localizer=localizer)
        # Re-send after Nav2 is confirmed active so AMCL definitely receives it
        self.setInitialPose(self._initial_pose)
        self.get_logger().info('Nav2 active — initial pose set, ready for goals')

        # Timer only checks timeout — no BasicNavigator spinning methods called
        self.create_timer(0.5, self._check_timeout)

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _twin_state_cb(self, msg: String):
        try:
            self.twin_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Bad JSON on /twin_state')

    def _goal_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Bad JSON on /next_cell_goal')
            return

        status = data.get('status', '')
        mode = data.get('mode', '')

        if status == 'idle' or mode == 'idle':
            return

        if mode == 'return_to_base':
            try:
                bx, by, byaw = self._base_from_twin()
            except (TypeError, ValueError):
                self.get_logger().error('Invalid base pose in /twin_state')
                return
            self.get_logger().info(f'Return to base: ({bx}, {by})')
        elif mode == 'cleanup':
            goal = data.get('goal', {})
            if goal.get('frame_id') != 'map':
                self.get_logger().error('Ignoring non-map /next_cell_goal')
                return
            try:
                bx = _finite_float(goal['x'])
                by = _finite_float(goal['y'])
                byaw = _finite_float(goal.get('yaw', 0.0))
            except (KeyError, TypeError, ValueError):
                self.get_logger().error('Invalid cleanup goal in /next_cell_goal')
                return
            waste_type = data.get('waste_type', 'unknown')
            self.get_logger().info(
                f'Cleanup goal: ({bx}, {by}) type={waste_type} '
                f'utility={data.get("utility", "?")}'
            )
        else:
            self.get_logger().warn(f'Unknown mode: {mode!r} — ignoring')
            return

        self._navigate(mode, bx, by, byaw)

    # ── Navigation (fully async — no spin_until_future_complete) ───────────────

    def _base_from_twin(self):
        if self.twin_state:
            base = self.twin_state.get('base', {})
            pose = base.get('pose', {})
            return (
                _finite_float(pose.get('x', 0.0)),
                _finite_float(pose.get('y', 0.0)),
                _finite_float(pose.get('yaw', 0.0)),
            )
        return 0.0, 0.0, 0.0

    def _navigate(self, mode, x, y, yaw):
        """Send a goal only when Nav2 is not already executing one."""
        if self.goal_handle is not None or self.active_goal_started_at is not None:
            self.get_logger().info(
                f'Ignoring {mode} goal while Nav2 goal is active'
            )
            return

        self._pending_goal = (mode, x, y, yaw)

        self._send_pending_goal()

    def _on_cancel_done(self):
        """Called after Nav2 confirms the cancel — now safe to send the new goal."""
        self.get_logger().info('Previous goal canceled — sending pending goal')
        self._send_pending_goal()

    def _send_pending_goal(self):
        if self._pending_goal is None:
            return
        mode, x, y, yaw = self._pending_goal
        self._pending_goal = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose(x, y, yaw)

        send_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=None,  # no feedback needed
        )
        send_future.add_done_callback(lambda f: self._on_goal_accepted(f, mode, x, y))
        self.get_logger().info(f'Sending goal: mode={mode} target=({x:.2f}, {y:.2f})')

    def _on_goal_accepted(self, future, mode, x, y):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected by Nav2: ({x:.2f}, {y:.2f})')
            self.active_goal_started_at = None
            self._active_goal_intent = None
            return
        self.goal_handle = goal_handle
        self._active_goal_intent = {'mode': mode, 'x': x, 'y': y}
        self.active_goal_started_at = time.monotonic()  # start timeout only after acceptance
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)
        self.get_logger().info(f'Goal accepted — navigating to ({x:.2f}, {y:.2f})')

    def _on_nav_result(self, future):
        self.goal_handle = None
        self.active_goal_started_at = None
        intent = self._active_goal_intent
        self._active_goal_intent = None
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded')
            if intent and intent.get('mode') == 'cleanup':
                self._publish_collection_event(intent)
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal canceled')
        else:
            self.get_logger().warn(f'Goal failed (status={result.status})')

    def _publish_collection_event(self, intent):
        material = str(self.default_collection_material or 'unknown')
        event = {
            'schema': 'dtas.collection_event.v1',
            'stamp': self._stamp(),
            'source': 'mission_planner_node',
            'location': {
                'x': round(float(intent['x']), 3),
                'y': round(float(intent['y']), 3),
            },
            'count': 1,
            'items': [{'type': material, 'count': 1}],
            'materials': {material: 1},
            'collection_radius_m': self.collection_radius_m,
            'status': 'collected',
        }
        msg = String()
        msg.data = json.dumps(event)
        self.collection_pub.publish(msg)
        self.get_logger().info(
            f'Collection event after Nav2 success at '
            f'({event["location"]["x"]:.2f}, {event["location"]["y"]:.2f})'
        )

    def _check_timeout(self):
        if self.active_goal_started_at is None or self.goal_handle is None:
            return
        if time.monotonic() - self.active_goal_started_at > self.goal_timeout_sec:
            self.get_logger().warn('Goal timed out — canceling')
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: self.get_logger().info('Timeout cancel confirmed'))
            self.goal_handle = None
            self.active_goal_started_at = None
            self._active_goal_intent = None

    def _make_pose(self, x, y, yaw) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        q = yaw_to_quaternion(yaw)
        pose.pose.orientation.z = q['z']
        pose.pose.orientation.w = q['w']
        return pose

    def _stamp(self) -> str:
        t = self.get_clock().now().to_msg()
        return f'{t.sec}.{t.nanosec:09d}'


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
