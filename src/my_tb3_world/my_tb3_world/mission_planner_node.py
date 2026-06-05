#!/usr/bin/env python3

import json
import math
import time

_RETURN_COOLDOWN_SEC = 10.0
_ESCAPE_TIMEOUT_SEC = 10.0
_COMPUTE_PATH_START_OCCUPIED = 205  # nav2_msgs ComputePathToPose.START_OCCUPIED

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
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

        self.declare_parameter('goal_timeout_sec', 60.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('localizer', 'slam_toolbox')  # 'amcl' when using pre-built map

        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        localizer = self.get_parameter('localizer').value
        self.active_goal_started_at = None
        self.goal_handle = None
        self._pending_goal = None
        self._active_goal_intent = None
        self._send_in_flight = False

        self.twin_state = None
        self._last_return_failed_at: float | None = None
        self._start_occupied_escape_needed = False
        self._escape_active = False

        # VOLATILE QoS — matches field_planner_node publisher and ros2 topic pub
        self.create_subscription(String, '/next_cell_goal', self._goal_cb, 10)
        self.create_subscription(String, '/twin_state', self._twin_state_cb, 10)
        self.goal_reached_pub = self.create_publisher(String, '/goal_reached', 10)

        self._initial_pose = self._make_pose(
            float(self.get_parameter('initial_x').value),
            float(self.get_parameter('initial_y').value),
            float(self.get_parameter('initial_yaw').value),
        )
        self.setInitialPose(self._initial_pose)
        self.get_logger().info(f'Waiting for Nav2 (localizer={localizer})...')
        self.waitUntilNav2Active(localizer=localizer)
        self.setInitialPose(self._initial_pose)
        self.get_logger().info('Nav2 active — initial pose set, ready for goals')

        self.create_timer(0.5, self._check_timeout)

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
            if (self._last_return_failed_at is not None and
                    time.monotonic() - self._last_return_failed_at < _RETURN_COOLDOWN_SEC):
                remaining = _RETURN_COOLDOWN_SEC - (time.monotonic() - self._last_return_failed_at)
                self.get_logger().info(
                    f'return_to_base cooldown active — {remaining:.0f}s remaining',
                    throttle_duration_sec=10.0,
                )
                return
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
        if self.goal_handle is not None or self.active_goal_started_at is not None or self._send_in_flight:
            self.get_logger().info(
                f'Ignoring {mode} goal while Nav2 goal is active'
            )
            return
        self._pending_goal = (mode, x, y, yaw)
        self._send_pending_goal()

    def _on_cancel_done(self):
        self.get_logger().info('Previous goal canceled — sending pending goal')
        self._send_pending_goal()

    def _send_pending_goal(self):
        if self._pending_goal is None:
            return
        mode, x, y, yaw = self._pending_goal
        self._pending_goal = None
        # Clear global costmap of stale phantoms before sending; _send_in_flight stays
        # False during the clear so a more-important goal (e.g. return_to_base) can
        # still arrive and supersede this one.
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_global_srv.call_async(req)
        future.add_done_callback(lambda f: self._do_send_goal(mode, x, y, yaw))

    def _do_send_goal(self, mode, x, y, yaw):
        # A racing call may have already taken the slot (two clears fired in quick
        # succession) — drop the stale one rather than double-sending.
        if self._send_in_flight or self.goal_handle is not None:
            self.get_logger().info(
                f'Dropping stale {mode} goal ({x:.2f}, {y:.2f}) — newer goal already sent'
            )
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose(x, y, yaw)
        self._send_in_flight = True
        send_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=None,
        )
        send_future.add_done_callback(lambda f: self._on_goal_accepted(f, mode, x, y))
        self.get_logger().info(f'Sending goal: mode={mode} target=({x:.2f}, {y:.2f})')

    def _on_goal_accepted(self, future, mode, x, y):
        self._send_in_flight = False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected by Nav2: ({x:.2f}, {y:.2f})')
            self.active_goal_started_at = None
            self._active_goal_intent = None
            return
        self.goal_handle = goal_handle
        self._active_goal_intent = {'mode': mode, 'x': x, 'y': y}
        self.active_goal_started_at = time.monotonic()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)
        self.get_logger().info(f'Goal accepted — navigating to ({x:.2f}, {y:.2f})')

    def _on_nav_result(self, future):
        self.goal_handle = None
        self.active_goal_started_at = None
        intent = self._active_goal_intent
        self._active_goal_intent = None
        result = future.result()
        intent_mode = intent.get('mode') if intent else None
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded')
            self._last_return_failed_at = None
            if intent_mode == 'escape':
                self._escape_active = False
                self.get_logger().info('[escape] complete — resuming normal planning')
            elif intent_mode == 'cleanup':
                self._publish_goal_reached(intent)
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal canceled')
            if intent_mode == 'escape':
                self._escape_active = False
        else:
            error_code = getattr(result.result, 'error_code', 0)
            if intent_mode == 'escape':
                self._escape_active = False
                if error_code == _COMPUTE_PATH_START_OCCUPIED:
                    self.get_logger().warn(
                        '[escape] escape goal itself hit START_OCCUPIED — '
                        'clearing flag, resuming normal planning'
                    )
                else:
                    self.get_logger().info(
                        f'[escape] complete (failed, code={error_code}) — '
                        'resuming normal planning'
                    )
            elif error_code == _COMPUTE_PATH_START_OCCUPIED:
                pos = f'({intent["x"]:.2f}, {intent["y"]:.2f})' if intent else '(unknown)'
                self.get_logger().warn(
                    f'START_OCCUPIED (code={error_code}) at {pos} — '
                    'scheduling escape to origin'
                )
                self._start_occupied_escape_needed = True
            else:
                self.get_logger().warn(f'Goal failed (status={result.status})')
                if intent_mode == 'return_to_base':
                    self._last_return_failed_at = time.monotonic()
                    self.get_logger().info(
                        f'return_to_base failed — cooling down for {_RETURN_COOLDOWN_SEC:.0f}s'
                    )

    def _send_escape_goal(self):
        if self._escape_active or self.goal_handle is not None or self._send_in_flight:
            return
        self._escape_active = True
        self.get_logger().info('[escape] sending escape goal to origin (0.0, 0.0)')
        self._do_send_goal('escape', 0.0, 0.0, 0.0)

    def _publish_goal_reached(self, intent):
        event = {
            'schema': 'dtas.goal_reached.v1',
            'stamp': self._stamp(),
            'source': 'mission_planner_node',
            'mode': 'cleanup',
            'goal': {
                'frame_id': 'map',
                'x': round(float(intent['x']), 3),
                'y': round(float(intent['y']), 3),
            },
        }
        msg = String()
        msg.data = json.dumps(event)
        self.goal_reached_pub.publish(msg)
        self.get_logger().info(
            f'/goal_reached published for cleanup at '
            f'({event["goal"]["x"]:.2f}, {event["goal"]["y"]:.2f})'
        )

    def _check_timeout(self):
        if self._start_occupied_escape_needed and not self._escape_active:
            if self.goal_handle is None and not self._send_in_flight:
                self._start_occupied_escape_needed = False
                self._send_escape_goal()
                return
        if self.active_goal_started_at is None or self.goal_handle is None:
            return
        timeout = _ESCAPE_TIMEOUT_SEC if self._escape_active else self.goal_timeout_sec
        if time.monotonic() - self.active_goal_started_at > timeout:
            label = '[escape] escape timed out' if self._escape_active else 'Goal timed out'
            self.get_logger().warn(f'{label} — canceling')
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda f: self.get_logger().info('Timeout cancel confirmed')
            )
            if self._escape_active:
                self._escape_active = False
                self.get_logger().info('[escape] cleared after timeout — resuming normal planning')
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