#!/usr/bin/env python3

import math
import time
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String


def yaw_to_quaternion(yaw):
    half_yaw = yaw * 0.5
    return {
        "z": math.sin(half_yaw),
        "w": math.cos(half_yaw),
    }


class GoalNavigationNode(BasicNavigator):
    def __init__(self):
        super().__init__(node_name="goal_navigation_node")

        self.declare_parameter("goal_timeout_sec", 120.0)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)

        self.goal_timeout_sec = self.get_float_parameter("goal_timeout_sec")
        self.active_goal_started_at = None
        self.last_status = None

        self.nav2_ready = False
        self.pending_goal = None
        self._lock = threading.Lock()

        self.status_publisher = self.create_publisher(
            String, "/coral_g/navigation_status", 10)

        self.goal_subscription = self.create_subscription(
            PoseStamped, "/coral_g/goal_pose", self.goal_callback, 10)

        self.status_timer = self.create_timer(0.5, self.check_navigation_status)

        # Wait for Nav2 in a background thread so spin() starts immediately
        # and goals published during startup are queued rather than dropped
        threading.Thread(target=self._init_nav2, daemon=True).start()

    def _init_nav2(self):
        initial_pose = self._make_pose(
            self.get_float_parameter("initial_x"),
            self.get_float_parameter("initial_y"),
            self.get_float_parameter("initial_yaw"),
        )
        self.setInitialPose(initial_pose)
        self.publish_status("waiting_for_nav2")
        self.waitUntilNav2Active(localizer="slam_toolbox")

        with self._lock:
            self.nav2_ready = True

        self.publish_status("ready")
        self.get_logger().info("Nav2 active — ready to receive goals")

        # Send any goal that arrived before Nav2 was ready
        with self._lock:
            queued = self.pending_goal
            self.pending_goal = None

        if queued is not None:
            self.get_logger().info("Sending queued goal that arrived during startup")
            self._send_goal(queued)

    def get_float_parameter(self, name):
        return float(self.get_parameter(name).value)

    def _make_pose(self, x, y, yaw) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        q = yaw_to_quaternion(yaw)
        pose.pose.orientation.z = q["z"]
        pose.pose.orientation.w = q["w"]
        return pose

    def goal_callback(self, goal: PoseStamped):
        if goal.header.frame_id != "map":
            self.get_logger().error("Rejected goal: frame_id must be 'map'")
            self.publish_status("failed")
            return

        with self._lock:
            ready = self.nav2_ready

        if not ready:
            self.get_logger().info("Nav2 not ready yet — goal queued")
            with self._lock:
                self.pending_goal = goal
            return

        self._send_goal(goal)

    def _send_goal(self, goal: PoseStamped):
        with self._lock:
            if self.active_goal_started_at is not None:
                self.cancelTask()
                self.publish_status("canceled")

        goal.header.stamp = self.get_clock().now().to_msg()
        self.publish_status("received")
        self.goToPose(goal)

        with self._lock:
            self.active_goal_started_at = time.monotonic()

        self.publish_status("active")

    def check_navigation_status(self):
        with self._lock:
            started_at = self.active_goal_started_at

        if started_at is None:
            return

        if time.monotonic() - started_at > self.goal_timeout_sec:
            self.cancelTask()
            with self._lock:
                self.active_goal_started_at = None
            self.publish_status("timeout")
            return

        if not self.isTaskComplete():
            return

        result = self.getResult()

        with self._lock:
            self.active_goal_started_at = None

        if result == TaskResult.SUCCEEDED:
            self.publish_status("succeeded")
        elif result == TaskResult.CANCELED:
            self.publish_status("canceled")
        else:
            self.publish_status("failed")

    def publish_status(self, status):
        self.last_status = status
        message = String()
        message.data = status
        self.status_publisher.publish(message)
        self.get_logger().info(f"Navigation status: {status}")


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
