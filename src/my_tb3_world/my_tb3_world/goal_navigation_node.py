#!/usr/bin/env python3

import math
import time

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

        self.status_publisher = self.create_publisher(
            String,
            "/coral_g/navigation_status",
            10,
        )
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            "/coral_g/goal_pose",
            self.goal_callback,
            10,
        )
        self.status_timer = self.create_timer(0.5, self.check_navigation_status)

        initial_pose = self.create_initial_pose()
        self.setInitialPose(initial_pose)
        self.publish_status("waiting_for_nav2")
        self.waitUntilNav2Active(localizer="slam_toolbox")
        self.publish_status("ready")

    def create_initial_pose(self):
        initial_x = self.get_float_parameter("initial_x")
        initial_y = self.get_float_parameter("initial_y")
        initial_yaw = self.get_float_parameter("initial_yaw")

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = initial_x
        pose.pose.position.y = initial_y
        quaternion = yaw_to_quaternion(initial_yaw)
        pose.pose.orientation.z = quaternion["z"]
        pose.pose.orientation.w = quaternion["w"]
        return pose

    def get_float_parameter(self, name):
        value = self.get_parameter(name).value
        return float(value)

    def goal_callback(self, goal):
        if goal.header.frame_id != "map":
            self.get_logger().error(
                "Rejected navigation goal: frame_id must be 'map'"
            )
            self.publish_status("failed")
            return

        if self.active_goal_started_at is not None:
            self.cancelTask()
            self.publish_status("canceled")

        goal.header.stamp = self.get_clock().now().to_msg()
        self.publish_status("received")
        self.goToPose(goal)
        self.active_goal_started_at = time.monotonic()
        self.publish_status("active")

    def check_navigation_status(self):
        if self.active_goal_started_at is None:
            return

        if time.monotonic() - self.active_goal_started_at > self.goal_timeout_sec:
            self.cancelTask()
            self.active_goal_started_at = None
            self.publish_status("timeout")
            return

        if not self.isTaskComplete():
            return

        result = self.getResult()
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
