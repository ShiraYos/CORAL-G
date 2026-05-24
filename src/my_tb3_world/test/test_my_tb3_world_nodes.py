import importlib
import math
import sys
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = ROOT / "src" / "my_tb3_world"
sys.path.insert(0, str(PACKAGE_SRC))


class FakeLogger:
    def __init__(self):
        self.messages = []
        self.errors = []

    def info(self, message):
        self.messages.append(message)

    def error(self, message):
        self.errors.append(message)


class FakePublisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeClock:
    class _Now:
        def to_msg(self):
            return "fake-time"

    def now(self):
        return self._Now()


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.logger = FakeLogger()
        self.publishers = []
        self.subscriptions = []
        self.timers = []
        self.parameters = {}

    def create_publisher(self, _message_type, topic, _queue_size):
        publisher = FakePublisher(topic)
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = {
            "message_type": message_type,
            "topic": topic,
            "callback": callback,
            "qos": qos,
        }
        self.subscriptions.append(subscription)
        return subscription

    def create_timer(self, interval, callback):
        timer = {"interval": interval, "callback": callback}
        self.timers.append(timer)
        return timer

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return FakeClock()

    def declare_parameter(self, name, value):
        self.parameters[name] = value

    def get_parameter(self, name):
        value = self.parameters[name]
        return types.SimpleNamespace(
            value=value,
            get_parameter_value=lambda: types.SimpleNamespace(
                double_value=float(value),
            )
        )

    def destroy_node(self):
        pass


class FakeTwistStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id="")
        self.twist = types.SimpleNamespace(
            linear=types.SimpleNamespace(x=0.0),
            angular=types.SimpleNamespace(z=0.0),
        )


class FakeLaserScan:
    def __init__(
        self,
        ranges,
        angle_min=-math.pi,
        angle_increment=math.radians(1),
        range_min=0.0,
        range_max=10.0,
    ):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_increment = angle_increment
        self.range_min = range_min
        self.range_max = range_max


class FakePoseStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id="")
        self.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
        )


class FakeString:
    def __init__(self):
        self.data = ""


class FakeTaskResult:
    SUCCEEDED = "succeeded"
    CANCELED = "canceled"
    FAILED = "failed"


class FakeBasicNavigator(FakeNode):
    next_result = FakeTaskResult.SUCCEEDED
    task_complete = False

    def __init__(self, node_name="basic_navigator", namespace=""):
        super().__init__(node_name)
        FakeBasicNavigator.next_result = FakeTaskResult.SUCCEEDED
        FakeBasicNavigator.task_complete = False
        self.namespace = namespace
        self.initial_pose = None
        self.goals = []
        self.cancel_count = 0
        self.waited_for_nav2 = False

    def setInitialPose(self, pose):
        self.initial_pose = pose

    def waitUntilNav2Active(self, *args, **kwargs):
        self.waited_for_nav2 = True
        self.wait_args = args
        self.wait_kwargs = kwargs

    def goToPose(self, goal):
        self.goals.append(goal)
        FakeBasicNavigator.task_complete = False

    def cancelTask(self):
        self.cancel_count += 1
        FakeBasicNavigator.task_complete = True
        FakeBasicNavigator.next_result = FakeTaskResult.CANCELED

    def isTaskComplete(self):
        return FakeBasicNavigator.task_complete

    def getResult(self):
        return FakeBasicNavigator.next_result


def install_ros_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.spin = lambda node: None
    rclpy.shutdown = lambda: None

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode

    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = lambda **kwargs: kwargs
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT="best_effort")

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseStamped = FakePoseStamped
    geometry_msgs_msg.TwistStamped = FakeTwistStamped

    nav2_simple_commander = types.ModuleType("nav2_simple_commander")
    nav2_robot_navigator = types.ModuleType("nav2_simple_commander.robot_navigator")
    nav2_robot_navigator.BasicNavigator = FakeBasicNavigator
    nav2_robot_navigator.TaskResult = FakeTaskResult

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.LaserScan = FakeLaserScan

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = FakeString

    sys.modules.update(
        {
            "rclpy": rclpy,
            "rclpy.node": rclpy_node,
            "rclpy.qos": rclpy_qos,
            "geometry_msgs": geometry_msgs,
            "geometry_msgs.msg": geometry_msgs_msg,
            "nav2_simple_commander": nav2_simple_commander,
            "nav2_simple_commander.robot_navigator": nav2_robot_navigator,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": sensor_msgs_msg,
            "std_msgs": std_msgs,
            "std_msgs.msg": std_msgs_msg,
        }
    )


install_ros_stubs()


class MyTb3WorldNodeTests(unittest.TestCase):
    def test_publisher_publishes_forward_velocity(self):
        module = importlib.import_module("my_tb3_world.publisher_node")
        node = module.MovePublisher()

        node.publish_cmd()

        self.assertEqual(node.publisher_.topic, "/cmd_vel")
        self.assertEqual(len(node.publisher_.messages), 1)
        message = node.publisher_.messages[0]
        self.assertEqual(message.header.frame_id, "base_link")
        self.assertEqual(message.twist.linear.x, 0.2)
        self.assertEqual(message.twist.angular.z, 0.0)

    def test_subscriber_reads_front_lidar_range(self):
        module = importlib.import_module("my_tb3_world.subscriber_node")
        node = module.SubscriberNode()
        scan = FakeLaserScan([1.0, 2.5, 4.0])

        node.scan_callback(scan)

        self.assertIn("front = 2.500 m", node.logger.messages[-1])

    def test_object_avoidance_turns_when_front_is_blocked(self):
        module = importlib.import_module("my_tb3_world.objectAvoidance_node")
        node = module.ObjectAvoidanceNode()
        ranges = [2.0] * 181
        ranges[90] = 0.2
        scan = FakeLaserScan(ranges, angle_min=math.radians(-90), angle_increment=math.radians(1))

        node.lidar_callback(scan)

        message = node.publisher.messages[-1]
        self.assertEqual(message.twist.linear.x, 0.0)
        self.assertEqual(message.twist.angular.z, 1.0)

    def test_object_avoidance_moves_forward_when_front_is_clear(self):
        module = importlib.import_module("my_tb3_world.objectAvoidance_node")
        node = module.ObjectAvoidanceNode()
        scan = FakeLaserScan([2.0] * 181, angle_min=math.radians(-90), angle_increment=math.radians(1))

        node.lidar_callback(scan)

        message = node.publisher.messages[-1]
        self.assertEqual(message.twist.linear.x, 0.2)
        self.assertEqual(message.twist.angular.z, 0.0)

    def test_goal_navigation_accepts_map_goal_and_reports_active(self):
        module = importlib.import_module("my_tb3_world.goal_navigation_node")
        node = module.GoalNavigationNode()
        goal = FakePoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = 1.5

        node.goal_callback(goal)

        self.assertEqual(node.goal_subscription["topic"], "/coral_g/goal_pose")
        self.assertEqual(node.status_publisher.topic, "/coral_g/navigation_status")
        self.assertEqual(node.goals[-1], goal)
        self.assertEqual(node.status_publisher.messages[-2].data, "received")
        self.assertEqual(node.status_publisher.messages[-1].data, "active")

    def test_goal_navigation_rejects_non_map_goal(self):
        module = importlib.import_module("my_tb3_world.goal_navigation_node")
        node = module.GoalNavigationNode()
        goal = FakePoseStamped()
        goal.header.frame_id = "odom"

        node.goal_callback(goal)

        self.assertEqual(node.goals, [])
        self.assertEqual(node.status_publisher.messages[-1].data, "failed")

    def test_goal_navigation_reports_succeeded_when_task_completes(self):
        module = importlib.import_module("my_tb3_world.goal_navigation_node")
        node = module.GoalNavigationNode()
        goal = FakePoseStamped()
        goal.header.frame_id = "map"
        node.goal_callback(goal)

        FakeBasicNavigator.next_result = FakeTaskResult.SUCCEEDED
        FakeBasicNavigator.task_complete = True
        node.check_navigation_status()

        self.assertEqual(node.status_publisher.messages[-1].data, "succeeded")


if __name__ == "__main__":
    unittest.main()