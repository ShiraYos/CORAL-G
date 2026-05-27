import json
from typing import Any, Callable, Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from coral_g_interfaces.srv import ResetReseed

from .contracts import TOPICS, distance, dumps, quaternion_from_yaw, validate_json, yaw_from_quaternion
from .core import (
    DEFAULT_FIELD_POLICY,
    DEFAULT_RETURN_POLICY,
    DigitalTwinFusion,
    MaterialBelief,
    default_environment,
    fallback_map_update,
    make_collection_report,
    make_demo_report,
    make_robot_state,
    map_update_from_scan,
    plan_next_cell,
    simulate_density,
)


def _publish_json(publisher: Any, message: Dict[str, Any]) -> None:
    ros_message = String()
    ros_message.data = dumps(message)
    publisher.publish(ros_message)


def _json_callback(kind: str, handler: Callable[[Dict[str, Any]], None], logger: Any) -> Callable[[String], None]:
    def callback(message: String) -> None:
        try:
            handler(validate_json(kind, message.data))
        except Exception as exc:
            logger.warning(f"ignored invalid {kind} message: {exc}")

    return callback


class TwinInputNode(Node):
    def __init__(self) -> None:
        super().__init__("twin_input_node")
        self.declare_parameter("zone_width_m", 8)
        self.declare_parameter("zone_height_m", 10)
        self.declare_parameter("zone_x_min", -1.75)
        self.declare_parameter("zone_y_min", -3.75)
        self.declare_parameter("cell_size_m", 0.5)
        self.declare_parameter("tick_rate_hz", 1.0)
        self.environment_pub = self.create_publisher(String, TOPICS["environment_field"], 10)
        self.map_pub = self.create_publisher(String, TOPICS["map_sync_update"], 10)
        self.latest_map_update = fallback_map_update(False, self.get_parameter("cell_size_m").value)
        self.create_subscription(LaserScan, TOPICS["scan"], self._scan_callback, 10)
        tick = max(0.1, 1.0 / float(self.get_parameter("tick_rate_hz").value))
        self.create_timer(tick, self._tick)

    def _scan_callback(self, message: LaserScan) -> None:
        self.latest_map_update = map_update_from_scan(message.ranges, message.angle_min, message.angle_increment, message.range_max)

    def _tick(self) -> None:
        env = default_environment(
            int(self.get_parameter("zone_width_m").value),
            int(self.get_parameter("zone_height_m").value),
            float(self.get_parameter("cell_size_m").value),
            float(self.get_parameter("zone_x_min").value),
            float(self.get_parameter("zone_y_min").value),
        )
        _publish_json(self.environment_pub, env)
        _publish_json(self.map_pub, self.latest_map_update)


class MaterialBeliefNode(Node):
    def __init__(self) -> None:
        super().__init__("material_belief_node")
        self.belief = MaterialBelief()
        self.publisher = self.create_publisher(String, TOPICS["material_distribution"], 10)
        self.create_subscription(
            String,
            TOPICS["collection_report"],
            _json_callback("collection_report", self._collection_callback, self.get_logger()),
            10,
        )
        self.create_timer(1.0, self._tick)

    def _collection_callback(self, message: Dict[str, Any]) -> None:
        _publish_json(self.publisher, self.belief.update_from_report(message))

    def _tick(self) -> None:
        _publish_json(self.publisher, self.belief.message())


class DigitalTwinStateNode(Node):
    def __init__(self) -> None:
        super().__init__("digital_twin_state")
        self.fusion = DigitalTwinFusion()
        self.publisher = self.create_publisher(String, TOPICS["twin_state"], 10)
        subscriptions = {
            "environment_field": self.fusion.set_environment,
            "map_sync_update": self.fusion.set_map_update,
            "material_distribution": self.fusion.set_material_distribution,
            "collection_report": self.fusion.add_collection_report,
        }
        for kind, handler in subscriptions.items():
            self.create_subscription(
                String,
                TOPICS[kind],
                _json_callback(kind, lambda msg, h=handler: self._update(h, msg), self.get_logger()),
                10,
            )
        self.create_timer(1.0, self._tick)

    def _update(self, handler: Callable[[Dict[str, Any]], Dict[str, Any]], message: Dict[str, Any]) -> None:
        _publish_json(self.publisher, handler(message))

    def _tick(self) -> None:
        _publish_json(self.publisher, self.fusion.message())


class DebrisSimulationNode(Node):
    def __init__(self) -> None:
        super().__init__("debris_simulation_node")
        self.declare_parameter("debris_count", 100)
        self.declare_parameter("simulation_horizon_sec", 60.0)
        self.declare_parameter("random_seed", 23)
        self.declare_parameter("tick_rate_hz", 1.0)
        self.twin_state: Optional[Dict[str, Any]] = None
        self.publisher = self.create_publisher(String, TOPICS["debris_density_map"], 10)
        self.create_subscription(
            String,
            TOPICS["twin_state"],
            _json_callback("twin_state", self._twin_callback, self.get_logger()),
            10,
        )
        self.create_service(ResetReseed, "reset_reseed", self._reset_reseed)
        self.create_timer(max(0.1, 1.0 / float(self.get_parameter("tick_rate_hz").value)), self._tick)

    def _twin_callback(self, message: Dict[str, Any]) -> None:
        self.twin_state = message
        self._tick()

    def _reset_reseed(self, request: ResetReseed.Request, response: ResetReseed.Response) -> ResetReseed.Response:
        self.set_parameters([Parameter("random_seed", Parameter.Type.INTEGER, int(request.random_seed))])
        response.accepted = True
        response.message = f"reset accepted: {request.reason}; seed={request.random_seed}"
        self._tick()
        return response

    def _tick(self) -> None:
        if not self.twin_state:
            return
        density = simulate_density(
            self.twin_state,
            int(self.get_parameter("debris_count").value),
            float(self.get_parameter("simulation_horizon_sec").value),
            int(self.get_parameter("random_seed").value),
        )
        _publish_json(self.publisher, density)


class RobotStateNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_state_node")
        self.declare_parameter("state_tick_rate_hz", 1.0)
        self.declare_parameter("storage_fill", 0.0)
        self.declare_parameter("fuel_level", 1.0)
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("heading", 0.0)
        self.navigation_status = "waiting_for_nav2"
        self.latest_goal_mode = "idle"
        self.pose = {
            "x": float(self.get_parameter("x").value),
            "y": float(self.get_parameter("y").value),
            "heading": float(self.get_parameter("heading").value),
        }
        self.publisher = self.create_publisher(String, TOPICS["robot_state"], 10)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self.create_subscription(String, TOPICS["navigation_status"], self._status_callback, 10)
        self.create_subscription(
            String,
            TOPICS["next_cell_goal"],
            _json_callback("next_cell_goal", self._goal_callback, self.get_logger()),
            10,
        )
        tick = max(0.1, 1.0 / float(self.get_parameter("state_tick_rate_hz").value))
        self.create_timer(tick, self._tick)

    def _goal_callback(self, message: Dict[str, Any]) -> None:
        self.latest_goal_mode = str(message.get("mode", "idle"))

    def _odom_callback(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        self.pose = {
            "x": float(message.pose.pose.position.x),
            "y": float(message.pose.pose.position.y),
            "heading": yaw_from_quaternion({"z": orientation.z, "w": orientation.w}),
        }

    def _status_callback(self, message: String) -> None:
        self.navigation_status = message.data
        if message.data == "succeeded":
            current_fuel = float(self.get_parameter("fuel_level").value)
            current_storage = float(self.get_parameter("storage_fill").value)
            if self.latest_goal_mode == "return_to_base" and distance(self.pose, DEFAULT_RETURN_POLICY["base"]) <= 0.5:
                self.set_parameters(
                    [
                        Parameter("fuel_level", Parameter.Type.DOUBLE, 1.0),
                        Parameter("storage_fill", Parameter.Type.DOUBLE, 0.0),
                    ]
                )
            elif self.latest_goal_mode == "cleanup_cell":
                self.set_parameters(
                    [
                        Parameter("fuel_level", Parameter.Type.DOUBLE, max(0.0, current_fuel - 0.08)),
                        Parameter("storage_fill", Parameter.Type.DOUBLE, min(1.0, current_storage + 0.25)),
                    ]
                )

    def _tick(self) -> None:
        state = make_robot_state(
            self.pose["x"],
            self.pose["y"],
            self.pose["heading"],
            float(self.get_parameter("storage_fill").value),
            float(self.get_parameter("fuel_level").value),
            self.navigation_status,
            DEFAULT_RETURN_POLICY["base"],
        )
        _publish_json(self.publisher, state)


class FieldPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("field_planner_node")
        self.twin_state: Optional[Dict[str, Any]] = None
        self.density_map: Optional[Dict[str, Any]] = None
        self.robot_state: Optional[Dict[str, Any]] = None
        self.publisher = self.create_publisher(String, TOPICS["next_cell_goal"], 10)
        for kind, setter in {
            "twin_state": "_set_twin",
            "debris_density_map": "_set_density",
            "robot_state": "_set_robot",
        }.items():
            self.create_subscription(
                String,
                TOPICS[kind],
                _json_callback(kind, getattr(self, setter), self.get_logger()),
                10,
            )
        self.create_timer(1.0, self._tick)

    def _set_twin(self, message: Dict[str, Any]) -> None:
        self.twin_state = message

    def _set_density(self, message: Dict[str, Any]) -> None:
        self.density_map = message

    def _set_robot(self, message: Dict[str, Any]) -> None:
        self.robot_state = message

    def _tick(self) -> None:
        decision = plan_next_cell(self.twin_state, self.density_map, self.robot_state, DEFAULT_FIELD_POLICY, DEFAULT_RETURN_POLICY)
        _publish_json(self.publisher, decision)


class MissionPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_planner_node")
        self.robot_state: Optional[Dict[str, Any]] = None
        self.navigation_status = "waiting_for_nav2"
        self.last_goal_key = None
        self.publisher = self.create_publisher(PoseStamped, TOPICS["goal_pose"], 10)
        self.create_subscription(
            String,
            TOPICS["next_cell_goal"],
            _json_callback("next_cell_goal", self._goal_callback, self.get_logger()),
            10,
        )
        self.create_subscription(String, TOPICS["navigation_status"], self._nav_status_callback, 10)
        self.create_subscription(
            String,
            TOPICS["robot_state"],
            _json_callback("robot_state", self._robot_callback, self.get_logger()),
            10,
        )

    def _robot_callback(self, message: Dict[str, Any]) -> None:
        self.robot_state = message

    def _nav_status_callback(self, message: String) -> None:
        self.navigation_status = message.data
        if message.data in ("succeeded", "failed", "timeout", "waiting_for_nav2"):
            self.last_goal_key = None

    def _goal_callback(self, message: Dict[str, Any]) -> None:
        if message.get("status") != "ready" or message.get("mode") == "idle":
            return
        if self.navigation_status in ("received", "active"):
            return
        target = DEFAULT_RETURN_POLICY["base"] if message.get("mode") == "return_to_base" else message.get("cell", {})
        goal_key = (
            message.get("mode"),
            round(float(target.get("x", 0.0)), 2),
            round(float(target.get("y", 0.0)), 2),
            round(float(target.get("heading", 0.0)), 2),
        )
        if goal_key == self.last_goal_key:
            return
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = float(target.get("x", 0.0))
        goal.pose.position.y = float(target.get("y", 0.0))
        goal.pose.position.z = 0.0
        q = quaternion_from_yaw(float(target.get("heading", 0.0)))
        goal.pose.orientation.x = q["x"]
        goal.pose.orientation.y = q["y"]
        goal.pose.orientation.z = q["z"]
        goal.pose.orientation.w = q["w"]
        self.last_goal_key = goal_key
        self.publisher.publish(goal)


class GoalNavigationNode(Node):
    def __init__(self) -> None:
        super().__init__("goal_navigation_node")
        self.declare_parameter("goal_timeout_sec", 120.0)
        self.status_pub = self.create_publisher(String, TOPICS["navigation_status"], 10)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.active_goal = None
        self.pending_goal_key = None
        self.active_goal_key = None
        self.completed_goal_key = None
        self.goal_start = None
        self.create_subscription(PoseStamped, TOPICS["goal_pose"], self._goal_callback, 10)
        self.create_timer(1.0, self._watchdog)
        self._publish_status("waiting_for_nav2")

    def _goal_key(self, pose: PoseStamped) -> tuple:
        return (
            round(float(pose.pose.position.x), 2),
            round(float(pose.pose.position.y), 2),
            round(float(pose.pose.orientation.z), 3),
            round(float(pose.pose.orientation.w), 3),
        )

    def _publish_status(self, status: str) -> None:
        message = String()
        message.data = status
        self.status_pub.publish(message)

    def _goal_callback(self, pose: PoseStamped) -> None:
        if pose.header.frame_id != "map":
            self._publish_status("failed")
            self.get_logger().warning("rejected goal_pose because header.frame_id is not map")
            return
        goal_key = self._goal_key(pose)
        if goal_key in (self.pending_goal_key, self.active_goal_key, self.completed_goal_key):
            return
        self._publish_status("received")
        if not self.client.wait_for_server(timeout_sec=0.1):
            self._publish_status("waiting_for_nav2")
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.goal_start = self.get_clock().now()
        self.pending_goal_key = goal_key
        self.completed_goal_key = None
        send_future = self.client.send_goal_async(goal)
        send_future.add_done_callback(lambda future, key=goal_key: self._goal_response_callback(future, key))

    def _goal_response_callback(self, future: Any, goal_key: tuple) -> None:
        goal_handle = future.result()
        self.pending_goal_key = None
        if not goal_handle.accepted:
            self._publish_status("failed")
            return
        self.active_goal = goal_handle
        self.active_goal_key = goal_key
        self._publish_status("active")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda future, key=goal_key: self._result_callback(future, key))

    def _result_callback(self, future: Any, goal_key: tuple) -> None:
        status = getattr(future.result(), "status", None)
        self.active_goal = None
        self.active_goal_key = None
        if status == 4:
            self.completed_goal_key = goal_key
            self._publish_status("succeeded")
        else:
            self.completed_goal_key = None
            self._publish_status("failed")

    def _watchdog(self) -> None:
        if not self.active_goal or not self.goal_start:
            return
        age_ns = (self.get_clock().now() - self.goal_start).nanoseconds
        if age_ns / 1_000_000_000.0 > float(self.get_parameter("goal_timeout_sec").value):
            self.active_goal.cancel_goal_async()
            self.active_goal = None
            self.active_goal_key = None
            self.completed_goal_key = None
            self._publish_status("timeout")


class CollectionReportNode(Node):
    def __init__(self) -> None:
        super().__init__("collection_report_node")
        self.declare_parameter("default_material", "unknown")
        self.declare_parameter("collection_radius_m", 0.75)
        self.latest: Dict[str, Optional[Dict[str, Any]]] = {
            "next_cell_goal": None,
            "robot_state": None,
            "material_distribution": None,
            "debris_density_map": None,
        }
        self.navigation_status = "waiting_for_nav2"
        self.collection_pub = self.create_publisher(String, TOPICS["collection_report"], 10)
        self.report_pub = self.create_publisher(String, TOPICS["report"], 10)
        for kind in self.latest:
            self.create_subscription(
                String,
                TOPICS[kind],
                _json_callback(kind, lambda msg, k=kind: self._set_latest(k, msg), self.get_logger()),
                10,
            )
        self.create_subscription(String, TOPICS["navigation_status"], self._status_callback, 10)
        self.create_timer(5.0, self._tick)

    def _set_latest(self, kind: str, message: Dict[str, Any]) -> None:
        self.latest[kind] = message

    def _status_callback(self, message: String) -> None:
        self.navigation_status = message.data
        if message.data == "succeeded" and self.latest.get("next_cell_goal"):
            goal = self.latest["next_cell_goal"]
            if goal.get("status") != "ready" or goal.get("mode") != "cleanup_cell":
                return
            location = goal.get("cell", {})
            report = make_collection_report(
                location,
                str(self.get_parameter("default_material").value),
                1,
                float(self.get_parameter("collection_radius_m").value),
            )
            _publish_json(self.collection_pub, report)

    def _tick(self) -> None:
        report = make_demo_report(
            "coral_g_demo",
            self.latest.get("next_cell_goal"),
            self.latest.get("robot_state"),
            self.latest.get("material_distribution"),
            self.latest.get("debris_density_map"),
            DEFAULT_FIELD_POLICY,
        )
        _publish_json(self.report_pub, report)


class DemoControlSurface(Node):
    def __init__(self) -> None:
        super().__init__("demo_control_surface")
        self.create_subscription(
            String,
            TOPICS["report"],
            _json_callback("report", self._report_callback, self.get_logger()),
            10,
        )
        self.reset_client = self.create_client(ResetReseed, "reset_reseed")

    def _report_callback(self, message: Dict[str, Any]) -> None:
        total = message.get("collection_stats", {}).get("total", 0)
        self.get_logger().info(f"demo report received: run_id={message.get('run_id')} total_collected={total}")


def _spin(node_cls: Callable[[], Node]) -> None:
    rclpy.init()
    node = node_cls()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def twin_input_main() -> None:
    _spin(TwinInputNode)


def material_belief_main() -> None:
    _spin(MaterialBeliefNode)


def digital_twin_main() -> None:
    _spin(DigitalTwinStateNode)


def debris_simulation_main() -> None:
    _spin(DebrisSimulationNode)


def robot_state_main() -> None:
    _spin(RobotStateNode)


def field_planner_main() -> None:
    _spin(FieldPlannerNode)


def mission_planner_main() -> None:
    _spin(MissionPlannerNode)


def goal_navigation_main() -> None:
    _spin(GoalNavigationNode)


def collection_report_main() -> None:
    _spin(CollectionReportNode)


def demo_control_main() -> None:
    _spin(DemoControlSurface)
