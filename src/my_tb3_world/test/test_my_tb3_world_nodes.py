import importlib
import json
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
        self.warnings = []

    def info(self, message):
        self.messages.append(message)

    def error(self, message):
        self.errors.append(message)

    def warn(self, message):
        self.warnings.append(message)


class FakePublisher:
    def __init__(self, topic, qos=None):
        self.topic = topic
        self.qos = qos
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeFuture:
    def __init__(self, result=None):
        self._result = result

    def done(self):
        return self._result is not None

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        callback(self)


class FakePendingFuture:
    def __init__(self, result=None):
        self._result = result
        self.callbacks = []

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        self.callbacks.append(callback)


class FakeGoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.cancel_count = 0
        self.result_future = FakePendingFuture(
            types.SimpleNamespace(status="succeeded")
        )

    def cancel_goal_async(self):
        self.cancel_count += 1
        return FakeFuture(types.SimpleNamespace(status="canceled"))

    def get_result_async(self):
        return self.result_future


class FakeNavToPoseClient:
    def __init__(self):
        self.goals = []
        self.last_goal_handle = None

    def send_goal_async(self, goal_msg, feedback_callback=None):
        self.goals.append(goal_msg)
        self.last_goal_handle = FakeGoalHandle(accepted=True)
        return FakeFuture(self.last_goal_handle)


class FakeClient:
    def __init__(self, service_type, service_name):
        self.service_type = service_type
        self.service_name = service_name
        self.requests = []
        self.response = None

    def service_is_ready(self):
        return self.response is not None

    def call_async(self, request):
        self.requests.append(request)
        return FakeFuture(self.response)


class FakeClock:
    class _Now:
        def to_msg(self):
            return types.SimpleNamespace(sec=0, nanosec=0)

    def now(self):
        return self._Now()


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.logger = FakeLogger()
        self.publishers = []
        self.subscriptions = []
        self.clients = []
        self.services = []
        self.timers = []
        self.parameters = {}

    def create_publisher(self, _message_type, topic, qos):
        publisher = FakePublisher(topic, qos)
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

    def create_client(self, service_type, service_name):
        client = FakeClient(service_type, service_name)
        self.clients.append(client)
        return client

    def create_service(self, service_type, service_name, callback):
        service = {
            "service_type": service_type,
            "service_name": service_name,
            "callback": callback,
        }
        self.services.append(service)
        return service

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


class FakeOdometry:
    def __init__(self):
        self.pose = types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.0, y=0.0),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )
        self.twist = types.SimpleNamespace(
            twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=0.0),
            )
        )


class FakeOccupancyGrid:
    def __init__(
        self,
        width=4,
        height=4,
        resolution=1.0,
        origin_x=-2.0,
        origin_y=-2.0,
        data=None,
    ):
        self.info = types.SimpleNamespace(
            width=width,
            height=height,
            resolution=resolution,
            origin=types.SimpleNamespace(
                position=types.SimpleNamespace(x=origin_x, y=origin_y),
            ),
        )
        self.data = data if data is not None else [0] * (width * height)


class FakeString:
    def __init__(self):
        self.data = ""


class FakeEnvironmentCell:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.occupancy = ""
        self.has_environment = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.wind_x = 0.0
        self.wind_y = 0.0
        self.wave_height = 0.0
        self.confidence = 0.0


class FakeGenerateEnvironmentField:
    class Request:
        def __init__(self):
            self.frame_id = ""
            self.cell_size_m = 0.0
            self.generation_controls_json = ""

    class Response:
        def __init__(self):
            self.success = False
            self.environment_status = ""
            self.environment_source = ""
            self.confidence = 0.0
            self.cells = []
            self.warnings = []


class FakeNavigateToPose:
    class Goal:
        def __init__(self):
            self.pose = None


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
        self.nav_to_pose_client = FakeNavToPoseClient()

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
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(
        BEST_EFFORT="best_effort",
        RELIABLE="reliable",
    )
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL="transient_local")

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseStamped = FakePoseStamped
    geometry_msgs_msg.TwistStamped = FakeTwistStamped

    nav2_simple_commander = types.ModuleType("nav2_simple_commander")
    nav2_robot_navigator = types.ModuleType("nav2_simple_commander.robot_navigator")
    nav2_robot_navigator.BasicNavigator = FakeBasicNavigator
    nav2_robot_navigator.TaskResult = FakeTaskResult

    nav2_msgs = types.ModuleType("nav2_msgs")
    nav2_msgs_action = types.ModuleType("nav2_msgs.action")
    nav2_msgs_action.NavigateToPose = FakeNavigateToPose

    action_msgs = types.ModuleType("action_msgs")
    action_msgs_msg = types.ModuleType("action_msgs.msg")
    action_msgs_msg.GoalStatus = types.SimpleNamespace(
        STATUS_SUCCEEDED="succeeded",
        STATUS_CANCELED="canceled",
    )

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.LaserScan = FakeLaserScan

    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")
    nav_msgs_msg.Odometry = FakeOdometry
    nav_msgs_msg.OccupancyGrid = FakeOccupancyGrid

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = FakeString

    interfaces = types.ModuleType("my_tb3_world_interfaces")
    interfaces_msg = types.ModuleType("my_tb3_world_interfaces.msg")
    interfaces_msg.EnvironmentCell = FakeEnvironmentCell
    interfaces_srv = types.ModuleType("my_tb3_world_interfaces.srv")
    interfaces_srv.GenerateEnvironmentField = FakeGenerateEnvironmentField

    sys.modules.update(
        {
            "rclpy": rclpy,
            "rclpy.node": rclpy_node,
            "rclpy.qos": rclpy_qos,
            "geometry_msgs": geometry_msgs,
            "geometry_msgs.msg": geometry_msgs_msg,
            "nav2_simple_commander": nav2_simple_commander,
            "nav2_simple_commander.robot_navigator": nav2_robot_navigator,
            "nav2_msgs": nav2_msgs,
            "nav2_msgs.action": nav2_msgs_action,
            "action_msgs": action_msgs,
            "action_msgs.msg": action_msgs_msg,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": sensor_msgs_msg,
            "nav_msgs": nav_msgs,
            "nav_msgs.msg": nav_msgs_msg,
            "std_msgs": std_msgs,
            "std_msgs.msg": std_msgs_msg,
            "my_tb3_world_interfaces": interfaces,
            "my_tb3_world_interfaces.msg": interfaces_msg,
            "my_tb3_world_interfaces.srv": interfaces_srv,
        }
    )


install_ros_stubs()


class MyTb3WorldNodeTests(unittest.TestCase):
    def _odom_at(self, x, y):
        odom = FakeOdometry()
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        return odom

    def _latest_dashboard(self, node):
        return json.loads(node.dashboard_pub.messages[-1].data)

    def _string_msg(self, payload):
        msg = FakeString()
        msg.data = json.dumps(payload)
        return msg

    def test_environment_field_generation_masks_blocked_unknown_cells(self):
        module = importlib.import_module("my_tb3_world.environment_field")
        grid = FakeOccupancyGrid(
            data=[
                0, 100, -1, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
            ],
        )

        cells = module.build_environment_cells(grid, 1.0)

        self.assertEqual(len(cells), 16)
        self.assertEqual(cells[0]["occupancy"], "free")
        self.assertTrue(cells[0]["has_environment"])
        self.assertEqual(cells[4]["occupancy"], "blocked")
        self.assertFalse(cells[4]["has_environment"])
        self.assertEqual(cells[4]["current_x"], 0.0)
        self.assertEqual(cells[8]["occupancy"], "unknown")
        self.assertFalse(cells[8]["has_environment"])

    def test_environment_field_layers_vary_current_more_than_wind(self):
        module = importlib.import_module("my_tb3_world.environment_field")
        grid = FakeOccupancyGrid(width=4, height=4, resolution=1.0)

        cells = module.build_environment_cells(grid, 1.0)

        current_x_values = [cell["current_x"] for cell in cells]
        wind_x_values = [cell["wind_x"] for cell in cells]
        self.assertGreater(max(current_x_values) - min(current_x_values), 0.1)
        self.assertLess(max(wind_x_values) - min(wind_x_values), 0.03)

    def test_environment_field_currents_change_with_field_time(self):
        module = importlib.import_module("my_tb3_world.environment_field")
        grid = FakeOccupancyGrid(width=4, height=4, resolution=1.0)

        early = module.build_environment_cells(grid, 1.0, {"field_time_sec": 0.0})
        later = module.build_environment_cells(grid, 1.0, {"field_time_sec": 30.0})

        early_vectors = [(cell["current_x"], cell["current_y"]) for cell in early]
        later_vectors = [(cell["current_x"], cell["current_y"]) for cell in later]
        self.assertNotEqual(early_vectors, later_vectors)

    def test_environment_field_defaults_make_visible_eddies(self):
        module = importlib.import_module("my_tb3_world.environment_field")
        grid = FakeOccupancyGrid(width=5, height=5, resolution=1.0)

        cells = module.build_environment_cells(grid, 1.0, {"field_time_sec": 12.0})
        current_magnitudes = [
            math.hypot(cell["current_x"], cell["current_y"])
            for cell in cells
            if cell["has_environment"]
        ]

        self.assertGreater(max(current_magnitudes) - min(current_magnitudes), 0.35)

    def test_environment_field_vortex_rotates_around_center(self):
        module = importlib.import_module("my_tb3_world.environment_field")
        grid = FakeOccupancyGrid(width=4, height=4, resolution=1.0)
        controls = {
            "current_strength": 0.0,
            "current_spatial_variation": 0.0,
            "current_shear": 0.0,
            "current_noise_strength": 0.0,
            "wind_x": 0.0,
            "wind_y": 0.0,
            "wind_spatial_variation": 0.0,
            "wave_height": 0.0,
            "wave_spatial_variation": 0.0,
            "tide_strength": 0.0,
            "vortex_strength": 1.0,
            "vortex_x": 0.0,
            "vortex_y": 0.0,
            "vortex_radius": 3.0,
            "vortex2_strength": 0.0,
        }

        cells = {
            (cell["x"], cell["y"]): cell
            for cell in module.build_environment_cells(grid, 1.0, controls)
        }

        self.assertGreater(cells[(-0.5, -0.5)]["current_x"], 0.0)
        self.assertLess(cells[(0.5, 0.5)]["current_x"], 0.0)
        self.assertLess(cells[(-0.5, -0.5)]["current_y"], 0.0)
        self.assertGreater(cells[(0.5, 0.5)]["current_y"], 0.0)

    def test_environment_field_redirects_current_away_from_land(self):
        module = importlib.import_module("my_tb3_world.environment_field")
        grid = FakeOccupancyGrid(
            width=3,
            height=3,
            resolution=1.0,
            origin_x=0.0,
            origin_y=0.0,
            data=[
                0, 0, 0,
                0, 100, 0,
                0, 0, 0,
            ],
        )
        controls = {
            "current_strength": 1.0,
            "current_heading_deg": 180.0,
            "current_spatial_variation": 0.0,
            "current_shear": 0.0,
            "current_noise_strength": 0.0,
            "wind_x": 0.0,
            "wind_y": 0.0,
            "wind_spatial_variation": 0.0,
            "wave_height": 1.0,
            "wave_spatial_variation": 0.0,
            "tide_strength": 0.0,
            "vortex_strength": 0.0,
            "vortex2_strength": 0.0,
            "shore_influence_radius": 1.5,
        }

        cells = {
            (cell["x"], cell["y"]): cell
            for cell in module.build_environment_cells(grid, 1.0, controls)
        }

        right_of_land = cells[(2.5, 1.5)]
        self.assertGreaterEqual(right_of_land["current_x"], 0.0)
        self.assertGreater(abs(right_of_land["current_y"]), 0.0)

    def test_environment_field_handles_two_blocked_incoming_cells(self):
        module = importlib.import_module("my_tb3_world.environment_field")
        grid = FakeOccupancyGrid(
            width=4,
            height=3,
            resolution=1.0,
            origin_x=0.0,
            origin_y=0.0,
            data=[
                0, 0, 0, 0,
                0, 100, 100, 0,
                0, 0, 0, 0,
            ],
        )
        controls = {
            "current_strength": 0.8,
            "current_heading_deg": 0.0,
            "current_noise_strength": 0.0,
            "vortex_strength": 0.0,
            "vortex2_strength": 0.0,
            "shore_influence_radius": 1.5,
        }

        cells = {
            (cell["x"], cell["y"]): cell
            for cell in module.build_environment_cells(grid, 1.0, controls)
        }

        self.assertFalse(cells[(1.5, 1.5)]["has_environment"])
        self.assertFalse(cells[(2.5, 1.5)]["has_environment"])
        left_of_blocked = cells[(0.5, 1.5)]
        self.assertLessEqual(left_of_blocked["current_x"], 0.0)
        self.assertGreater(abs(left_of_blocked["current_y"]), 0.0)
        self.assertTrue(math.isfinite(cells[(0.5, 0.5)]["current_x"]))
        self.assertTrue(math.isfinite(cells[(3.5, 2.5)]["current_y"]))

    def test_environment_field_keeps_wave_stable_near_land(self):
        module = importlib.import_module("my_tb3_world.environment_field")
        grid = FakeOccupancyGrid(
            width=5,
            height=1,
            resolution=1.0,
            origin_x=0.0,
            origin_y=0.0,
            data=[100, 0, 0, 0, 0],
        )
        controls = {
            "current_strength": 0.0,
            "current_spatial_variation": 0.0,
            "current_shear": 0.0,
            "current_noise_strength": 0.0,
            "wave_height": 1.0,
            "wave_spatial_variation": 0.0,
            "tide_strength": 0.0,
            "vortex_strength": 0.0,
            "vortex2_strength": 0.0,
            "shore_influence_radius": 3.0,
        }

        cells = {
            (cell["x"], cell["y"]): cell
            for cell in module.build_environment_cells(grid, 1.0, controls)
        }

        self.assertEqual(cells[(1.5, 0.5)]["wave_height"], cells[(4.5, 0.5)]["wave_height"])

    def test_environment_field_tide_modifies_wave_not_current(self):
        module = importlib.import_module("my_tb3_world.environment_field")
        grid = FakeOccupancyGrid(width=2, height=1, resolution=1.0, origin_x=0.0, origin_y=0.0)
        controls = {
            "current_strength": 0.4,
            "current_heading_deg": 0.0,
            "current_noise_strength": 0.0,
            "vortex_strength": 0.0,
            "vortex2_strength": 0.0,
            "wind_spatial_variation": 0.0,
            "wave_height": 0.2,
            "wave_spatial_variation": 0.0,
        }

        no_tide = module.build_environment_cells(grid, 1.0, {**controls, "tide_strength": 0.0})
        tide = module.build_environment_cells(grid, 1.0, {**controls, "tide_strength": 0.5, "tide_phase": 1.0})

        self.assertEqual(no_tide[0]["current_x"], tide[0]["current_x"])
        self.assertEqual(no_tide[0]["current_y"], tide[0]["current_y"])
        self.assertGreater(tide[0]["wave_height"], no_tide[0]["wave_height"])

    def test_environment_field_opensimplex_current_varies_across_space(self):
        module = importlib.import_module("my_tb3_world.environment_field")
        grid = FakeOccupancyGrid(width=5, height=5, resolution=1.0)

        cells = module.build_environment_cells(
            grid,
            1.0,
            {
                "current_strength": 0.0,
                "current_noise_strength": 0.5,
                "vortex_strength": 0.0,
                "vortex2_strength": 0.0,
            },
        )

        current_vectors = {
            (cell["current_x"], cell["current_y"])
            for cell in cells
            if cell["has_environment"]
        }
        self.assertGreater(len(current_vectors), 4)

    def test_environment_grid_uses_map_bounds_and_cell_size(self):
        module = importlib.import_module("my_tb3_world.environment_field")
        grid = FakeOccupancyGrid(
            width=5,
            height=3,
            resolution=1.0,
            origin_x=-2.0,
            origin_y=-1.0,
        )

        bounds = module.map_bounds(grid)
        map_cells = module.build_map_cells(grid, 1.0)
        field_cells = module.build_environment_cells(grid, 1.0)

        self.assertEqual(
            bounds,
            {"min_x": -2.0, "min_y": -1.0, "max_x": 3.0, "max_y": 2.0},
        )
        self.assertEqual(len(map_cells), 15)
        self.assertEqual(len(field_cells), 15)
        self.assertEqual(map_cells[0], {"x": -1.5, "y": -0.5, "occupancy": "free"})
        self.assertEqual(map_cells[-1], {"x": 2.5, "y": 1.5, "occupancy": "free"})

    def test_physical_particle_seeding_is_deterministic(self):
        module = importlib.import_module("my_tb3_world.debris_particles")

        first = module.seed_physical_particles(random_seed=23)
        second = module.seed_physical_particles(random_seed=23)

        self.assertEqual(first, second)
        self.assertEqual(len(first), 100)
        self.assertEqual(first[0]["status"], "active")
        self.assertEqual(first[0]["material"], "plastic")
        self.assertIn("vx", first[0])
        self.assertNotEqual((first[0]["x"], first[0]["y"]), (0.88, 1.04))

    def test_physical_particle_seed_spans_free_cells(self):
        module = importlib.import_module("my_tb3_world.debris_particles")
        cells = [
            {"x": float(index), "y": 0.0, "occupancy": "free", "has_environment": True}
            for index in range(100)
        ]

        particles = module.seed_physical_particles(cells, count=10, random_seed=23)
        initial_x = [particle["initial_x"] for particle in particles]

        self.assertLess(min(initial_x), 10.0)
        self.assertGreater(max(initial_x), 90.0)

    def test_physical_particle_material_response_differs_by_material(self):
        module = importlib.import_module("my_tb3_world.debris_particles")
        field_cell = {
            "current_x": 0.2,
            "current_y": 0.0,
            "wind_x": 0.2,
            "wind_y": 0.0,
            "wave_height": 0.0,
        }
        particles = [
            {
                "id": "physical_particle_a",
                "material": "plastic",
                "x": 0.0,
                "y": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "status": "active",
            },
            {
                "id": "physical_particle_c",
                "material": "metal",
                "x": 0.0,
                "y": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "status": "active",
            },
        ]

        plastic = module.advance_particle(particles[0], [particles[0]], field_cell, 1.0)
        dense = module.advance_particle(particles[1], [particles[1]], field_cell, 1.0)

        self.assertGreater(plastic["x"], dense["x"])

    def test_digital_twin_map_cells_use_map_derived_grid(self):
        module = importlib.import_module("my_tb3_world.digital_twin_state_node")
        node = module.DigitalTwinStateNode()
        node._map_cb(FakeOccupancyGrid(
            width=5,
            height=3,
            resolution=1.0,
            origin_x=-2.0,
            origin_y=-1.0,
        ))

        self.assertTrue(node.map_received)
        self.assertEqual(len(node.map_cells), 375)
        self.assertEqual(node.map_cells[0], {"x": -1.9, "y": -0.9, "occupancy": "free"})
        self.assertEqual(node.map_cells[-1], {"x": 2.9, "y": 1.9, "occupancy": "free"})

    def test_digital_twin_uses_environment_cells_when_map_is_missing(self):
        module = importlib.import_module("my_tb3_world.digital_twin_state_node")
        node = module.DigitalTwinStateNode()
        node._env_obs_cb(self._string_msg({
            "cells": [
                {
                    "x": 0.0,
                    "y": 0.0,
                    "occupancy": "free",
                    "current_x": 0.2,
                    "current_y": 0.1,
                    "wind_x": 0.03,
                    "wind_y": 0.01,
                    "wave_height": 0.4,
                },
                {
                    "x": 0.2,
                    "y": 0.0,
                    "occupancy": "blocked",
                    "has_environment": False,
                    "current_x": 9.0,
                    "current_y": 9.0,
                },
            ],
            "environment_status": "ready",
            "environment_source": "preset_fallback",
        }))

        node._publish()

        twin_state = json.loads(node.twin_pub.messages[-1].data)
        cells = twin_state["map"]["cells"]
        self.assertEqual(twin_state["sync_status"], "partial (waiting: robot_state, base_pose, map)")
        self.assertEqual(len(cells), 2)
        self.assertEqual(cells[0]["occupancy"], "free")
        self.assertEqual(cells[0]["current_x"], 0.2)
        self.assertEqual(cells[0]["wave_height"], 0.4)
        self.assertEqual(cells[1]["occupancy"], "blocked")
        self.assertNotIn("current_x", cells[1])

    def test_digital_twin_contract_keeps_density_and_debug_truth_out(self):
        module = importlib.import_module("my_tb3_world.digital_twin_state_node")
        node = module.DigitalTwinStateNode()
        node._map_cb(FakeOccupancyGrid(
            width=2,
            height=2,
            resolution=1.0,
            origin_x=0.0,
            origin_y=0.0,
        ))
        node._env_obs_cb(self._string_msg({
            "environment_status": "ready",
            "environment_source": "preset_fallback",
            "cells": [
                {
                    "x": 0.1,
                    "y": 0.1,
                    "occupancy": "free",
                    "current_x": 0.2,
                    "current_y": 0.1,
                    "wind_x": 0.03,
                    "wind_y": 0.01,
                    "wave_height": 0.4,
                    "density": 0.9,
                    "physical_particles": [{"id": "hidden_truth"}],
                },
            ],
            "density_cells": [{"x": 0.1, "y": 0.1, "density": 1.0}],
            "physical_particles": [{"id": "hidden_truth"}],
        }))
        node._base_pose_cb(self._string_msg({
            "schema": "dtas.base_pose.v1",
            "pose": {"x": -1.0, "y": -1.0, "yaw": 0.5},
            "status": "known",
        }))
        node._robot_state_cb(self._string_msg({
            "schema": "dtas.robot_state.v1",
            "pose": {"x": 0.5, "y": 0.6, "yaw": 0.1},
            "velocity": 0.2,
            "odom_distance_m": 3.4,
            "fuel_level": 0.7,
            "storage_fill": 0.3,
            "storage_count": 1,
            "storage_capacity": 3,
            "at_base": False,
        }))
        node._collection_event_cb(self._string_msg({
            "schema": "dtas.collection_event.v1",
            "location": {"x": 0.5, "y": 0.6},
            "count": 2,
            "materials": {"plastic": 1, "wood": 1},
        }))

        node._publish()

        twin_state = json.loads(node.twin_pub.messages[-1].data)
        self.assertEqual(twin_state["schema"], "dtas.twin_state.v1")
        self.assertEqual(twin_state["source"], "digital_twin_state_node")
        self.assertEqual(twin_state["sync_status"], "ready")
        self.assertEqual(twin_state["base"]["pose"]["x"], -1.0)
        self.assertEqual(twin_state["robot"]["pose"], {"x": 0.5, "y": 0.6, "yaw": 0.1})
        self.assertEqual(twin_state["robot"]["fuel_level"], 0.7)
        self.assertEqual(twin_state["material_evidence"], {"plastic": 1, "wood": 1, "metal": 0})
        self.assertEqual(len(twin_state["collection_events"]), 1)
        self.assertEqual(twin_state["environment_status"], "ready")
        self.assertEqual(twin_state["environment_source"], "preset_fallback")
        self.assertNotIn("density_cells", twin_state)
        self.assertNotIn("physical_particles", twin_state)
        for cell in twin_state["map"]["cells"]:
            self.assertNotIn("density", cell)
            self.assertNotIn("physical_particles", cell)

    def test_digital_twin_debug_reset_clears_material_evidence(self):
        module = importlib.import_module("my_tb3_world.digital_twin_state_node")
        node = module.DigitalTwinStateNode()
        node._collection_event_cb(self._string_msg({
            "schema": "dtas.collection_event.v1",
            "location": {"x": 0.5, "y": 0.6},
            "count": 2,
            "materials": {"plastic": 1, "wood": 1},
        }))

        node._debug_reset_cb(self._string_msg({"scope": "all"}))
        node._publish()

        twin_state = json.loads(node.twin_pub.messages[-1].data)
        self.assertEqual(twin_state["material_evidence"], {"plastic": 0, "wood": 0, "metal": 0})
        self.assertEqual(twin_state["collection_events"], [])

    def test_robot_state_debug_reset_clears_storage_and_fuel(self):
        module = importlib.import_module("my_tb3_world.robot_state_node")
        node = module.RobotStateNode()
        node.storage_count = 3
        node.fuel_pct = 12.0
        node.fuel_low = True

        node._debug_reset_cb(self._string_msg({"scope": "all"}))

        self.assertEqual(node.storage_count, 0)
        self.assertEqual(node.fuel_pct, 100.0)
        self.assertFalse(node.fuel_low)

    def test_environment_generator_service_waits_for_map(self):
        module = importlib.import_module("my_tb3_world.environment_generator_node")
        node = module.EnvironmentGeneratorNode()
        request = FakeGenerateEnvironmentField.Request()
        request.frame_id = "map"
        request.cell_size_m = 1.0
        response = FakeGenerateEnvironmentField.Response()

        response = node._generate_environment_field(request, response)

        self.assertFalse(response.success)
        self.assertEqual(response.environment_status, "waiting_for_map")
        self.assertEqual(response.cells, [])
        self.assertIn("no /map received", response.warnings)

    def test_environment_generator_service_masks_non_free_map_cells(self):
        module = importlib.import_module("my_tb3_world.environment_generator_node")
        node = module.EnvironmentGeneratorNode()
        node.parameters["current_strength"] = 0.6
        node.parameters["current_heading_deg"] = 0.0
        node.parameters["current_noise_strength"] = 0.0
        node.parameters["vortex_strength"] = 0.0
        node.parameters["vortex2_strength"] = 0.0
        request = FakeGenerateEnvironmentField.Request()
        request.frame_id = "map"
        request.cell_size_m = 1.0
        request.generation_controls_json = '{"wind_x": 0.4}'
        response = FakeGenerateEnvironmentField.Response()
        node._map_cb(FakeOccupancyGrid(
            data=[
                0, 100, -1, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
            ],
        ))

        response = node._generate_environment_field(request, response)

        self.assertTrue(response.success)
        self.assertEqual(response.environment_status, "ready")
        self.assertEqual(len(response.cells), 16)
        self.assertTrue(response.cells[0].has_environment)
        self.assertGreater(response.cells[0].current_x, 0.4)
        self.assertAlmostEqual(response.cells[0].wind_x, 0.386, places=3)
        self.assertEqual(response.cells[4].occupancy, "blocked")
        self.assertFalse(response.cells[4].has_environment)
        self.assertEqual(response.cells[8].occupancy, "unknown")
        self.assertFalse(response.cells[8].has_environment)

    def test_debris_prediction_drifts_from_twin_environment_cells(self):
        module = importlib.import_module("my_tb3_world.debris_prediction_node")
        node = module.DebrisPredictionNode()
        node.prediction_debris_count = 1
        node.prediction_drift_scale = 1.0
        node.prediction_particles = [
            {
                "id": "prediction_particle_test",
                "x": 0.88,
                "y": 1.04,
                "initial_x": 0.88,
                "initial_y": 1.04,
                "material": "plastic",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            }
        ]
        twin_state = {
            "map": {
                "cells": [
                    {
                        "x": 0.88,
                        "y": 1.04,
                        "occupancy": "free",
                        "current_x": 0.1,
                        "current_y": 0.2,
                    },
                    {
                        "x": 0.98,
                        "y": 1.24,
                        "occupancy": "free",
                        "current_x": 0.1,
                        "current_y": 0.2,
                    },
                ]
            },
            "collection_events": [],
        }

        node._twin_state_cb(self._string_msg(twin_state))
        node.prediction_particles = [
            {
                "id": "prediction_particle_test",
                "x": 0.88,
                "y": 1.04,
                "initial_x": 0.88,
                "initial_y": 1.04,
                "material": "plastic",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            }
        ]
        node._publish()

        prediction = node.prediction_particles[0]
        density = json.loads(node.density_pub.messages[-1].data)
        active_cells = [cell for cell in density["cells"] if cell["density"] > 0.0]
        self.assertEqual(prediction["x"], 0.98)
        self.assertEqual(prediction["y"], 1.24)
        self.assertEqual(prediction["vx"], 0.1)
        self.assertEqual(density["source"], "debris_prediction_node")
        self.assertEqual(density["prediction_particle_count"], 1)
        self.assertTrue(density["prediction_drift_enabled"])
        self.assertEqual(len(active_cells), 1)
        self.assertAlmostEqual(sum(cell["density"] for cell in density["density_cells"]), 1.0, places=5)
        self.assertNotIn("cluster_x", active_cells[0])
        self.assertNotIn("prediction_particles", density)

        dashboard = json.loads(node.prediction_dashboard_pub.messages[-1].data)
        self.assertEqual(dashboard["schema"], "dtas.prediction_dashboard.v1")
        self.assertTrue(dashboard["debug_only"])
        self.assertTrue(dashboard["forbidden_as_planner_input"])
        self.assertEqual(len(dashboard["prediction_particles"]), 1)
        self.assertEqual(dashboard["prediction_particles"][0]["x"], 0.98)
        self.assertEqual(dashboard["prediction_particles"][0]["material"], "plastic")

    def test_debris_density_map_contract_is_planner_facing_only(self):
        module = importlib.import_module("my_tb3_world.debris_prediction_node")
        node = module.DebrisPredictionNode()
        node.prediction_debris_count = 2
        node.prediction_drift_enabled = False
        node.twin_map_cells = [
            {"x": 0.0, "y": 0.0, "occupancy": "free"},
            {"x": 1.0, "y": 0.0, "occupancy": "free"},
        ]
        node.prediction_particles = [
            {
                "id": "prediction_particle_a",
                "x": 0.0,
                "y": 0.0,
                "initial_x": 0.0,
                "initial_y": 0.0,
                "material": "plastic",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            },
            {
                "id": "prediction_particle_b",
                "x": 1.0,
                "y": 0.0,
                "initial_x": 1.0,
                "initial_y": 0.0,
                "material": "wood",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            },
        ]

        node._publish()

        density = json.loads(node.density_pub.messages[-1].data)
        self.assertEqual(density["schema"], "dtas.debris_density_map.v1")
        self.assertEqual(density["source"], "debris_prediction_node")
        self.assertEqual(density["frame_id"], "map")
        self.assertEqual(density["status"], "ready")
        self.assertEqual(density["cells"], density["density_cells"])
        self.assertAlmostEqual(
            sum(cell["density"] for cell in density["density_cells"]),
            1.0,
            places=5,
        )
        self.assertNotIn("prediction_particles", density)
        self.assertNotIn("physical_particles", density)
        self.assertNotIn("material_belief", density)
        self.assertNotIn("robot", density)
        self.assertNotIn("map", density)
        for cell in density["density_cells"]:
            self.assertEqual(set(cell.keys()), {"x", "y", "density"})
            self.assertNotIn("material", cell)
            self.assertNotIn("cell_id", cell)
            self.assertGreaterEqual(cell["density"], 0.0)
            self.assertLessEqual(cell["density"], 1.0)

        dashboard = json.loads(node.prediction_dashboard_pub.messages[-1].data)
        self.assertIn("prediction_particles", dashboard)
        self.assertIn("material_belief", dashboard)

    def test_debris_prediction_washes_out_into_blocked_twin_cell(self):
        module = importlib.import_module("my_tb3_world.debris_prediction_node")
        node = module.DebrisPredictionNode()
        node.prediction_debris_count = 1
        node.prediction_drift_scale = 1.0
        node.prediction_particles = [
            {
                "id": "prediction_particle_blocked_test",
                "x": 0.0,
                "y": 0.0,
                "initial_x": 0.0,
                "initial_y": 0.0,
                "material": "plastic",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            }
        ]
        twin_state = {
            "map": {
                "cells": [
                    {
                        "x": 0.0,
                        "y": 0.0,
                        "occupancy": "free",
                        "current_x": 0.5,
                        "current_y": 0.0,
                    },
                    {
                        "x": 0.5,
                        "y": 0.0,
                        "occupancy": "blocked",
                    },
                ]
            },
            "collection_events": [],
        }

        node._twin_state_cb(self._string_msg(twin_state))
        node.prediction_particles = [
            {
                "id": "prediction_particle_blocked_test",
                "x": 0.0,
                "y": 0.0,
                "initial_x": 0.0,
                "initial_y": 0.0,
                "material": "plastic",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            }
        ]
        node._publish()

        density = json.loads(node.density_pub.messages[-1].data)
        self.assertEqual(len(node.prediction_particles), 1)
        self.assertEqual(node.prediction_particles[0]["status"], "active")
        self.assertEqual(density["prediction_counts"]["washed_out"], 1)
        self.assertEqual(density["prediction_counts"]["active"], 1)
        self.assertEqual(density["prediction_counts"]["respawned"], 1)

    def test_debris_prediction_waits_when_force_evidence_is_missing(self):
        module = importlib.import_module("my_tb3_world.debris_prediction_node")
        node = module.DebrisPredictionNode()
        node.prediction_debris_count = 1
        node.prediction_particles = [
            {
                "id": "prediction_particle_waiting_test",
                "x": 0.0,
                "y": 0.0,
                "initial_x": 0.0,
                "initial_y": 0.0,
                "material": "plastic",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            }
        ]
        node.twin_map_cells = [{"x": 0.0, "y": 0.0, "occupancy": "free"}]

        node._publish()

        prediction = node.prediction_particles[0]
        density = json.loads(node.density_pub.messages[-1].data)
        self.assertEqual(prediction["status"], "active")
        self.assertEqual(prediction["x"], 0.0)
        self.assertEqual(density["prediction_counts"]["active"], 1)
        self.assertAlmostEqual(sum(cell["density"] for cell in density["density_cells"]), 1.0)

    def test_debris_prediction_washes_out_outside_twin_map(self):
        module = importlib.import_module("my_tb3_world.debris_prediction_node")
        node = module.DebrisPredictionNode()
        node.prediction_debris_count = 1
        node.prediction_drift_scale = 1.0
        node.prediction_particles = [
            {
                "id": "prediction_particle_edge_test",
                "x": 0.0,
                "y": 0.0,
                "initial_x": 0.0,
                "initial_y": 0.0,
                "material": "plastic",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            }
        ]
        twin_state = {
            "map": {
                "cells": [
                    {
                        "x": 0.0,
                        "y": 0.0,
                        "occupancy": "free",
                        "current_x": 1.0,
                        "current_y": 0.0,
                    },
                ]
            },
            "collection_events": [],
        }

        node._twin_state_cb(self._string_msg(twin_state))
        node.prediction_particles = [
            {
                "id": "prediction_particle_edge_test",
                "x": 0.0,
                "y": 0.0,
                "initial_x": 0.0,
                "initial_y": 0.0,
                "material": "plastic",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            }
        ]
        node._publish()

        density = json.loads(node.density_pub.messages[-1].data)
        self.assertEqual(len(node.prediction_particles), 1)
        self.assertEqual(node.prediction_particles[0]["status"], "active")
        self.assertEqual(density["prediction_counts"]["washed_out"], 1)
        self.assertEqual(density["prediction_counts"]["respawned"], 1)

    def test_debris_prediction_collection_event_removes_density(self):
        module = importlib.import_module("my_tb3_world.debris_prediction_node")
        node = module.DebrisPredictionNode()
        node.prediction_debris_count = 3
        node.prediction_drift_enabled = False
        node.prediction_particles = [
            {
                "id": f"prediction_particle_{index}",
                "x": 0.0,
                "y": 0.0,
                "initial_x": 0.0,
                "initial_y": 0.0,
                "material": "plastic",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            }
            for index in range(3)
        ]
        node.twin_map_cells = [{"x": 0.0, "y": 0.0, "occupancy": "free"}]

        node._collection_event_cb(self._string_msg({
            "stamp": "test",
            "location": {"x": 0.0, "y": 0.0},
            "count": 2,
        }))
        node._publish()

        density = json.loads(node.density_pub.messages[-1].data)
        self.assertEqual(density["clusters_remaining"], 3)
        self.assertEqual(density["prediction_counts"]["active"], 3)
        self.assertEqual(density["prediction_counts"]["observed_removed"], 2)
        self.assertEqual(density["prediction_counts"]["respawned"], 2)
        self.assertAlmostEqual(sum(cell["density"] for cell in density["density_cells"]), 1.0, places=5)
        dashboard = json.loads(node.prediction_dashboard_pub.messages[-1].data)
        self.assertEqual(dashboard["lifetime_counts"]["observed_removed"], 2)
        self.assertEqual(dashboard["lifetime_counts"]["respawned"], 2)
        self.assertEqual(dashboard["events"][-1]["type"], "respawned")

    def test_debris_prediction_collection_event_updates_belief_distribution(self):
        module = importlib.import_module("my_tb3_world.debris_prediction_node")
        node = module.DebrisPredictionNode()
        node.prediction_debris_count = 4
        node.prediction_drift_enabled = False
        node.twin_map_cells = [
            {"x": 0.0, "y": 0.0, "occupancy": "free"},
            {"x": 1.0, "y": 0.0, "occupancy": "free"},
            {"x": 2.0, "y": 0.0, "occupancy": "free"},
            {"x": 3.0, "y": 0.0, "occupancy": "free"},
        ]
        node.prediction_particles = [
            {
                "id": f"prediction_particle_{index}",
                "x": float(index),
                "y": 0.0,
                "initial_x": float(index),
                "initial_y": 0.0,
                "material": "plastic",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            }
            for index in range(4)
        ]
        event = {
            "stamp": "belief-test",
            "location": {"x": 0.0, "y": 0.0},
            "count": 1,
        }

        node._collection_event_cb(self._string_msg(event))
        node._collection_event_cb(self._string_msg(event))
        node._publish()

        density = json.loads(node.density_pub.messages[-1].data)
        cells = {cell["x"]: cell["density"] for cell in density["density_cells"]}
        active_particles = [
            particle for particle in node.prediction_particles
            if particle["status"] == "active"
        ]
        density_cells = density["density_cells"]
        for particle in active_particles:
            nearest = min(
                density_cells,
                key=lambda cell: (
                    (cell["x"] - particle["x"]) ** 2 +
                    (cell["y"] - particle["y"]) ** 2
                ),
            )
            self.assertGreater(nearest["density"], 0.0)
        self.assertEqual(density["prediction_counts"]["observed_removed"], 1)
        self.assertEqual(density["prediction_counts"]["active"], 4)
        self.assertAlmostEqual(sum(cells.values()), 1.0, places=5)
        dashboard = json.loads(node.prediction_dashboard_pub.messages[-1].data)
        self.assertEqual(dashboard["belief_model"]["type"], "active_prediction_particle_density")
        self.assertTrue(dashboard["belief_model"]["normalized_for_planner_reward"])
        self.assertTrue(dashboard["belief_model"]["counts_are_debug_status"])

    def test_debris_prediction_density_does_not_leave_collection_hole_over_active_particle(self):
        module = importlib.import_module("my_tb3_world.debris_prediction_node")
        node = module.DebrisPredictionNode()
        node.prediction_debris_count = 2
        node.prediction_drift_enabled = False
        node.twin_map_cells = [
            {"x": 0.0, "y": 0.0, "occupancy": "free"},
            {"x": 1.0, "y": 0.0, "occupancy": "free"},
        ]
        node.prediction_particles = [
            {
                "id": "prediction_particle_removed",
                "x": 0.0,
                "y": 0.0,
                "initial_x": 0.0,
                "initial_y": 0.0,
                "material": "plastic",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            },
            {
                "id": "prediction_particle_other",
                "x": 1.0,
                "y": 0.0,
                "initial_x": 1.0,
                "initial_y": 0.0,
                "material": "wood",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            },
        ]

        node._collection_event_cb(self._string_msg({
            "stamp": "hole-test",
            "location": {"x": 0.0, "y": 0.0},
            "count": 1,
        }))
        node.prediction_particles = [
            particle for particle in node.prediction_particles
            if particle["status"] == "active"
        ] + [{
            "id": "prediction_particle_back_in_cell",
            "x": 0.0,
            "y": 0.0,
            "initial_x": 0.0,
            "initial_y": 0.0,
            "material": "unknown",
            "vx": 0.0,
            "vy": 0.0,
            "ax": 0.0,
            "ay": 0.0,
            "status": "active",
        }]
        node._publish()

        density = json.loads(node.density_pub.messages[-1].data)
        cells = {cell["x"]: cell["density"] for cell in density["density_cells"]}
        self.assertGreater(cells[0.0], 0.0)
        self.assertGreater(cells[1.0], 0.0)
        self.assertAlmostEqual(sum(cells.values()), 1.0, places=5)

    def test_debris_prediction_material_prior_starts_unknown(self):
        module = importlib.import_module("my_tb3_world.debris_prediction_node")
        node = module.DebrisPredictionNode()

        node._publish()

        dashboard = json.loads(node.prediction_dashboard_pub.messages[-1].data)
        probabilities = dashboard["material_belief"]["probabilities"]
        self.assertEqual(probabilities["unknown"], 1.0)
        self.assertEqual(probabilities["plastic"], 0.0)
        self.assertEqual(probabilities["wood"], 0.0)
        self.assertEqual(probabilities["metal"], 0.0)
        self.assertTrue(all(
            particle["material"] == "unknown"
            for particle in dashboard["prediction_particles"]
        ))

    def test_debris_prediction_seed_spans_free_cells(self):
        module = importlib.import_module("my_tb3_world.debris_prediction_node")
        cells = [
            {"x": float(index), "y": 0.0, "occupancy": "free"}
            for index in range(100)
        ]

        particles = module.seed_prediction_particles(cells, count=10, random_seed=23)
        initial_x = [particle["initial_x"] for particle in particles]

        self.assertLess(min(initial_x), 10.0)
        self.assertGreater(max(initial_x), 90.0)

    def test_debris_prediction_material_evidence_biases_internal_particles_only(self):
        module = importlib.import_module("my_tb3_world.debris_prediction_node")
        node = module.DebrisPredictionNode()
        node.prediction_debris_count = 20
        twin_state = {
            "map": {
                "cells": [
                    {"x": float(index), "y": 0.0, "occupancy": "free"}
                    for index in range(4)
                ]
            },
            "material_evidence": {"metal": 20},
            "collection_events": [],
        }

        node._twin_state_cb(self._string_msg(twin_state))
        node._publish()

        material_counts = {}
        for particle in node.prediction_particles:
            material_counts[particle["material"]] = material_counts.get(particle["material"], 0) + 1
        self.assertGreater(material_counts.get("metal", 0), material_counts.get("plastic", 0))
        self.assertGreater(material_counts.get("metal", 0), material_counts.get("wood", 0))

        density = json.loads(node.density_pub.messages[-1].data)
        self.assertNotIn("material_belief", density)
        self.assertFalse(any("material" in cell for cell in density["density_cells"]))

        dashboard = json.loads(node.prediction_dashboard_pub.messages[-1].data)
        belief = dashboard["material_belief"]
        self.assertTrue(belief["debug_only"])
        self.assertEqual(belief["prior"], "unknown")
        self.assertGreater(belief["probabilities"]["metal"], belief["probabilities"]["plastic"])

    def test_debris_prediction_debug_reset_reseeds_100_particles(self):
        module = importlib.import_module("my_tb3_world.debris_prediction_node")
        node = module.DebrisPredictionNode()
        node.prediction_particles = []
        node.material_evidence = {"plastic": 4, "wood": 2, "metal": 1}
        node._material_weights = node._material_belief_weights()

        node._debug_reset_cb(self._string_msg({"scope": "prediction"}))
        node._publish()

        dashboard = json.loads(node.prediction_dashboard_pub.messages[-1].data)
        self.assertEqual(len(node.prediction_particles), 100)
        self.assertTrue(all(p["status"] == "active" for p in node.prediction_particles))
        self.assertEqual(
            dashboard["material_belief"]["evidence"],
            {"plastic": 0, "wood": 0, "metal": 0},
        )
        self.assertEqual(dashboard["material_belief"]["probabilities"]["unknown"], 1.0)
        self.assertTrue(all(
            particle["material"] == "unknown"
            for particle in dashboard["prediction_particles"]
        ))

    def test_field_planner_targets_best_density_cell_without_cluster_fields(self):
        module = importlib.import_module("my_tb3_world.field_planner_node")
        node = module.FieldPlannerNode()
        node._twin_state_cb(self._string_msg({
            "robot": {
                "pose": {"x": 0.0, "y": 0.0},
                "fuel_level": 1.0,
                "storage_fill": 0.0,
            }
        }))
        node._density_map_cb(self._string_msg({
            "schema": "dtas.debris_density_map.v1",
            "prediction_counts": {"active": 100},
            "density_cells": [
                {"x": 0.0, "y": 0.0, "density": 0.1},
                {"x": 3.0, "y": 0.0, "density": 0.8},
            ],
        }))

        node._plan()

        self.assertEqual(node.goal_pub.topic, "/next_cell_goal")
        self.assertEqual(node.goal_pub.qos["durability"], "transient_local")
        self.assertEqual(node.goal_pub.qos["reliability"], "reliable")
        goal = json.loads(node.goal_pub.messages[-1].data)
        self.assertEqual(goal["mode"], "cleanup")
        self.assertEqual(goal["status"], "selected")
        self.assertEqual(goal["goal"]["x"], 3.0)
        self.assertEqual(goal["goal"]["y"], 0.0)
        self.assertEqual(goal["goal"]["frame_id"], "map")
        self.assertNotIn("cell_id", goal["goal"])
        self.assertEqual(goal["reason"], "highest utility reachable cell with return reserve")
        self.assertTrue(goal["return_feasible"])
        self.assertEqual(
            set(goal["components"]),
            {
                "density_reward",
                "travel_cost",
                "storage_penalty",
                "fuel_penalty",
                "map_risk",
                "return_cost",
                "fuel_margin",
            },
        )
        self.assertGreater(goal["components"]["density_reward"], goal["components"]["travel_cost"])

    def test_field_planner_demo_threshold_accepts_low_normalized_density(self):
        module = importlib.import_module("my_tb3_world.field_planner_node")
        twin_state = {
            "robot": {
                "pose": {"x": 0.0, "y": 0.0},
                "fuel_level": 1.0,
                "storage_fill": 0.0,
            },
            "map": {
                "known_area_ratio": 1.0,
                "cells": [{"x": 0.5, "y": 0.0, "occupancy": "free"}],
            },
        }
        density_map = {
            "schema": "dtas.debris_density_map.v1",
            "prediction_counts": {"active": 4},
            "density_cells": [{"x": 0.5, "y": 0.0, "density": 0.02}],
        }

        default_node = module.FieldPlannerNode()
        default_node._twin_state_cb(self._string_msg(twin_state))
        default_node._density_map_cb(self._string_msg(density_map))
        default_node._plan()

        default_goal = json.loads(default_node.goal_pub.messages[-1].data)
        self.assertEqual(default_goal["mode"], "idle")

        demo_node = module.FieldPlannerNode()
        demo_node._min_density_reward = 0.001
        demo_node._twin_state_cb(self._string_msg(twin_state))
        demo_node._density_map_cb(self._string_msg(density_map))
        demo_node._plan()

        demo_goal = json.loads(demo_node.goal_pub.messages[-1].data)
        self.assertEqual(demo_goal["mode"], "cleanup")
        self.assertEqual(demo_goal["goal"]["x"], 0.5)
        self.assertEqual(demo_goal["components"]["density_reward"], 0.02)

    def test_field_planner_rejects_density_cells_blocked_in_twin_map(self):
        module = importlib.import_module("my_tb3_world.field_planner_node")
        node = module.FieldPlannerNode()
        node._twin_state_cb(self._string_msg({
            "robot": {
                "pose": {"x": 0.0, "y": 0.0},
                "fuel_level": 1.0,
                "storage_fill": 0.0,
            },
            "map": {
                "known_area_ratio": 1.0,
                "cells": [
                    {"x": 1.0, "y": 0.0, "occupancy": "blocked"},
                    {"x": 2.0, "y": 0.0, "occupancy": "unknown"},
                    {"x": 3.0, "y": 0.0, "occupancy": "free"},
                ],
            },
        }))
        node._density_map_cb(self._string_msg({
            "schema": "dtas.debris_density_map.v1",
            "prediction_counts": {"active": 100},
            "density_cells": [
                {"x": 1.0, "y": 0.0, "density": 0.95},
                {"x": 2.0, "y": 0.0, "density": 0.9},
                {"x": 3.0, "y": 0.0, "density": 0.4},
            ],
        }))

        node._plan()

        goal = json.loads(node.goal_pub.messages[-1].data)
        self.assertEqual(goal["mode"], "cleanup")
        self.assertEqual(goal["goal"]["x"], 3.0)
        self.assertEqual(goal["goal"]["y"], 0.0)
        self.assertNotIn("cell_id", goal["goal"])

    def test_field_planner_suppresses_duplicate_goal_publications(self):
        module = importlib.import_module("my_tb3_world.field_planner_node")
        node = module.FieldPlannerNode()
        node._twin_state_cb(self._string_msg({
            "robot": {
                "pose": {"x": 0.0, "y": 0.0},
                "fuel_level": 1.0,
                "storage_fill": 0.0,
            },
        }))
        node._density_map_cb(self._string_msg({
            "schema": "dtas.debris_density_map.v1",
            "prediction_counts": {"active": 100},
            "density_cells": [
                {"x": 1.0, "y": 0.0, "density": 0.8},
            ],
        }))

        node._plan()
        node._plan()

        self.assertEqual(len(node.goal_pub.messages), 1)

    def test_field_planner_idles_from_prediction_count_not_legacy_clusters(self):
        module = importlib.import_module("my_tb3_world.field_planner_node")
        node = module.FieldPlannerNode()
        node._twin_state_cb(self._string_msg({
            "robot": {
                "pose": {"x": 0.0, "y": 0.0},
                "fuel_level": 1.0,
                "storage_fill": 0.0,
            }
        }))
        node._density_map_cb(self._string_msg({
            "schema": "dtas.debris_density_map.v1",
            "prediction_counts": {"active": 0},
            "clusters_remaining": 3,
            "density_cells": [
                {"x": 0.0, "y": 0.0, "density": 0.0},
            ],
        }))

        node._plan()

        goal = json.loads(node.goal_pub.messages[-1].data)
        self.assertEqual(goal["mode"], "idle")
        self.assertEqual(goal["status"], "idle")

    def test_field_planner_return_to_base_uses_twin_base_pose(self):
        module = importlib.import_module("my_tb3_world.field_planner_node")
        node = module.FieldPlannerNode()
        node._twin_state_cb(self._string_msg({
            "base": {"pose": {"x": -1.5, "y": 1.25, "yaw": 0.4}},
            "robot": {
                "pose": {"x": 2.0, "y": 0.0},
                "fuel_level": 0.2,
                "storage_fill": 0.0,
            },
            "map": {"known_area_ratio": 0.75},
        }))
        node._density_map_cb(self._string_msg({
            "schema": "dtas.debris_density_map.v1",
            "prediction_counts": {"active": 100},
            "density_cells": [
                {"x": 2.0, "y": 0.0, "density": 0.8},
            ],
        }))

        node._plan()

        goal = json.loads(node.goal_pub.messages[-1].data)
        self.assertEqual(goal["mode"], "return_to_base")
        self.assertEqual(goal["goal"]["x"], -1.5)
        self.assertEqual(goal["goal"]["y"], 1.25)
        self.assertEqual(goal["goal"]["yaw"], 0.4)
        self.assertEqual(goal["reason"], "fuel low")
        self.assertIn("return_cost", goal["components"])

    def test_dashboard_web_state_merges_planner_intent_debug_only(self):
        module = importlib.import_module("tools.debris_dashboard_web")
        state = module.DashboardState()
        state.set_payload({
            "schema": "dtas.dashboard.v1",
            "status": "ready",
            "physical_particles": [{"id": "physical_particle_001"}],
            "robot": {"pose": {"x": 0.0, "y": 0.0}},
        })
        state.set_prediction({
            "schema": "dtas.debris_density_map.v1",
            "density_cells": [{"x": 1.0, "y": 1.0, "density": 1.0}],
            "prediction_counts": {"active": 100},
        })
        state.set_planner_intent({
            "schema": "dtas.next_cell_goal.v1",
            "stamp": "planner-stamp",
            "source": "field_planner_node",
            "status": "selected",
            "mode": "cleanup",
            "goal": {"frame_id": "map", "x": 1.0, "y": 1.0, "yaw": 0.0},
            "utility": 0.72,
            "return_feasible": True,
            "reason": "highest utility reachable cell with return reserve",
            "components": {"density_reward": 0.9, "travel_cost": 0.18},
        })

        payload = state.get_payload()
        planner = payload["planner_intent"]
        self.assertEqual(payload["physical_particles"][0]["id"], "physical_particle_001")
        self.assertEqual(payload["prediction"]["density_cells"][0]["density"], 1.0)
        self.assertEqual(planner["schema"], "dtas.next_cell_goal.v1")
        self.assertTrue(planner["debug_only"])
        self.assertTrue(planner["forbidden_as_mission_input"])
        self.assertFalse(planner["stale"])
        self.assertEqual(planner["stale_after_sec"], 6.0)
        self.assertEqual(planner["goal"]["frame_id"], "map")
        self.assertEqual(planner["goal"]["x"], 1.0)
        self.assertIn("age_sec", planner)

    def test_dashboard_web_force_toggles_are_independent(self):
        module = importlib.import_module("tools.debris_dashboard_web")

        self.assertIn("layerCurrent", module.HTML)
        self.assertIn("layerWind", module.HTML)
        self.assertIn("layerWave", module.HTML)
        self.assertIn("layerSum", module.HTML)
        self.assertIn("current arrows: base flow, noise curl, eddies, shore redirect", module.HTML)
        self.assertIn("wave arrows: turbulence and tide-height signal", module.HTML)
        self.assertIn("if (layers.current)", module.HTML)
        self.assertIn("if (layers.wind)", module.HTML)
        self.assertIn("if (layers.wave)", module.HTML)
        self.assertIn("if (layers.sum)", module.HTML)

    def test_digital_side_loop_outputs_next_cell_goal_without_debug_inputs(self):
        twin_module = importlib.import_module("my_tb3_world.digital_twin_state_node")
        prediction_module = importlib.import_module("my_tb3_world.debris_prediction_node")
        planner_module = importlib.import_module("my_tb3_world.field_planner_node")

        twin = twin_module.DigitalTwinStateNode()
        twin.cell_size = 1.0
        twin._map_cb(FakeOccupancyGrid(
            width=4,
            height=1,
            resolution=1.0,
            origin_x=0.0,
            origin_y=0.0,
            data=[0, 0, 100, 0],
        ))
        twin._env_obs_cb(self._string_msg({
            "environment_status": "ready",
            "environment_source": "integration_fixture",
            "cells": [
                {
                    "x": 0.5,
                    "y": 0.5,
                    "occupancy": "free",
                    "current_x": 0.0,
                    "current_y": 0.0,
                    "wind_x": 0.0,
                    "wind_y": 0.0,
                    "wave_height": 0.1,
                    "density": 0.99,
                    "physical_particles": [{"id": "hidden_truth_particle"}],
                },
                {
                    "x": 1.5,
                    "y": 0.5,
                    "occupancy": "free",
                    "current_x": 0.0,
                    "current_y": 0.0,
                    "wind_x": 0.0,
                    "wind_y": 0.0,
                    "wave_height": 0.1,
                },
                {
                    "x": 2.5,
                    "y": 0.5,
                    "occupancy": "blocked",
                    "has_environment": False,
                },
                {
                    "x": 3.5,
                    "y": 0.5,
                    "occupancy": "free",
                    "current_x": 0.0,
                    "current_y": 0.0,
                    "wind_x": 0.0,
                    "wind_y": 0.0,
                    "wave_height": 0.1,
                },
            ],
            "density_cells": [{"x": 0.5, "y": 0.5, "density": 1.0}],
            "physical_particles": [{"id": "hidden_truth_particle"}],
        }))
        twin._robot_state_cb(self._string_msg({
            "schema": "dtas.robot_state.v1",
            "pose": {"x": 0.5, "y": 0.5, "yaw": 0.0},
            "fuel_level": 1.0,
            "storage_fill": 0.0,
            "storage_count": 0,
            "storage_capacity": 100,
        }))
        twin._base_pose_cb(self._string_msg({
            "schema": "dtas.base_pose.v1",
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "status": "known",
        }))
        twin._collection_event_cb(self._string_msg({
            "stamp": "integration-event",
            "location": {"x": 0.5, "y": 0.5},
            "count": 1,
            "materials": {"plastic": 1},
        }))
        twin._publish()

        twin_msg = twin.twin_pub.messages[-1]
        twin_state = json.loads(twin_msg.data)
        self.assertEqual(twin_state["sync_status"], "ready")
        self.assertEqual(twin_state["material_evidence"]["plastic"], 1)
        self.assertNotIn("density_cells", twin_state)
        self.assertNotIn("physical_particles", twin_state)
        self.assertEqual(twin_state["map"]["cells"][2]["occupancy"], "blocked")

        prediction = prediction_module.DebrisPredictionNode()
        prediction.prediction_debris_count = 4
        prediction.prediction_drift_enabled = False
        prediction._twin_state_cb(twin_msg)
        prediction.prediction_particles = [
            {
                "id": "prediction_particle_near",
                "x": 0.5,
                "y": 0.5,
                "initial_x": 0.5,
                "initial_y": 0.5,
                "material": "plastic",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            },
        ] + [
            {
                "id": f"prediction_particle_far_{index}",
                "x": 3.5,
                "y": 0.5,
                "initial_x": 3.5,
                "initial_y": 0.5,
                "material": "plastic",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            }
            for index in range(3)
        ]
        prediction._publish()

        density_msg = prediction.density_pub.messages[-1]
        density = json.loads(density_msg.data)
        density_by_x = {cell["x"]: cell["density"] for cell in density["density_cells"]}
        self.assertEqual(density["schema"], "dtas.debris_density_map.v1")
        self.assertEqual(density["status"], "ready")
        self.assertNotIn(2.5, density_by_x)
        self.assertGreater(density_by_x[3.5], density_by_x[0.5])
        self.assertAlmostEqual(sum(density_by_x.values()), 1.0, places=5)
        self.assertNotIn("prediction_particles", density)

        prediction_dashboard = json.loads(
            prediction.prediction_dashboard_pub.messages[-1].data
        )
        self.assertTrue(prediction_dashboard["debug_only"])
        self.assertIn("prediction_particles", prediction_dashboard)

        planner = planner_module.FieldPlannerNode()
        planner_topics = {subscription["topic"] for subscription in planner.subscriptions}
        self.assertEqual(planner_topics, {"/twin_state", "/debris_density_map"})
        planner._twin_state_cb(twin_msg)
        planner._density_map_cb(density_msg)
        planner._plan()

        goal = json.loads(planner.goal_pub.messages[-1].data)
        self.assertEqual(goal["schema"], "dtas.next_cell_goal.v1")
        self.assertEqual(goal["mode"], "cleanup")
        self.assertEqual(goal["goal"]["frame_id"], "map")
        self.assertEqual(goal["goal"]["x"], 3.5)
        self.assertEqual(goal["goal"]["y"], 0.5)
        self.assertNotIn("cell_id", goal["goal"])

    def test_mission_planner_converts_cleanup_goal_to_nav2_pose(self):
        module = importlib.import_module("my_tb3_world.mission_planner_node")
        node = module.MissionPlannerNode()

        node._goal_cb(self._string_msg({
            "schema": "dtas.next_cell_goal.v1",
            "status": "selected",
            "mode": "cleanup",
            "goal": {
                "frame_id": "map",
                "x": 1.25,
                "y": -0.5,
                "yaw": 0.4,
            },
        }))

        self.assertTrue(node.waited_for_nav2)
        self.assertEqual(len(node.nav_to_pose_client.goals), 1)
        goal = node.nav_to_pose_client.goals[-1]
        self.assertEqual(goal.pose.header.frame_id, "map")
        self.assertEqual(goal.pose.pose.position.x, 1.25)
        self.assertEqual(goal.pose.pose.position.y, -0.5)
        self.assertAlmostEqual(goal.pose.pose.orientation.z, math.sin(0.2))
        self.assertAlmostEqual(goal.pose.pose.orientation.w, math.cos(0.2))

    def test_mission_planner_return_to_base_uses_twin_state_base_pose(self):
        module = importlib.import_module("my_tb3_world.mission_planner_node")
        node = module.MissionPlannerNode()
        node._twin_state_cb(self._string_msg({
            "base": {
                "pose": {
                    "x": -1.2,
                    "y": 0.8,
                    "yaw": -0.3,
                }
            }
        }))

        node._goal_cb(self._string_msg({
            "schema": "dtas.next_cell_goal.v1",
            "status": "active",
            "mode": "return_to_base",
            "goal": {
                "frame_id": "map",
                "x": 9.0,
                "y": 9.0,
                "yaw": 9.0,
            },
        }))

        goal = node.nav_to_pose_client.goals[-1]
        self.assertEqual(goal.pose.pose.position.x, -1.2)
        self.assertEqual(goal.pose.pose.position.y, 0.8)
        self.assertAlmostEqual(goal.pose.pose.orientation.z, math.sin(-0.15))
        self.assertAlmostEqual(goal.pose.pose.orientation.w, math.cos(-0.15))

    def test_mission_planner_successful_cleanup_emits_collection_event(self):
        module = importlib.import_module("my_tb3_world.mission_planner_node")
        node = module.MissionPlannerNode()

        node._goal_cb(self._string_msg({
            "schema": "dtas.next_cell_goal.v1",
            "status": "selected",
            "mode": "cleanup",
            "goal": {
                "frame_id": "map",
                "x": 1.25,
                "y": -0.5,
                "yaw": 0.4,
            },
            "waste_type": "plastic",
        }))
        result_future = node.nav_to_pose_client.last_goal_handle.result_future
        for callback in list(result_future.callbacks):
            callback(result_future)

        self.assertEqual(node.collection_pub.topic, "/collection_event")
        event = json.loads(node.collection_pub.messages[-1].data)
        self.assertEqual(event["schema"], "dtas.collection_event.v1")
        self.assertEqual(event["source"], "mission_planner_node")
        self.assertEqual(event["status"], "collected")
        self.assertEqual(event["location"], {"x": 1.25, "y": -0.5})
        self.assertEqual(event["count"], 1)
        self.assertEqual(event["materials"], {"unknown": 1})
        self.assertEqual(event["items"], [{"type": "unknown", "count": 1}])
        self.assertEqual(event["collection_radius_m"], 0.2)

    def test_mission_planner_protects_active_nav2_goal_from_planner_churn(self):
        module = importlib.import_module("my_tb3_world.mission_planner_node")
        node = module.MissionPlannerNode()

        node._goal_cb(self._string_msg({
            "schema": "dtas.next_cell_goal.v1",
            "status": "selected",
            "mode": "cleanup",
            "goal": {
                "frame_id": "map",
                "x": 1.0,
                "y": 0.0,
                "yaw": 0.0,
            },
        }))
        first_goal_handle = node.nav_to_pose_client.last_goal_handle

        node._goal_cb(self._string_msg({
            "schema": "dtas.next_cell_goal.v1",
            "status": "selected",
            "mode": "cleanup",
            "goal": {
                "frame_id": "map",
                "x": 2.0,
                "y": 0.0,
                "yaw": 0.0,
            },
        }))

        self.assertEqual(len(node.nav_to_pose_client.goals), 1)
        self.assertEqual(first_goal_handle.cancel_count, 0)
        self.assertIn("Ignoring cleanup goal while Nav2 goal is active", node.logger.messages)

        result_future = first_goal_handle.result_future
        for callback in list(result_future.callbacks):
            callback(result_future)

        node._goal_cb(self._string_msg({
            "schema": "dtas.next_cell_goal.v1",
            "status": "selected",
            "mode": "cleanup",
            "goal": {
                "frame_id": "map",
                "x": 2.0,
                "y": 0.0,
                "yaw": 0.0,
            },
        }))

        self.assertEqual(len(node.nav_to_pose_client.goals), 2)

    def test_mission_planner_ignores_idle_goal(self):
        module = importlib.import_module("my_tb3_world.mission_planner_node")
        node = module.MissionPlannerNode()

        node._goal_cb(self._string_msg({
            "schema": "dtas.next_cell_goal.v1",
            "status": "idle",
            "mode": "idle",
        }))

        self.assertEqual(node.nav_to_pose_client.goals, [])

    def test_mission_planner_rejects_invalid_cleanup_goal(self):
        module = importlib.import_module("my_tb3_world.mission_planner_node")
        node = module.MissionPlannerNode()

        node._goal_cb(self._string_msg({
            "schema": "dtas.next_cell_goal.v1",
            "status": "selected",
            "mode": "cleanup",
            "goal": {
                "frame_id": "odom",
                "x": 1.0,
                "y": 2.0,
                "yaw": 0.0,
            },
        }))
        node._goal_cb(self._string_msg({
            "schema": "dtas.next_cell_goal.v1",
            "status": "selected",
            "mode": "cleanup",
            "goal": {
                "frame_id": "map",
                "x": "bad",
                "y": 2.0,
                "yaw": 0.0,
            },
        }))

        self.assertEqual(node.nav_to_pose_client.goals, [])
        self.assertEqual(len(node.logger.errors), 2)

    def test_mock_nav_success_flows_to_prediction_and_next_planner_goal(self):
        mission_module = importlib.import_module("my_tb3_world.mission_planner_node")
        prediction_module = importlib.import_module("my_tb3_world.debris_prediction_node")
        planner_module = importlib.import_module("my_tb3_world.field_planner_node")

        mission = mission_module.MissionPlannerNode()
        mission._goal_cb(self._string_msg({
            "schema": "dtas.next_cell_goal.v1",
            "status": "selected",
            "mode": "cleanup",
            "goal": {
                "frame_id": "map",
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
            },
        }))
        result_future = mission.nav_to_pose_client.last_goal_handle.result_future
        for callback in list(result_future.callbacks):
            callback(result_future)
        collection_msg = mission.collection_pub.messages[-1]

        prediction = prediction_module.DebrisPredictionNode()
        prediction.prediction_debris_count = 2
        prediction.prediction_drift_enabled = False
        prediction.twin_map_cells = [
            {"x": 0.0, "y": 0.0, "occupancy": "free"},
            {"x": 1.0, "y": 0.0, "occupancy": "free"},
        ]
        prediction.prediction_particles = [
            {
                "id": "prediction_particle_collected",
                "x": 0.0,
                "y": 0.0,
                "initial_x": 0.0,
                "initial_y": 0.0,
                "material": "plastic",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            },
            {
                "id": "prediction_particle_remaining",
                "x": 1.0,
                "y": 0.0,
                "initial_x": 1.0,
                "initial_y": 0.0,
                "material": "wood",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            },
        ]
        prediction._collection_event_cb(collection_msg)
        prediction._publish()

        density_msg = prediction.density_pub.messages[-1]
        density = json.loads(density_msg.data)
        self.assertEqual(density["prediction_counts"]["observed_removed"], 1)
        self.assertGreater(density["prediction_counts"]["active"], 0)

        planner = planner_module.FieldPlannerNode()
        planner._min_density_reward = 0.001
        planner._twin_state_cb(self._string_msg({
            "robot": {
                "pose": {"x": 0.0, "y": 0.0},
                "fuel_level": 1.0,
                "storage_fill": 0.0,
            },
            "base": {"pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}},
            "map": {
                "known_area_ratio": 1.0,
                "cells": [
                    {"x": 0.0, "y": 0.0, "occupancy": "free"},
                    {"x": 1.0, "y": 0.0, "occupancy": "free"},
                ],
            },
        }))
        planner._density_map_cb(density_msg)
        planner._plan()

        next_goal = json.loads(planner.goal_pub.messages[-1].data)
        self.assertEqual(next_goal["schema"], "dtas.next_cell_goal.v1")
        self.assertIn(next_goal["mode"], {"cleanup", "idle"})

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
        scan = FakeLaserScan(
            [2.0] * 181,
            angle_min=math.radians(-90),
            angle_increment=math.radians(1),
        )

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

    def test_environment_node_publishes_dashboard_truth(self):
        module = importlib.import_module("my_tb3_world.environment_node")
        node = module.EnvironmentNode()
        odom = self._odom_at(9.0, 9.0)

        node._odom_cb(odom)
        node._tick()

        self.assertEqual(node.dashboard_pub.topic, "/dashboard")
        dashboard = self._latest_dashboard(node)
        self.assertEqual(dashboard["schema"], "dtas.dashboard.v1")
        self.assertEqual(dashboard["source"], "environment_node")
        self.assertTrue(dashboard["debug_only"])
        self.assertEqual(dashboard["frame_id"], "map")
        self.assertEqual(dashboard["status"], "ready")
        self.assertEqual(dashboard["physical_debris"]["model"], "particles")
        self.assertEqual(dashboard["collection_radius_m"], module.COLLECTION_RADIUS_M)
        self.assertEqual(dashboard["physical_debris"]["target_active_count"], 100)
        self.assertEqual(dashboard["physical_debris"]["material_response"]["plastic"]["wind"], 0.65)
        self.assertEqual(dashboard["physical_debris"]["boid_rules"]["neighbor_radius_m"], 0.3)
        self.assertEqual(dashboard["physical_debris"]["boid_rules"]["water_damping"], 0.65)
        self.assertEqual(dashboard["collected_count"], 0)
        self.assertEqual(dashboard["remaining_count"], 100)
        self.assertEqual(dashboard["washed_out_count"], 0)
        self.assertEqual(len(dashboard["physical_particles"]), 100)
        self.assertEqual(dashboard["clusters"], [])
        self.assertEqual(dashboard["physical_debris"]["counts"]["collected"], 0)
        self.assertEqual(dashboard["robot"]["pose"], {"x": 9.0, "y": 9.0})
        self.assertEqual(dashboard["environment"]["service"], "generate_environment_field")
        self.assertEqual(dashboard["environment"]["source"], "preset_fallback")
        self.assertEqual(dashboard["environment"]["cell_count"], 400)
        self.assertEqual(dashboard["environment"]["environment_cell_count"], 400)
        observation = json.loads(node.env_pub.messages[-1].data)
        self.assertNotIn("physical_particles", observation)

    def test_environment_node_dashboard_reports_debris_motion_parameters(self):
        module = importlib.import_module("my_tb3_world.environment_node")
        node = module.EnvironmentNode()
        node.parameters["debris_material.plastic.wind"] = 0.9
        node.parameters["debris_boid.neighbor_radius_m"] = 0.7
        node.parameters["debris_boid.water_damping"] = 0.8

        node._odom_cb(self._odom_at(9.0, 9.0))
        node._tick()

        dashboard = self._latest_dashboard(node)
        self.assertEqual(dashboard["physical_debris"]["material_response"]["plastic"]["wind"], 0.9)
        self.assertEqual(dashboard["physical_debris"]["boid_rules"]["neighbor_radius_m"], 0.7)
        self.assertEqual(dashboard["physical_debris"]["boid_rules"]["water_damping"], 0.8)

    def test_environment_node_dashboard_uses_service_field_when_available(self):
        module = importlib.import_module("my_tb3_world.environment_node")
        node = module.EnvironmentNode()
        node.debris_drift_enabled = False
        response = FakeGenerateEnvironmentField.Response()
        response.success = True
        response.environment_status = "ready"
        response.environment_source = "environment_generator_node"
        response.confidence = 0.7
        cell = FakeEnvironmentCell()
        cell.x = -1.5
        cell.y = -1.5
        cell.occupancy = "free"
        cell.has_environment = True
        cell.current_x = 0.4
        cell.current_y = 0.0
        cell.wind_x = 0.2
        cell.wind_y = 0.1
        cell.wave_height = 0.3
        cell.confidence = 0.7
        response.cells = [cell]
        node.environment_client.response = response

        node._tick()
        node._tick()

        dashboard = self._latest_dashboard(node)
        self.assertTrue(dashboard["environment"]["service_available"])
        self.assertEqual(dashboard["environment"]["source"], "environment_generator_node")
        self.assertEqual(dashboard["environment"]["confidence"], 0.7)
        self.assertEqual(dashboard["environment"]["cell_count"], 1)
        self.assertEqual(dashboard["physical_debris"]["counts"]["active"], 100)
        observation = json.loads(node.env_pub.messages[-1].data)
        self.assertEqual(observation["environment_source"], "environment_generator_node")
        self.assertEqual(observation["cells"][0]["current_x"], 0.4)

    def test_environment_node_drifts_hidden_physical_debris_on_free_field(self):
        module = importlib.import_module("my_tb3_world.environment_node")
        node = module.EnvironmentNode()
        node.debris_drift_scale = 1.0
        node._field_cells = [
            {
                "x": 0.88,
                "y": 1.04,
                "occupancy": "free",
                "has_environment": True,
                "current_x": 0.1,
                "current_y": 0.2,
                "wind_x": 0.0,
                "wind_y": 0.0,
                "wave_height": 0.0,
                "confidence": 1.0,
            },
            {
                "x": 0.98,
                "y": 1.24,
                "occupancy": "free",
                "has_environment": True,
                "current_x": 0.1,
                "current_y": 0.2,
                "wind_x": 0.0,
                "wind_y": 0.0,
                "wave_height": 0.0,
                "confidence": 1.0,
            },
        ]
        node.particles = [{
            "id": "physical_particle_test",
            "x": 0.88,
            "y": 1.04,
            "initial_x": 0.88,
            "initial_y": 1.04,
            "material": "plastic",
            "vx": 0.0,
            "vy": 0.0,
            "ax": 0.0,
            "ay": 0.0,
            "status": "active",
        }]

        node._tick()

        particle = node.particles[0]
        dashboard = self._latest_dashboard(node)
        self.assertEqual(particle["x"], 0.98)
        self.assertEqual(particle["y"], 1.24)
        self.assertEqual(particle["vx"], 0.1)
        self.assertEqual(particle["vy"], 0.2)
        self.assertTrue(dashboard["physical_debris"]["hidden_truth_only"])
        self.assertEqual(dashboard["physical_particles"][0]["initial_x"], 0.88)
        self.assertEqual(dashboard["physical_particles"][0]["vy"], 0.2)

    def test_environment_node_washes_out_particle_into_non_free_cell(self):
        module = importlib.import_module("my_tb3_world.environment_node")
        node = module.EnvironmentNode()
        node.debris_drift_scale = 1.0
        node._field_cells = [
            {
                "x": 0.0,
                "y": 0.0,
                "occupancy": "free",
                "has_environment": True,
                "current_x": 0.5,
                "current_y": 0.0,
                "wind_x": 0.0,
                "wind_y": 0.0,
                "wave_height": 0.0,
                "confidence": 1.0,
            },
            {
                "x": 0.5,
                "y": 0.0,
                "occupancy": "blocked",
                "has_environment": False,
                "current_x": 0.0,
                "current_y": 0.0,
                "wind_x": 0.0,
                "wind_y": 0.0,
                "wave_height": 0.0,
                "confidence": 0.0,
            },
        ]
        node.particles = [{
            "id": "physical_particle_blocked_test",
            "x": 0.0,
            "y": 0.0,
            "initial_x": 0.0,
            "initial_y": 0.0,
            "material": "plastic",
            "vx": 0.0,
            "vy": 0.0,
            "ax": 0.0,
            "ay": 0.0,
            "status": "active",
        }]

        node._tick()

        self.assertEqual(len(node.particles), 100)
        self.assertTrue(all(particle["status"] == "active" for particle in node.particles))
        dashboard = self._latest_dashboard(node)
        self.assertEqual(dashboard["washed_out_count"], 1)
        self.assertEqual(dashboard["remaining_count"], 100)
        self.assertEqual(dashboard["physical_debris"]["lifetime_counts"]["respawned"], 100)

    def test_environment_node_washes_out_particle_outside_field_bounds(self):
        module = importlib.import_module("my_tb3_world.environment_node")
        node = module.EnvironmentNode()
        node.debris_drift_scale = 1.0
        node._field_cells = [
            {
                "x": 0.0,
                "y": 0.0,
                "occupancy": "free",
                "has_environment": True,
                "current_x": 1.0,
                "current_y": 0.0,
                "wind_x": 0.0,
                "wind_y": 0.0,
                "wave_height": 0.0,
                "confidence": 1.0,
            },
        ]
        node.particles = [{
            "id": "physical_particle_edge_test",
            "x": 0.0,
            "y": 0.0,
            "initial_x": 0.0,
            "initial_y": 0.0,
            "material": "plastic",
            "vx": 0.0,
            "vy": 0.0,
            "ax": 0.0,
            "ay": 0.0,
            "status": "active",
        }]

        node._tick()

        self.assertEqual(len(node.particles), 100)
        self.assertTrue(all(particle["status"] == "active" for particle in node.particles))
        dashboard = self._latest_dashboard(node)
        self.assertEqual(dashboard["washed_out_count"], 1)

    def test_environment_node_odom_sequence_collects_particles_once(self):
        module = importlib.import_module("my_tb3_world.environment_node")
        node = module.EnvironmentNode()
        node.debris_drift_enabled = False
        node._collection_resume_time = -1.0

        node._tick()
        dashboard = self._latest_dashboard(node)
        self.assertEqual(dashboard["status"], "waiting_for_pose")
        self.assertEqual(dashboard["collected_count"], 0)
        self.assertEqual(dashboard["remaining_count"], 100)

        node._odom_cb(self._odom_at(0.0, 0.0))
        node._tick()
        dashboard = self._latest_dashboard(node)
        self.assertEqual(dashboard["status"], "ready")
        collected_after_center = dashboard["collected_count"]
        self.assertGreaterEqual(collected_after_center, 0)

        node.particles = [
            {
                "id": "physical_particle_collect_a",
                "x": 1.0,
                "y": 1.0,
                "initial_x": 1.0,
                "initial_y": 1.0,
                "material": "plastic",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            },
            {
                "id": "physical_particle_collect_b",
                "x": 1.1,
                "y": 1.0,
                "initial_x": 1.1,
                "initial_y": 1.0,
                "material": "wood",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            },
        ]
        node.collection_pub.messages = []
        node.collection_radius = 0.25
        node._odom_cb(self._odom_at(1.0, 1.0))
        node._tick()

        dashboard = self._latest_dashboard(node)
        event = json.loads(node.collection_pub.messages[-1].data)
        self.assertEqual(event["schema"], "dtas.collection_event.v1")
        self.assertNotIn("cluster_id", event)
        self.assertEqual(event["count"], 2)
        self.assertEqual(event["materials"]["plastic"], 1)
        self.assertEqual(event["materials"]["wood"], 1)
        self.assertEqual(dashboard["collected_count"], 2)
        self.assertEqual(dashboard["remaining_count"], 100)
        self.assertEqual(dashboard["physical_debris"]["lifetime_counts"]["respawned"], 100)

    def test_environment_node_reset_grace_prevents_immediate_collection(self):
        module = importlib.import_module("my_tb3_world.environment_node")
        node = module.EnvironmentNode()
        node.debris_drift_enabled = False
        node.collection_radius = 0.25
        node.particles = [
            {
                "id": "physical_particle_grace",
                "x": 1.0,
                "y": 1.0,
                "initial_x": 1.0,
                "initial_y": 1.0,
                "material": "plastic",
                "vx": 0.0,
                "vy": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "status": "active",
            },
        ]
        node._odom_cb(self._odom_at(1.0, 1.0))

        node._collection_resume_time = 1.0
        node._check_collections()
        self.assertEqual(node.collection_pub.messages, [])
        self.assertEqual(node.particles[0]["status"], "active")

        node._collection_resume_time = -1.0
        node._check_collections()
        event = json.loads(node.collection_pub.messages[-1].data)
        self.assertEqual(event["count"], 1)


if __name__ == "__main__":
    unittest.main()
