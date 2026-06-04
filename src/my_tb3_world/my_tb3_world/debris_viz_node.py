#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray

MATERIAL_COLORS = {
    'plastic': (0.18, 0.56, 0.44),
    'wood':    (0.55, 0.44, 0.24),
    'metal':   (0.36, 0.40, 0.44),
    'unknown': (0.60, 0.65, 0.68),
}
PINK = (1.0, 0.41, 0.71)
WHITE = (1.0, 1.0, 1.0)


def _color(r, g, b, a=1.0):
    c = ColorRGBA()
    c.r = float(r)
    c.g = float(g)
    c.b = float(b)
    c.a = float(a)
    return c


def _vec3(x, y, z):
    v = Vector3()
    v.x = float(x)
    v.y = float(y)
    v.z = float(z)
    return v


def _point(x, y, z):
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


def _deleteall_marker(ns, frame_id='map'):
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = ns
    m.id = 0
    m.action = Marker.DELETEALL
    return m


class DebrisVizNode(Node):
    def __init__(self):
        super().__init__('debris_viz_node')

        self.declare_parameter('publish_force_vectors', True)
        self.declare_parameter('publish_density', True)

        self._force_vectors_enabled = self.get_parameter('publish_force_vectors').value
        self._density_enabled = self.get_parameter('publish_density').value

        self._dashboard = None
        self._density_map = None
        self._twin_state = None
        self._planner_goal = None

        self.create_subscription(String, '/dashboard', self._dashboard_cb, 10)
        self.create_subscription(String, '/debris_density_map', self._density_map_cb, 10)
        self.create_subscription(String, '/twin_state', self._twin_state_cb, 10)
        self.create_subscription(String, '/next_cell_goal', self._goal_cb, 10)

        self._particles_pub = self.create_publisher(
            MarkerArray, '/debris_viz/physical_particles', 10
        )
        self._density_pub = self.create_publisher(
            MarkerArray, '/debris_viz/density_cells', 10
        )
        self._force_pub = self.create_publisher(
            MarkerArray, '/debris_viz/force_vectors', 10
        )
        self._goal_pub = self.create_publisher(
            MarkerArray, '/debris_viz/planner_goal', 10
        )
        self._robot_pub = self.create_publisher(
            MarkerArray, '/debris_viz/robot_state', 10
        )

        self.create_timer(0.5, self._publish)
        self.get_logger().info('DebrisVizNode started')

    def _dashboard_cb(self, msg: String):
        try:
            self._dashboard = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _density_map_cb(self, msg: String):
        try:
            self._density_map = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _twin_state_cb(self, msg: String):
        try:
            self._twin_state = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _goal_cb(self, msg: String):
        try:
            self._planner_goal = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _publish(self):
        now = self._stamp()
        self._publish_physical_particles(now)
        self._publish_density_cells(now)
        self._publish_force_vectors(now)
        self._publish_planner_goal(now)
        self._publish_robot_state(now)

    def _publish_physical_particles(self, now):
        markers = [_deleteall_marker('physical_particles')]
        if self._dashboard:
            for idx, p in enumerate(self._dashboard.get('physical_particles', [])):
                if p.get('status') != 'active':
                    continue
                material = p.get('material', 'unknown')
                rgb = MATERIAL_COLORS.get(material, MATERIAL_COLORS['unknown'])
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = now
                m.ns = 'physical_particles'
                m.id = idx + 1
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position = _point(p['x'], p['y'], 0.05)
                m.pose.orientation.w = 1.0
                m.scale = _vec3(0.08, 0.08, 0.08)
                m.color = _color(*rgb)
                markers.append(m)
        ma = MarkerArray()
        ma.markers = markers
        self._particles_pub.publish(ma)

    def _publish_density_cells(self, now):
        markers = [_deleteall_marker('density_cells')]
        if self._density_enabled and self._density_map:
            cells = (
                self._density_map.get('density_cells')
                or self._density_map.get('cells', [])
            )
            for idx, cell in enumerate(cells):
                density = float(cell.get('density', 0.0))
                if density < 0.005:
                    continue
                alpha = min(0.8, density * 8.0)
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = now
                m.ns = 'density_cells'
                m.id = idx + 1
                m.type = Marker.CUBE
                m.action = Marker.ADD
                m.pose.position = _point(cell['x'], cell['y'], 0.01)
                m.pose.orientation.w = 1.0
                m.scale = _vec3(0.45, 0.45, 0.01)
                m.color = _color(0.18, 0.56, 0.44, alpha)
                markers.append(m)
        ma = MarkerArray()
        ma.markers = markers
        self._density_pub.publish(ma)

    def _publish_force_vectors(self, now):
        markers = [_deleteall_marker('force_vectors')]
        if self._force_vectors_enabled and self._twin_state:
            cells = self._twin_state.get('map', {}).get('cells', [])
            for idx, cell in enumerate(cells):
                cx = float(cell.get('current_x', 0.0))
                cy = float(cell.get('current_y', 0.0))
                mag = math.sqrt(cx * cx + cy * cy)
                if mag < 1e-6:
                    continue
                scale = min(mag, 1.0) * 0.3
                ex = cell['x'] + cx / mag * scale
                ey = cell['y'] + cy / mag * scale
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = now
                m.ns = 'force_vectors'
                m.id = idx + 1
                m.type = Marker.ARROW
                m.action = Marker.ADD
                m.points = [_point(cell['x'], cell['y'], 0.02), _point(ex, ey, 0.02)]
                m.scale = _vec3(0.02, 0.04, 0.05)
                m.color = _color(0.30, 0.70, 1.0, 0.7)
                markers.append(m)
        ma = MarkerArray()
        ma.markers = markers
        self._force_pub.publish(ma)

    def _publish_planner_goal(self, now):
        markers = [_deleteall_marker('planner_goal')]
        if self._planner_goal:
            mode = self._planner_goal.get('mode', '')
            goal = self._planner_goal.get('goal', {})
            if mode in ('cleanup', 'return_to_base') and goal:
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = now
                m.ns = 'planner_goal'
                m.id = 1
                m.type = Marker.CYLINDER
                m.action = Marker.ADD
                m.pose.position = _point(goal['x'], goal['y'], 0.15)
                m.pose.orientation.w = 1.0
                m.scale = _vec3(0.15, 0.15, 0.30)
                m.color = _color(*PINK)
                markers.append(m)
        ma = MarkerArray()
        ma.markers = markers
        self._goal_pub.publish(ma)

    def _publish_robot_state(self, now):
        markers = [_deleteall_marker('robot_state')]
        if self._twin_state:
            robot = self._twin_state.get('robot', {})
            pose = robot.get('pose', {})
            rx = float(pose.get('x', 0.0))
            ry = float(pose.get('y', 0.0))
            fuel = robot.get('fuel_level', 1.0)
            storage = robot.get('storage_fill', 0.0)
            label = f'fuel={fuel:.0%} stor={storage:.0%}'
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'robot_state'
            m.id = 1
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position = _point(rx, ry, 0.4)
            m.pose.orientation.w = 1.0
            m.scale = _vec3(0.0, 0.0, 0.15)
            m.color = _color(*WHITE)
            m.text = label
            markers.append(m)
        ma = MarkerArray()
        ma.markers = markers
        self._robot_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = DebrisVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
