#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


ARENA_MIN = -2.0
ARENA_MAX = 2.0
CELL_SIZE = 1.0

# Must match environment_node.py and new_world.world
CLUSTERS = [
    {'id': 'cluster_1', 'x':  0.88, 'y':  1.04, 'type': 'A', 'count': 4},
    {'id': 'cluster_2', 'x': -1.2,  'y': -1.2,  'type': 'B', 'count': 4},
    {'id': 'cluster_3', 'x': -1.2,  'y':  1.2,  'type': 'C', 'count': 3},
]


def _cell_center(px, py):
    """Snap a world position to the center of its 1m grid cell."""
    cx = math.floor((px - ARENA_MIN) / CELL_SIZE) * CELL_SIZE + ARENA_MIN + CELL_SIZE / 2.0
    cy = math.floor((py - ARENA_MIN) / CELL_SIZE) * CELL_SIZE + ARENA_MIN + CELL_SIZE / 2.0
    return round(cx, 3), round(cy, 3)


class DebrisPredictionNode(Node):
    def __init__(self):
        super().__init__('debris_prediction_node')

        self.declare_parameter('publish_rate_hz', 1.0)

        self.collected_ids: set = set()

        self.create_subscription(String, '/twin_state', self._twin_state_cb, 10)
        self.create_subscription(String, '/collection_event', self._collection_event_cb, 10)

        self.density_pub = self.create_publisher(String, '/debris_density_map', 10)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(f'DebrisPredictionNode: tracking {len(CLUSTERS)} clusters')

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _twin_state_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Bad JSON on /twin_state')
            return

        # Bootstrap from twin_state's collection history so late-starts catch up
        for event in data.get('collection_events', []):
            cid = event.get('cluster_id')
            if cid:
                self.collected_ids.add(cid)

    def _collection_event_cb(self, msg: String):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        cid = event.get('cluster_id')
        if cid:
            self.collected_ids.add(cid)
            self.get_logger().info(f'Cluster {cid} marked collected')

    # ── Publish ────────────────────────────────────────────────────────────────

    def _stamp(self) -> str:
        t = self.get_clock().now().to_msg()
        return f'{t.sec}.{t.nanosec:09d}'

    def _publish(self):
        # Build a zero-density grid over the arena
        cells: dict = {}
        x = ARENA_MIN + CELL_SIZE / 2.0
        while x < ARENA_MAX:
            y = ARENA_MIN + CELL_SIZE / 2.0
            while y < ARENA_MAX:
                key = (round(x, 3), round(y, 3))
                cells[key] = {
                    'x': key[0],
                    'y': key[1],
                    'density': 0.0,
                    'waste_type': None,
                    'cluster_id': None,
                }
                y += CELL_SIZE
            x += CELL_SIZE

        # Paint uncollected clusters onto the grid
        remaining = 0
        for cluster in CLUSTERS:
            if cluster['id'] in self.collected_ids:
                continue
            remaining += 1
            cx, cy = _cell_center(cluster['x'], cluster['y'])
            key = (cx, cy)
            if key in cells:
                cells[key]['density'] = 1.0
                cells[key]['waste_type'] = cluster['type']
                cells[key]['cluster_id'] = cluster['id']
                # Store actual cluster position — planner uses this, not the cell center,
                # so the nav goal avoids wall inflation zones around cell boundaries
                cells[key]['cluster_x'] = cluster['x']
                cells[key]['cluster_y'] = cluster['y']

        cell_list = sorted(cells.values(), key=lambda c: (c['x'], c['y']))

        out = String()
        out.data = json.dumps({
            'schema': 'dtas.debris_density_map.v1',
            'stamp': self._stamp(),
            'source': 'debris_prediction_node',
            'cell_size_m': CELL_SIZE,
            'cells': cell_list,
            'clusters_remaining': remaining,
        })
        self.density_pub.publish(out)

        self.get_logger().info(
            f'Debris map: {remaining}/{len(CLUSTERS)} clusters remaining'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DebrisPredictionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
