#!/usr/bin/env python3

import argparse
import json
import time


DEMO_ODOM_POINTS = [
    ('water_test_position', 0.3, 0.9),
]


class PreviewComplete(Exception):
    pass


def format_dashboard(payload):
    robot = payload.get('robot', {})
    pose = robot.get('pose', {})
    clusters = payload.get('clusters', [])
    environment = payload.get('environment', {})
    lines = [
        'CORAL-G debris dashboard preview',
        f'schema: {payload.get("schema", "?")} | status: {payload.get("status", "?")}',
        f'robot: x={pose.get("x", "?")} y={pose.get("y", "?")} '
        f'pose_received={robot.get("pose_received", False)}',
        f'collected: {payload.get("collected_count", 0)} | '
        f'remaining: {payload.get("remaining_count", 0)} | '
        f'radius: {payload.get("collection_radius_m", "?")}m',
        f'field: source={environment.get("source", "?")} '
        f'available={environment.get("service_available", False)} '
        f'cells={environment.get("environment_cell_count", 0)}/'
        f'{environment.get("cell_count", 0)} '
        f'blocked={environment.get("blocked_cell_count", 0)} '
        f'unknown={environment.get("unknown_cell_count", 0)}',
        '',
        'cluster      type  count  state      position',
        '-----------  ----  -----  ---------  ----------------',
    ]

    for cluster in clusters:
        state = 'collected' if cluster.get('collected') else 'open'
        lines.append(
            f'{cluster.get("id", "?"):<11}  '
            f'{cluster.get("type", "?"):<4}  '
            f'{cluster.get("count", "?"):<5}  '
            f'{state:<9}  '
            f'({cluster.get("x", "?")}, {cluster.get("y", "?")})'
        )

    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(
        description='Preview physical debris truth from /dashboard.'
    )
    parser.add_argument(
        '--once',
        action='store_true',
        help='Print the first received dashboard payload and exit.',
    )
    parser.add_argument(
        '--demo-odom',
        action='store_true',
        help='Publish a small best-effort /odom sequence through all debris clusters.',
    )
    parser.add_argument(
        '--interval-sec',
        type=float,
        default=1.5,
        help='Seconds between demo odom points.',
    )
    args = parser.parse_args()

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('debris_dashboard_preview')
    current_demo_started = 0.0
    last_demo_publish = 0.0
    next_demo_index = 0

    odom_pub = None
    if args.demo_odom:
        odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        odom_pub = node.create_publisher(Odometry, '/odom', odom_qos)

    def dashboard_cb(msg):
        payload = json.loads(msg.data)
        print('\n' + format_dashboard(payload), flush=True)
        remaining_count = int(payload.get('remaining_count', 0))
        if args.once:
            raise PreviewComplete
        if args.demo_odom and remaining_count == 0:
            raise PreviewComplete

    node.create_subscription(String, '/dashboard', dashboard_cb, 10)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if not args.demo_odom or odom_pub is None:
                continue

            now = time.monotonic()
            hold_last = next_demo_index >= len(DEMO_ODOM_POINTS)
            label, x, y = DEMO_ODOM_POINTS[-1 if hold_last else next_demo_index]
            if current_demo_started == 0.0 and not hold_last:
                current_demo_started = now
                print(f'\n[demo_odom] {label}: x={x} y={y}', flush=True)

            if now - last_demo_publish >= 0.2:
                odom = Odometry()
                odom.pose.pose.position.x = x
                odom.pose.pose.position.y = y
                odom.pose.pose.orientation.w = 1.0
                odom_pub.publish(odom)
                last_demo_publish = now

            if hold_last or now - current_demo_started < args.interval_sec:
                continue

            next_demo_index += 1
            current_demo_started = 0.0
    except KeyboardInterrupt:
        pass
    except PreviewComplete:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
