#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BaseReferenceNode(Node):
    def __init__(self):
        super().__init__('base_reference_node')

        self.declare_parameter('base_x', 0.0)
        self.declare_parameter('base_y', 0.0)
        self.declare_parameter('base_yaw', 0.0)

        self.base_x = float(self.get_parameter('base_x').value)
        self.base_y = float(self.get_parameter('base_y').value)
        self.base_yaw = float(self.get_parameter('base_yaw').value)

        self.pub = self.create_publisher(String, '/base_pose', 10)
        self.create_timer(1.0, self._publish)

        self.get_logger().info(
            f'BaseReferenceNode: base at ({self.base_x}, {self.base_y}, yaw={self.base_yaw})'
        )

    def _publish(self):
        msg = String()
        msg.data = json.dumps({
            'schema': 'dtas.base_pose.v1',
            'pose': {
                'x': self.base_x,
                'y': self.base_y,
                'yaw': self.base_yaw,
            },
            'status': 'known',
        })
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BaseReferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
