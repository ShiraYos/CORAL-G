#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from my_tb3_world.environment_field import (
    DEFAULT_CELL_SIZE,
    DEFAULT_CONTROLS,
    build_environment_cells,
    parse_generation_controls,
)
from my_tb3_world_interfaces.msg import EnvironmentCell
from my_tb3_world_interfaces.srv import GenerateEnvironmentField


class EnvironmentGeneratorNode(Node):
    def __init__(self):
        super().__init__('environment_generator_node')

        self.declare_parameter('cell_size_m', DEFAULT_CELL_SIZE)
        for key, value in DEFAULT_CONTROLS.items():
            self.declare_parameter(key, value)

        self.map_msg = None
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)
        self.create_service(
            GenerateEnvironmentField,
            'generate_environment_field',
            self._generate_environment_field,
        )

        self.get_logger().info('EnvironmentGeneratorNode started')

    def _map_cb(self, msg):
        self.map_msg = msg
        self.get_logger().info(
            f'Map received for environment generation: '
            f'{msg.info.width}x{msg.info.height}'
        )

    def _parameter_controls(self):
        return {
            key: self.get_parameter(key).value
            for key in DEFAULT_CONTROLS
        }

    def _generate_environment_field(self, request, response):
        controls = self._parameter_controls()
        requested_controls, warnings = parse_generation_controls(
            request.generation_controls_json
        )
        controls.update(requested_controls)

        if request.frame_id and request.frame_id != 'map':
            response.success = False
            response.environment_status = 'invalid_frame'
            response.environment_source = 'environment_generator_node'
            response.confidence = 0.0
            response.cells = []
            response.warnings = warnings + ['frame_id must be map']
            return response

        if self.map_msg is None:
            response.success = False
            response.environment_status = 'waiting_for_map'
            response.environment_source = 'environment_generator_node'
            response.confidence = 0.0
            response.cells = []
            response.warnings = warnings + ['no /map received']
            return response

        cell_size = request.cell_size_m or self.get_parameter('cell_size_m').value
        field_cells = build_environment_cells(self.map_msg, cell_size, controls)
        response.success = True
        response.environment_status = 'ready'
        response.environment_source = 'environment_generator_node'
        response.confidence = round(controls['confidence'], 3)
        response.cells = [self._to_msg(cell) for cell in field_cells]
        response.warnings = warnings
        return response

    def _to_msg(self, cell):
        msg = EnvironmentCell()
        msg.x = cell['x']
        msg.y = cell['y']
        msg.occupancy = cell['occupancy']
        msg.has_environment = cell['has_environment']
        msg.current_x = cell['current_x']
        msg.current_y = cell['current_y']
        msg.wind_x = cell['wind_x']
        msg.wind_y = cell['wind_y']
        msg.wave_height = cell['wave_height']
        msg.confidence = cell['confidence']
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
