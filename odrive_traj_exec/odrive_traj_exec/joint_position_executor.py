#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from odrive_can.msg import ControlMessage
import math

# Control modes
POSITION_CONTROL = 3
POSITION_INPUT = 5

class JointPositionExecutor(Node):
    def __init__(self):
        super().__init__('joint_position_listener')

        # Same axis mapping
        self.axis_map = [
            None,
            ('odrive0', 0, 64.0),   # Joint 2
            ('odrive1', 0, 64.0),   # Joint 3
            # ('odrive2', 0, 64.0),   # Joint 4
            ('odrive2', 0, 10.0),   # Joint 5
            ('odrive3', 0, 10.0),   # Joint 6
        ]

        # Create publishers
        self.pubs = {}
        for entry in self.axis_map:
            if not entry:
                continue
            ns, _, _ = entry
            topic = f'/{ns}/control_message'
            self.pubs[ns] = self.create_publisher(ControlMessage, topic, 10)
            self.get_logger().info(f'Publishing ControlMessage to {topic}')

        # Subscribe to the Float64MultiArray with joint positions
        self.create_subscription(
            Float64MultiArray,
            '/odrive_can/command_joint_positions',
            self.on_command,
            10
        )

    def on_command(self, msg: Float64MultiArray):
        positions = msg.data
        self.get_logger().info(f'Received joint positions: {positions}')

        # For each joint, publish the ODrive command
        for jidx, rad in enumerate(positions):
            entry = self.axis_map[jidx]
            if entry is None:
                continue
            ns, _, ratio = entry
            turns = (rad * ratio) / (2 * math.pi)
            cm = ControlMessage()
            cm.control_mode = POSITION_CONTROL
            cm.input_mode = POSITION_INPUT
            cm.input_pos = turns
            self.pubs[ns].publish(cm)

def main(args=None):
    rclpy.init(args=args)
    node = JointPositionExecutor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
