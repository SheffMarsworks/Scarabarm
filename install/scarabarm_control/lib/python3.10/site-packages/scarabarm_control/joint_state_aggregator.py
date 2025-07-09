#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from odrive_can.msg import ODriveStatus               # ← 패키지 수정

MAP = {
    'joint_1_to_joint_2': '/odrive_axis0/odrive_status',
    'joint_2_to_link_2':  '/odrive_axis1/odrive_status',
    'link_2_to_joint_3':  '/odrive_axis2/odrive_status',
    'joint_4_to_joint_5': '/odrive_axis3/odrive_status',
    'joint_5_to_joint_6': '/odrive_axis4/odrive_status',
    'joint_6_to_flange':  '/odrive_axis5/odrive_status',
}
GEAR = {
    'joint_1_to_joint_2': 64,
    'joint_2_to_link_2':  64,
    'link_2_to_joint_3':  64,
    'joint_4_to_joint_5': 10,
    'joint_5_to_joint_6': 10,
    'joint_6_to_flange':  10,
}

class JSAggregator(Node):
    def __init__(self):
        super().__init__('scarab_js_agg')
        self.pos = {j: 0.0 for j in MAP}
        for joint, topic in MAP.items():
            self.create_subscription(
                ODriveStatus, topic,
                lambda msg, j=joint: self.cb(msg, j), 10)

        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(0.02, self.timer_cb)       # 50 Hz

    def cb(self, msg, joint):
        self.pos[joint] = msg.pos_turns * 2 * math.pi / GEAR[joint]

    def timer_cb(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.pos.keys())
        js.position = [self.pos[j] for j in js.name]
        self.pub.publish(js)

def main():
    rclpy.init()
    node = JSAggregator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
