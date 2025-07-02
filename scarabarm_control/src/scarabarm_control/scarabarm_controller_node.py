#!/usr/bin/env python3
import math
import asyncio
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from control_msgs.action import FollowJointTrajectory
from odrive_can.msg import ControlMessage

GEAR = {
    'joint_1_to_joint_2': 64,
    'joint_2_to_link_2':  64,
    'link_2_to_joint_3':  64,
    'joint_4_to_joint_5': 10,
    'joint_5_to_joint_6': 10,
    'joint_6_to_flange':  10,
}

TOPIC = {
    'joint_1_to_joint_2': '/odrive_axis0/control_message',
    'joint_2_to_link_2':  '/odrive_axis1/control_message',
    'link_2_to_joint_3':  '/odrive_axis2/control_message',
    'joint_4_to_joint_5': '/odrive_axis3/control_message',
    'joint_5_to_joint_6': '/odrive_axis4/control_message',
    'joint_6_to_flange':  '/odrive_axis5/control_message',
}

class ScarabController(Node):
    def __init__(self):
        super().__init__('scarab_controller')

        # ★ 변수명 변경: self.pub
        self.pub = {j: self.create_publisher(ControlMessage, t, 10)
                    for j, t in TOPIC.items()}

        self.server = ActionServer(
            self, FollowJointTrajectory, 'follow_joint_trajectory',
            execute_callback=self.execute_cb,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT)

    def rad2turns(self, joint, rad):
        return rad * GEAR[joint] / (2.0 * math.pi)

    async def execute_cb(self, goal_handle):
        traj = goal_handle.request.trajectory
        start = self.get_clock().now()

        for pt in traj.points:
            # 목표 시각까지 대기
            target = start + rclpy.time.Duration(
                seconds=pt.time_from_start.sec +
                        pt.time_from_start.nanosec * 1e-9)
            while self.get_clock().now() < target:
                # Use asyncio.sleep for non-blocking delay
                await asyncio.sleep(0.001)

            # 각 관절에 명령 전송
            for idx, joint in enumerate(traj.joint_names):
                msg = ControlMessage()
                msg.control_mode = 3      # POSITION_CONTROL
                msg.input_mode   = 6      # TRAP_TRAJ
                msg.input_pos    = float(self.rad2turns(joint, pt.positions[idx]))
                self.pub[joint].publish(msg)

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

def main():
    rclpy.init()
    node = ScarabController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
