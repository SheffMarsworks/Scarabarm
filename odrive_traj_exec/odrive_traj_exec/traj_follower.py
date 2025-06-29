#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from odrive_can.msg import ControlMessage
import math

class TrajFollower(Node):
    def __init__(self):
        super().__init__('traj_follower_cm')

        # joint 0 (j1): None, skip
        # joints 1–5 (j2–j6): odrive0…odrive4
        # in your TrajFollower.__init__()
        self.axis_map = [
        # idx 0 is J1: no ODrive → skip it later
        None,
        # idx 1 → J2 lives on odrive0, axis 0
        ('odrive0', 0),
        ('odrive1', 0),
        ('odrive2', 0),
        ('odrive3', 0),
        ('odrive4', 0),
        ]


        # one publisher per ODrive namespace
        self.pubs = {}
        seen = set()
        for entry in self.axis_map:
            if entry is None:
                continue
            ns, _ = entry
            if ns in seen:
                continue
            seen.add(ns)
            topic = f'/{ns}/control_message'
            self.pubs[ns] = self.create_publisher(ControlMessage, topic, 10)
            self.get_logger().info(f'Publishing ControlMessage to {topic}')

        self.create_subscription(
            JointTrajectory,
            '/command_trajectory',
            self.on_trajectory,
            10)

    def on_trajectory(self, traj: JointTrajectory):
        if not traj.points:
            self.get_logger().warn('Empty trajectory')
            return

        t0 = self.get_clock().now()
        for pt in traj.points:
            # wait until waypoint time
            secs = pt.time_from_start.sec + pt.time_from_start.nanosec*1e-9
            target = t0 + rclpy.duration.Duration(seconds=secs)
            while self.get_clock().now() < target:
                rclpy.spin_once(self, timeout_sec=0.001)

            # send one ControlMessage per axis
            for jidx, rad in enumerate(pt.positions):
                entry = self.axis_map[jidx]
                if entry is None:
                    self.get_logger().warn(f"Joint {jidx} has no ODrive → skipping")
                    continue
                ns, arr_idx = entry
                turns = rad / (2*math.pi)
                cm = ControlMessage()
                CONTROL_MODE_POSITION = 3
                INPUT_MODE_POSITION   = 1
                # then in the loop:
                cm.control_mode = CONTROL_MODE_POSITION
                cm.input_mode   = INPUT_MODE_POSITION
                cm.input_pos    = turns
                cm.input_vel    = 0.2   # e.g. 0.2
                # leave input_vel, input_torque at 0
                self.pubs[ns].publish(cm)
                self.get_logger().info(
                    f"→ [{ns}] cm=(mode={cm.control_mode}, in_mode={cm.input_mode}, pos={cm.input_pos})"
                )

        self.get_logger().info('All waypoints sent via ControlMessage')

def main(args=None):
    rclpy.init(args=args)
    node = TrajFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
