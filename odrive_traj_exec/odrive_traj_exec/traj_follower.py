#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from odrive_can.msg import ControlMessage
from rclpy.duration import Duration
import math

# Optional: define your own enums
POSITION_CONTROL = 3
POSITION_INPUT   = 5

class TrajFollower(Node):
    def __init__(self):
        super().__init__('traj_follower_cm')
        self.axis_map = [
            None,
            ('odrive0', 0, 64.0),
            ('odrive1', 0, 64.0),
            # ('odrive2', 0, 64.0),
            ('odrive2', 0, 10.0),
            ('odrive3', 0, 10.0),
        ]

        self.pubs = {}
        for entry in self.axis_map:
            if not entry:
                continue
            ns, _, _ = entry
            if ns in self.pubs:
                continue
            topic = f'/{ns}/control_message'
            self.pubs[ns] = self.create_publisher(ControlMessage, topic, 10)
            self.get_logger().info(f'Publishing ControlMessage to {topic}')

        self.trajectory = None
        self.start_time = None
        self.point_idx = 0

        self.timer = self.create_timer(0.01, self.timer_cb)

        self.create_subscription(
            JointTrajectory,
            '/command_trajectory',
            self.on_trajectory,
            10
        )

    def on_trajectory(self, traj: JointTrajectory):
        if not traj.points:
            self.get_logger().warn('Empty trajectory')
            return

        self.trajectory = traj
        self.start_time = self.get_clock().now()
        self.point_idx = 0
        self.get_logger().info('New trajectory received')

    def timer_cb(self):
        if self.trajectory is None:
            return

        now = self.get_clock().now()
        while self.point_idx < len(self.trajectory.points):
            pt = self.trajectory.points[self.point_idx]
            secs = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            target = self.start_time + Duration(seconds=secs)

            if now < target:
                # Not time yet
                return

            # Publish commands for this waypoint
            for jidx, rad in enumerate(pt.positions):
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

            self.point_idx += 1

        if self.point_idx >= len(self.trajectory.points):
            self.get_logger().info('All waypoints sent via ControlMessage')
            self.trajectory = None
            self.point_idx = 0
            self.start_time = None

def main(args=None):
    rclpy.init(args=args)
    node = TrajFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
