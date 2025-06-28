#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from odrive_msgs.srv import SetAxisState
from std_msgs.msg import Float32MultiArray      # example CAN interface msg

ODRIVE_AXES = [("/odrive0/axis0", 0),
               ("/odrive0/axis1", 1)]           # (service_prefix, joint_idx)

CLOSED_LOOP = 8

class TrajExecutor(Node):
    def __init__(self):
        super().__init__('odrive_traj_exec')

        # 1) put every axis in closed-loop
        for srv_prefix,_ in ODRIVE_AXES:
            cli = self.create_client(SetAxisState, f"{srv_prefix}/set_state")
            if not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(f"Service {srv_prefix}/set_state not available")
                rclpy.shutdown(); return
            req = SetAxisState.Request()
            req.requested_state = CLOSED_LOOP
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result() and future.result().success:
                self.get_logger().info(f"{srv_prefix} → CLOSED_LOOP_CONTROL")
            else:
                self.get_logger().error(f"Failed to arm {srv_prefix}")

        # 2) publisher towards odrive_can (example topic)
        self.pub = self.create_publisher(Float32MultiArray,
                                         '/odrive_can/command_joint_positions', 10)

        # 3) trajectory subscriber
        self.create_subscription(JointTrajectory,
                                 '/command_trajectory',
                                 self.traj_cb, 10)

    # –– callback –– ----------------------------------------------------
    def traj_cb(self, msg: JointTrajectory):
        if not msg.points:
            self.get_logger().warn("Received empty trajectory"); return
        self.get_logger().info(f"Executing trajectory with {len(msg.points)} points")

        t0 = self.get_clock().now()
        for pt in msg.points:
            # simple blocking wait – good enough for <50 Hz trajectories
            t_target = (t0 +
                rclpy.duration.Duration(seconds=int(pt.time_from_start.sec),
                                        nanoseconds=int(pt.time_from_start.nanosec)))
            while self.get_clock().now() < t_target:
                rclpy.spin_once(self, timeout_sec=0.001)

            # build CAN command (Float32MultiArray: [pos0,pos1,...])
            cmd = Float32MultiArray()
            cmd.data = list(pt.positions)
            self.pub.publish(cmd)

        self.get_logger().info("Trajectory finished")

def main():
    rclpy.init()
    node = TrajExecutor()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
