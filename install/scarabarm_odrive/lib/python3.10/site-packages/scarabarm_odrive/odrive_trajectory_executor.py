import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from odrive_can.msg import ControlMessage
import asyncio

class ODriveTrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('odrive_trajectory_executor')

        # Joint name to CAN topic map
        self.joint_map = {
            'joint_2': '/odrive_axis2/control_message',
            'joint_3': '/odrive_axis3/control_message',
            'joint_4': '/odrive_axis4/control_message',
            'joint_5': '/odrive_axis5/control_message',
            'joint_6': '/odrive_axis6/control_message',
        }

        # Output-to-motor gear ratios
        self.gear_ratios = {
            'joint_2': 64.0,
            'joint_3': 64.0,
            'joint_4': 64.0,
            'joint_5': 10.0,
            'joint_6': 10.0,
        }

        # Publishers for each joint
        self.joint_publishers = {
            name: self.create_publisher(ControlMessage, topic, 10)
            for name, topic in self.joint_map.items()
        }

        # Subscribe to MoveIt-generated trajectory
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )

        self.get_logger().info('ODrive Trajectory Executor with Interpolation and Gear Ratios Ready.')

    def trajectory_callback(self, msg: JointTrajectory):
        if not msg.points:
            self.get_logger().warn("Received empty trajectory!")
            return

        self.get_logger().info(f"Executing {len(msg.points)} trajectory points...")
        asyncio.create_task(self.execute_trajectory(msg))

    async def execute_trajectory(self, msg: JointTrajectory):
        prev_time = 0.0

        for point in msg.points:
            time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            sleep_time = time_from_start - prev_time
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            prev_time = time_from_start

            for i, joint_name in enumerate(msg.joint_names):
                if joint_name not in self.joint_map:
                    self.get_logger().warn(f"Unknown joint: {joint_name}")
                    continue

                target_output_pos = point.positions[i]
                gear_ratio = self.gear_ratios[joint_name]
                motor_pos = target_output_pos * gear_ratio

                cmd = ControlMessage()
                cmd.control_mode = 3  # POSITION_CONTROL
                cmd.input_mode = 1    # PASSTHROUGH
                cmd.input_pos = motor_pos
                cmd.input_vel = 0.0
                cmd.input_torque = 0.0

                self.joint_publishers[joint_name].publish(cmd)

        self.get_logger().info("Trajectory execution complete.")

def main():
    rclpy.init()
    node = ODriveTrajectoryExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
