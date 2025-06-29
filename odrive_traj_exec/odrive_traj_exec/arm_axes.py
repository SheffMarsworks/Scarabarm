import rclpy
from rclpy.node import Node
from odrive_can.srv import AxisState
# Optionally: from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL

class ArmAxes(Node):
    def __init__(self):
        super().__init__('arm_axes')
        self.cli = self.create_client(AxisState, '/request_axis_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /request_axis_state…')
        self.get_logger().info('/request_axis_state is available!')

        req = AxisState.Request()
        req.axis_requested_state = 8  # closed-loop control
        self.future = self.cli.call_async(req)
        self.future.add_done_callback(self.on_response)

    def on_response(self, future):
        res = future.result()
        if res.success:
            self.get_logger().info('Closed-loop control enabled!')
        else:
            self.get_logger().error('Failed to enable closed-loop')

def main(args=None):
    rclpy.init(args=args)
    node = ArmAxes()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
