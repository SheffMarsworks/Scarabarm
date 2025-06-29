#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from odrive_msgs.srv import SetAxisState           # provided by ros_odrive
CLOSED_LOOP = 8                                    # AxisState enum

# list the axis namespaces you actually run
AXES = ["/odrive0/axis0", "/odrive0/axis1"]

class AxisArmer(Node):
    def __init__(self):
        super().__init__("axis_armer")
        self.pending = len(AXES)
        for ns in AXES:
            cli = self.create_client(SetAxisState, f"{ns}/set_state")
            self.get_logger().info(f"Waiting for {ns}/set_state …")
            cli.wait_for_service()
            req = SetAxisState.Request()
            req.requested_state = CLOSED_LOOP
            self.get_logger().info(f"Requesting CLOSED_LOOP on {ns}")
            cli.call_async(req).add_done_callback(
                lambda fut, ns=ns: self._cb(fut, ns))

    def _cb(self, future, ns):
        ok = future.result() and future.result().success
        msg = "✓" if ok else "✗"
        self.get_logger().info(f"{ns} {msg}")
        self.pending -= 1
        if self.pending == 0:
            self.get_logger().info("All axes processed — shutting down.")
            rclpy.shutdown()

def main():
    rclpy.init()
    AxisArmer()
    rclpy.spin_until_future_complete(rclpy.get_default_context())  # keep alive
if __name__ == "__main__":
    main()
