import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class GripperSerialNode(Node):
    def __init__(self):
        super().__init__('gripper_serial_node')
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.subscription = self.create_subscription(Float32, '/gripper_command', self.listener_callback, 10)
        self.get_logger().info("Gripper serial node started.")

    def listener_callback(self, msg):
        val = max(0.0, min(1.0, msg.data))  # clamp
        command = f"GRIP {val:.2f}\n"
        self.serial_port.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent to gripper: {command.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = GripperSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
