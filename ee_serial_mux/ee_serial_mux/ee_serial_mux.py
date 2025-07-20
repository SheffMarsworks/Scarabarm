# ee_serial_mux.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32
import serial

class EESerialMux(Node):
    def __init__(self):
        super().__init__('ee_serial_mux')

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.current_type = 'dynamixel'  # default

        self.sub_type  = self.create_subscription(String,  '/ee_command/type',  self.cb_type,  10)
        self.sub_grip  = self.create_subscription(Float32, '/ee_command/grip',  self.cb_grip,  10)
        self.sub_motor = self.create_subscription(Int32,   '/ee_command/motor', self.cb_motor, 10)

        self.get_logger().info("EE Serial MUX ready.")

    def cb_type(self, msg):
        self.current_type = msg.data.strip().lower()
        self.get_logger().info(f"End-effector type set to: {self.current_type}")

    def cb_grip(self, msg):
        val = max(0.0, min(1.0, msg.data))
        if self.current_type == "drill":
            cmd = "GRIP_OPEN\n" if val < 0.5 else "GRIP_CLOSE\n"
        elif self.current_type == "dynamixel":
            cmd = f"GRIP {val:.2f}\n"
        else:
            self.get_logger().warn(f"Unknown EE type for grip: {self.current_type}")
            return
        self.send(cmd)

    def cb_motor(self, msg):
        val = max(-255, min(255, msg.data))
        if self.current_type == "drill":
            cmd = f"MOTOR {val}\n"
        else:
            self.get_logger().warn("Motor command only supported for drill-based EE")
            return
        self.send(cmd)

    def send(self, cmd):
        try:
            self.ser.write(cmd.encode('utf-8'))
            self.get_logger().info(f"Sent: {cmd.strip()}")
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EESerialMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
