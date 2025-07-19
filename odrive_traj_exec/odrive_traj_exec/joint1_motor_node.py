#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pigpio
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class Joint1Controller(Node):
    def __init__(self):
        super().__init__('joint1_controller')

        # === CONFIGURATION ===
        self.PWM_PIN = 12   # GPIO12 (Pin 32)
        self.DIR_PIN = 23   # GPIO23 (Pin 16)
        self.A_PIN = 5      # Encoder A - GPIO5 (Pin 29)
        self.B_PIN = 6      # Encoder B - GPIO6 (Pin 31)

        self.GEAR_RATIO = 99.5
        self.COUNTS_PER_REV = 12 * 4  # 12 CPR × 4x = 48 counts/rev
        self.TOTAL_COUNTS_PER_JOINT_REV = self.COUNTS_PER_REV * self.GEAR_RATIO

        # PID parameters
        self.kp = 0.5
        self.ki = 0.0
        self.kd = 0.01
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

        # === SETUP pigpio ===
        self.pi = pigpio.pi()
        self.pi.set_mode(self.DIR_PIN, pigpio.OUTPUT)
        self.pi.set_mode(self.PWM_PIN, pigpio.OUTPUT)
        self.pi.set_mode(self.A_PIN, pigpio.INPUT)
        self.pi.set_mode(self.B_PIN, pigpio.INPUT)

        self.position_counts = 0
        self.target_counts = 0

        # Encoder callbacks
        self.cb_a = self.pi.callback(self.A_PIN, pigpio.EITHER_EDGE, self.encoder_callback)
        self.cb_b = self.pi.callback(self.B_PIN, pigpio.EITHER_EDGE, self.encoder_callback)

        # ROS 2 subs/pubs
        self.sub = self.create_subscription(Float64, '/joint1/command', self.target_callback, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Feedback publisher timer (20 Hz)
        self.create_timer(0.05, self.publish_joint_state)

        # PID control timer (100 Hz)
        self.create_timer(0.01, self.update_motor)

        self.get_logger().info('Joint1 controller running with PID and feedback')

    def encoder_callback(self, gpio, level, tick):
        a = self.pi.read(self.A_PIN)
        b = self.pi.read(self.B_PIN)
        direction = 1 if a == b else -1
        self.position_counts += direction

    def target_callback(self, msg):
        target_radians = msg.data
        self.target_counts = int((target_radians / (2 * 3.14159265359)) * self.TOTAL_COUNTS_PER_JOINT_REV)

    def update_motor(self):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt == 0:
            return

        error = self.target_counts - self.position_counts
        self.error_sum += error * dt
        d_error = (error - self.last_error) / dt

        output = self.kp * error + self.ki * self.error_sum + self.kd * d_error
        output = max(min(output, 1.0), -1.0)  # Clamp output to [-1, 1]

        if abs(error) < 5:
            self.pi.hardware_PWM(self.PWM_PIN, 0, 0)
        else:
            direction = 1 if output > 0 else 0
            duty = int(min(abs(output), 1.0) * 1_000_000)  # duty cycle in [0–1M]
            self.pi.write(self.DIR_PIN, direction)
            self.pi.hardware_PWM(self.PWM_PIN, 20000, duty)

        self.last_error = error
        self.last_time = current_time

    def publish_joint_state(self):
        js = JointState()
        js.name = ['joint_1_to_joint_2']

        # Convert encoder counts to radians
        radians = (self.position_counts / self.TOTAL_COUNTS_PER_JOINT_REV) * 2 * 3.14159265359
        js.position = [radians]

        now = self.get_clock().now().to_msg()
        js.header.stamp = now

        self.joint_pub.publish(js)

    def destroy_node(self):
        self.cb_a.cancel()
        self.cb_b.cancel()
        self.pi.hardware_PWM(self.PWM_PIN, 0, 0)
        self.pi.write(self.DIR_PIN, 0)
        self.pi.stop()
        super().destroy_node()

    def main(args=None):
        rclpy.init(args=args)
        node = Joint1Controller()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


