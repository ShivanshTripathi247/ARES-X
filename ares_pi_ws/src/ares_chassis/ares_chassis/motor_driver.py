#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Robot, OutputDevice

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        
        # 1. Turn ON the Enable Pins (The "Throttle")
        # ENA (Left) -> GPIO 17
        # ENB (Right) -> GPIO 18
        self.ena = OutputDevice(17)
        self.enb = OutputDevice(18)
        self.ena.on()
        self.enb.on()
        
        # 2. Initialize wheels (The "Steering")
        # Left = (IN1, IN2) -> (27, 22)
        # Right = (IN3, IN4) -> (10, 9)
        self.rover = Robot(left=(27, 22), right=(10, 9))
        
        # Subscribe to standard ROS 2 driving commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.get_logger().info("ARES-X Chassis Online: Listening for /cmd_vel")

    def cmd_vel_callback(self, msg):
        # Extract forward/backward (linear.x) and turning (angular.z)
        speed = msg.linear.x
        turn = msg.angular.z

        # Simple differential drive logic
        if speed > 0:
            self.rover.forward()
        elif speed < 0:
            self.rover.backward()
        elif turn > 0:
            self.rover.left()
        elif turn < 0:
            self.rover.right()
        else:
            self.rover.stop()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()