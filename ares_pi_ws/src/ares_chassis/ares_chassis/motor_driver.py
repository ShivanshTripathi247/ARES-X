#!/usr/.bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Robot

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        
        # Initialize L298N pins (using the ones you proved working!)
        self.rover = Robot(left=(17, 27), right=(22, 23))
        
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