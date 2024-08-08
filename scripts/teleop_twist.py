#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios

class TeleopTwistNode(Node):
    def __init__(self):
        super().__init__('teleop_twist_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Use arrow keys to control the robot. Press 'q' to quit.")
        self.run()

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        twist = Twist()
        try:
            while True:
                key = self.get_key()
                if key == 'w':
                    twist.linear.x = 1.0
                    twist.angular.z = 0.0
                elif key == 's':
                    twist.linear.x = -1.0
                    twist.angular.z = 0.0
                elif key == 'a':
                    twist.linear.x = 0.0
                    twist.angular.z = 1.0
                elif key == 'd':
                    twist.linear.x = 0.0
                    twist.angular.z = -1.0
                elif key == 'q':
                    break
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                self.publisher_.publish(twist)
                self.get_logger().info(f"Publishing: {twist}")
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")
        finally:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopTwistNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
