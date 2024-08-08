#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class AutonomousDriveNode(Node):
    def __init__(self):
        super().__init__('autonomous_drive_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub_ = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # PID constants
        self.kp = 0.5  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.5  # Derivative gain
        
        self.track_width_ = 2.2  # Assumed track width, adjust based on your environment
        self.max_speed_ = 5.0  # Max linear speed
        self.max_turn_ = 1.0  # Max angular speed
        self.last_error_ = 0.0
        self.integral_ = 0.0
        self.last_time_ = time.time()
        
        self.get_logger().info("Autonomous Drive Node Initialized")

    def scan_callback(self, msg):
        left_distance = min(msg.ranges[0:len(msg.ranges)//2])  # Min distance on the left side
        right_distance = min(msg.ranges[len(msg.ranges)//2:])  # Min distance on the right side

        centerline = (left_distance + right_distance) / 2.0
        deviation = (left_distance - right_distance) / self.track_width_
        
        current_time = time.time()
        dt = current_time - self.last_time_
        
        # PID calculations
        error = deviation
        self.integral_ += error * dt
        derivative = (error - self.last_error_) / dt
        
        control = self.kp * error + self.ki * self.integral_ + self.kd * derivative
        
        self.last_error_ = error
        self.last_time_ = current_time

        twist = Twist()
        twist.linear.x = 7.0  # Fixed speed adjust based on stability
        twist.angular.z = -control  # Adjust steering based on PID control
        
        self.publisher_.publish(twist)
        self.get_logger().info(f"Publishing Twist: Linear={twist.linear.x} Angular={twist.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
