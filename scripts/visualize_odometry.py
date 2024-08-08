#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arrow

class LidarOdomVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_odom_visualizer')
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.fig, self.ax = plt.subplots()
        self.scat = self.ax.scatter([], [], s=0.1, label='LiDAR Points')
        self.position_plot, = self.ax.plot([], [], 'ro', label='Robot Position')
        self.arrow = None
        self.timer = self.create_timer(0.1, self.update_plot)
        self.scan_data = None
        self.position = None
        self.orientation = None

        # Store LiDAR points
        self.lidar_points_x = []
        self.lidar_points_y = []

    def scan_callback(self, msg):
        self.scan_data = msg
        # self.get_logger().info(f"Unfiltered Ranges={self.scan_data}")

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation

    def update_plot(self):
        if self.scan_data is None or self.position is None or self.orientation is None:
            return

        angles = np.linspace(
            self.scan_data.angle_min,
            self.scan_data.angle_max,
            len(self.scan_data.ranges)
        )
        ranges = np.array(self.scan_data.ranges)
        # self.get_logger().info(f"Unfiltered = {len(ranges)}")
        # Filter out infinite readings
        finite_indices = np.isfinite(ranges)
        ranges = ranges[finite_indices]
        angles = angles[finite_indices]
        # self.get_logger().info(f"Filtered Ranges={len(ranges)}")

        # Convert polar coordinates to Cartesian coordinates
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        # self.get_logger().info(f"Publishing Twist: Linear={x} Angular={y}")

        # Transform LiDAR points to be relative to the robot's position
        robot_x = self.position.x
        robot_y = self.position.y
        robot_theta = 2 * np.arcsin(self.orientation.z)  # Assuming planar movement

        # Rotation matrix for the robot's orientation
        cos_theta = np.cos(robot_theta)
        sin_theta = np.sin(robot_theta)

        # Rotate and translate the LiDAR points
        x_rel = cos_theta * x - sin_theta * y + robot_x
        y_rel = sin_theta * x + cos_theta * y + robot_y

        # Append new points to the stored points
        self.lidar_points_x.extend(x_rel)
        self.lidar_points_y.extend(y_rel)

        # Update scatter plot with all points
        self.scat.set_offsets(np.c_[self.lidar_points_x, self.lidar_points_y])
        self.position_plot.set_data(robot_x, robot_y)
        
        # Add arrow to indicate the robot's orientation
        if self.arrow:
            self.arrow.remove()
        arrow_length = 5.0  # Length of the arrow
        self.arrow = Arrow(robot_x, robot_y, arrow_length * cos_theta, arrow_length * sin_theta, width=0.5, color='r')
        self.ax.add_patch(self.arrow)

        # Update plot limits to fit the robot's position and LiDAR data
        min_x = min(np.min(self.lidar_points_x), robot_x) - 5
        max_x = max(np.max(self.lidar_points_x), robot_x) + 5
        min_y = min(np.min(self.lidar_points_y), robot_y) - 5
        max_y = max(np.max(self.lidar_points_y), robot_y) + 5
        self.ax.set_xlim(min_x, max_x)
        self.ax.set_ylim(min_y, max_y)
        
        self.ax.draw_artist(self.ax.patch)
        self.ax.draw_artist(self.scat)
        self.ax.draw_artist(self.position_plot)
        self.ax.draw_artist(self.arrow)
        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = LidarOdomVisualizer()

    plt.ion()
    plt.legend()
    plt.show()
    
    # Initial draw
    node.fig.canvas.draw()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
