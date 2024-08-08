#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arrow
import os

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

        # Store initial robot pose
        self.initial_position = None
        self.initial_orientation = None

        # Track the return to the initial position
        self.first_returned_to_initial = False
        self.moved_away_from_initial = False
        self.map_saved = False

    def scan_callback(self, msg):
        if not self.map_saved:
            self.scan_data = msg

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation

        # Store the initial position
        if self.initial_position is None:
            self.initial_position = self.position
            self.initial_orientation = self.orientation

    def is_back_to_initial_pose(self):
        if self.initial_position is None:
            return False

        # Calculate distance and orientation difference from the initial pose
        distance = np.sqrt(
            (self.position.x - self.initial_position.x) ** 2 +
            (self.position.y - self.initial_position.y) ** 2
        )

        # Tolerances for considering the robot to be back at the initial pose
        position_tolerance = 5.0  # meters

        return distance < position_tolerance 

    def update_plot(self):
        if self.scan_data is None or self.position is None or self.orientation is None:
            return

        if not self.map_saved:
            angles = np.linspace(
                self.scan_data.angle_min,
                self.scan_data.angle_max,
                len(self.scan_data.ranges)
            )
            ranges = np.array(self.scan_data.ranges)

            # Filter points under 24 meters
            valid_indices = ranges < 24.0
            ranges = ranges[valid_indices]
            angles = angles[valid_indices]

            # Convert polar coordinates to Cartesian coordinates
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)

            # Transform LiDAR points to be relative to the robot's position
            robot_x = self.position.x
            robot_y = self.position.y
            robot_theta = 2 * np.arctan2(self.orientation.z, self.orientation.w)

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

            # Check if the robot has moved away from the initial pose
            if not self.moved_away_from_initial and not self.is_back_to_initial_pose():
                self.moved_away_from_initial = True
                self.get_logger().info("Robot has moved away from the initial pose.")

            # Check if the robot has returned to the initial pose
            if self.moved_away_from_initial and self.is_back_to_initial_pose():
                if self.first_returned_to_initial:
                    self.save_map()
                    self.map_saved = True
                    self.get_logger().info("Mapping complete, map saved.")
                else:
                    self.first_returned_to_initial = True
                    self.get_logger().info("First return to initial pose detected.")

        # Update the robot's position plot and arrow
        robot_x = self.position.x
        robot_y = self.position.y
        robot_theta = 2 * np.arctan2(self.orientation.z, self.orientation.w)

        # Update robot's position plot
        self.position_plot.set_data(robot_x, robot_y)

        # Add arrow to indicate the robot's orientation
        if self.arrow:
            self.arrow.remove()
        arrow_length = 5.0  # Length of the arrow
        cos_theta = np.cos(robot_theta)
        sin_theta = np.sin(robot_theta)
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

    def save_map(self):
        # Create a plot for the LiDAR points
        plt.figure(figsize=(8, 8))
        
        # Plot the LiDAR points
        plt.scatter(self.lidar_points_x, self.lidar_points_y, s=1, color='black')
        
        # Set plot titles and labels (optional)
        plt.title('LiDAR Map')
        plt.xlabel('X Coordinate (meters)')
        plt.ylabel('Y Coordinate (meters)')
        plt.grid(True)
        plt.axis('equal')  # Ensure equal scaling for X and Y axes

        # Define the path to save the PNG image
        save_path = os.path.expanduser('~/ros2_ws/src/rl_race/scripts/lidar_map.png')
        
        # Save the plot as a PNG file
        plt.savefig(save_path)
        
        # Close the plot to avoid displaying it in some environments
        plt.close()

        self.get_logger().info(f"Map saved as {save_path}")

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
