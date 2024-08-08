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
        # initial_theta = 2 * np.arctan2(self.initial_orientation.z, self.initial_orientation.w)
        # current_theta = 2 * np.arctan2(self.orientation.z, self.orientation.w)
        # orientation_diff = np.abs(current_theta - initial_theta)

        # Tolerances for considering the robot to be back at the initial pose
        position_tolerance = 5.0  # meters
        # orientation_tolerance = 0.1  # radians

        # return distance < position_tolerance and orientation_diff < orientation_tolerance
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

            # Filter out infinite readings
            finite_indices = np.isfinite(ranges)
            ranges = ranges[finite_indices]
            angles = angles[finite_indices]

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
        # Define map resolution
        resolution = 0.1  # meters per grid cell

        # Get the extents of the map
        min_x = min(self.lidar_points_x)
        max_x = max(self.lidar_points_x)
        min_y = min(self.lidar_points_y)
        max_y = max(self.lidar_points_y)

        width = int((max_x - min_x) / resolution) + 1
        height = int((max_y - min_y) / resolution) + 1

        # Create an empty occupancy grid
        occupancy_grid = np.zeros((height, width))

        # Mark the LiDAR points on the occupancy grid
        for x, y in zip(self.lidar_points_x, self.lidar_points_y):
            grid_x = int((x - min_x) / resolution)
            grid_y = int((y - min_y) / resolution)
            # Check if the indices are within bounds
            if 0 <= grid_x < width and 0 <= grid_y < height:
                occupancy_grid[grid_y, grid_x] = 1
        # self.get_logger().info(f"Map dim {self.lidar_points_x} , {self.lidar_points_y}")
        self.get_logger().info(f"Occupancy grid {occupancy_grid}")
        # Save the occupancy grid as a numpy file in the current directory
        save_path = 'occupancy_grid.npy'
        np.save(save_path, occupancy_grid)
        self.get_logger().info(f"Map saved at {save_path}")

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
