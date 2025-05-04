#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import numpy as np

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # Subscribers
        self.odometry_subscriber = self.create_subscription(Odometry, "/odom", self.odometry_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publisher
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/safety_node', 10)

        # Safety parameters
        self.braking_factor= 5.5
        self.safety_margin = 1.0  # Stop if an obstacle is closer than this (meters)
        self.obstacle_detected = False
        self.current_speed = 0.0
        self.previous_velocity = 0.0
        self.front_distance = 0

    def odometry_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x

    def scan_callback(self, msg):
        """ Processes LiDAR data and determines if the car should stop. """
        ranges = np.array(msg.ranges)
        if self.current_speed == 0:
            return

        #  This checks a cone of 15 degrees in front of the vehicle if the vehicle is moving less than 1 m/s
        if self.current_speed <= 1:
             # Consider a window of ±15 degrees around the front for better safety assessment
            center_index = len(ranges) // 2
            angle_range = 15  # Degrees (assuming evenly spaced scan)
            front_indices = np.arange(
                max(0, center_index - angle_range),
                min(len(ranges), center_index + angle_range)
            )
            # Filter out NaN, Inf, and zero values
            valid_ranges = [ranges[i] for i in front_indices if np.isfinite(ranges[i]) and ranges[i] > 0]

            if valid_ranges:
                self.front_distance = min(valid_ranges)  # Closest obstacle in front
            else:
                self.front_distance = float('inf')  # No valid readings, assume clear path

        
        front_index = len(ranges) // 2  # Middle index corresponds to the front
        self.front_distance = ranges[front_index] if np.isfinite(ranges[front_index]) else float('inf')

        # Compute stopping distance using physics formula: d = v² / (2 * |a|)
        stop_distance = (((self.current_speed ** 2)*2) / (2 * abs(self.braking_factor))) 
        stop_distance += self.safety_margin  # Add safety margin

        # Print only the front-facing LiDAR distance
        self.get_logger().info(f'Front LiDAR Distance: {self.front_distance:.2f}m, Stop Distance:  {stop_distance:.2f}m Current Speed: {self.current_speed:.2f}m/s' )

        # Check if the car needs to stop
        self.obstacle_detected = self.front_distance < stop_distance
        if self.obstacle_detected:
            self.previous_velocity = self.current_speed
            self.emergency_brake()
            self.get_logger().warn(f'🚨 Obstacle detected! Stopping. Distance: {self.front_distance:.2f}m Previous Velocity = {self.previous_velocity:.2f}')

            

    def emergency_brake(self):
        """ Sends a stop command to the vehicle. """
        brake_msg = AckermannDriveStamped()
        brake_msg.drive.speed = 0.0
        brake_msg.drive.acceleration = -100.0
        self.drive_pub.publish(brake_msg)
        self.get_logger().info('Emergency brake activated.')

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
