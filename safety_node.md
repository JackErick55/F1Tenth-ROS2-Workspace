#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publisher
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/safety_node', 10)

        # Safety parameters
        self.stop_distance = 0.3  # Stop if an obstacle is closer than this (meters)
        self.obstacle_detected = False

        # Timer to continuously check and publish emergency stop if needed
        self.create_timer(0.1, self.monitor_safety)

    def scan_callback(self, msg):
        """ Processes LiDAR data and determines if the car should stop. """
        ranges = np.array(msg.ranges)
        front_index = len(ranges) // 2  # Middle index corresponds to the front
        front_distance = ranges[front_index] if np.isfinite(ranges[front_index]) else float('inf')

        # Print only the front-facing LiDAR distance
        self.get_logger().info(f'Front LiDAR Distance: {front_distance:.2f}m')

        # Check if the car needs to stop
        self.obstacle_detected = front_distance < self.stop_distance
        if self.obstacle_detected:
            self.get_logger().warn(f'🚨 Obstacle detected! Stopping. Distance: {front_distance:.2f}m')

    def monitor_safety(self):
        """ Publishes a stop command if an obstacle is detected. """
        if self.obstacle_detected:
            self.emergency_brake()

    def emergency_brake(self):
        """ Sends a stop command to the vehicle. """
        brake_msg = AckermannDriveStamped()
        brake_msg.drive.speed = 0.0
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
