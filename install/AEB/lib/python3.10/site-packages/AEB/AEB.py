#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        # Publisher
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.current_speed = 0.0  # Car's current velocity

        # Parameters
        self.braking_acceleration = -5.0  # Deceleration in m/s² (adjust based on vehicle capability)
        self.safety_margin = 0.35  # Extra margin in meters to ensure safe stopping

    def odom_callback(self, msg):
        """ Updates the current vehicle speed from odometry data. """
        self.current_speed = msg.twist.twist.linear.x  

    def scan_callback(self, msg):
        """ Processes LiDAR scan data and computes stopping distance for emergency braking. """
        if self.current_speed <= 0:
            return  # No need to check if the car is stopped

        # Compute stopping distance using physics formula: d = v² / (2 * |a|)
        stopping_distance = (self.current_speed ** 2) / (2 * abs(self.braking_acceleration))
        stopping_distance += self.safety_margin  # Add safety margin

        ranges = np.array(msg.ranges)
        min_distance = np.nanmin(ranges[np.isfinite(ranges)])  # Find closest valid obstacle

        # Trigger emergency braking if an obstacle is within stopping distance
        if min_distance < stopping_distance:
            self.get_logger().warn(f'🚨 EMERGENCY BRAKING! Stopping Dist: {stopping_distance:.2f}m, Obstacle: {min_distance:.2f}m')
            self.emergency_brake()

    def emergency_brake(self):
        """ Publishes a braking command to stop the vehicle immediately. """
        brake_msg = AckermannDriveStamped()
        brake_msg.drive.speed = 0.0
        brake_msg.drive.acceleration = self.braking_acceleration
        self.drive_pub.publish(brake_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
