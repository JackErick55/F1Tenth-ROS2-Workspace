#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import os

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        
        # Callback groups for proper prioritization
        self.safety_cb_group = MutuallyExclusiveCallbackGroup()
        self.control_cb_group = MutuallyExclusiveCallbackGroup()

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10, callback_group=self.safety_cb_group)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10, callback_group=self.safety_cb_group)
        self.create_subscription(AckermannDriveStamped, '/controller_drive', self.controller_callback, 10, callback_group=self.control_cb_group)

        # Publisher
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', qos)
        
        self.current_speed = 0.0  # Car's current velocity
        self.safe_to_drive = True # Flag for safety override
        
        # Parameters
        self.braking_acceleration = -5.0  # Deceleration in m/s² (adjust as needed)
        self.safety_margin = 0.35  # Extra margin in meters

    def odom_callback(self, msg):
        """ Updates the current vehicle speed from odometry data. """
        self.current_speed = msg.twist.twist.linear.x  

    def scan_callback(self, msg):
        """ Processes LiDAR data and determines if emergency braking is needed. """
        if self.current_speed <= 0:
            return  # No need to check if the car is stopped

        # Compute stopping distance: d = v² / (2 * |a|)
        stopping_distance = (self.current_speed ** 2) / (2 * abs(self.braking_acceleration))
        stopping_distance += self.safety_margin  # Add margin

        ranges = np.array(msg.ranges)
        min_distance = np.nanmin(ranges[np.isfinite(ranges)])  # Find closest obstacle

        # Determine if braking is necessary
        if min_distance < stopping_distance:
            self.get_logger().warn(f'🚨 EMERGENCY BRAKING! Stopping Dist: {stopping_distance:.2f}m, Obstacle: {min_distance:.2f}m')
            self.safe_to_drive = False
            self.emergency_brake()
        else:
            self.safe_to_drive = True

    def controller_callback(self, msg):
        """ Intercepts controller commands and only forwards if it's safe. """
        if self.safe_to_drive:
            self.get_logger().info("✅ Forwarding controller command.")
            self.drive_pub.publish(msg)
        else:
            self.get_logger().warn("🚨 Safety override: Stopping vehicle!")
            self.emergency_brake()

    def emergency_brake(self):
        """ Publishes multiple braking commands to override the controller. """
        self.get_logger().warn("🚨 EMERGENCY BRAKING! Forcing stop.")

        brake_msg = AckermannDriveStamped()
        brake_msg.drive.speed = 0.0
        brake_msg.drive.acceleration = self.braking_acceleration
	    
        # Also try publishing to the brake topic
        brake_signal = Float64()
        brake_signal.data = 1.0

        for _ in range(10):  # Publish multiple times to ensure it takes effect
            self.drive_pub.publish(brake_msg)
            self.brake_pub.publish(brake_signal)
            self.get_logger().warn("🚨 Emergency Stop Command Sent!")
            rclpy.sleep(0.1)  # Short delay to ensure it's received


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
