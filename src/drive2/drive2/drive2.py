#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class DriveStraightNode(Node):
    def __init__(self):
        super().__init__('drive2')
       
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
       
        self.drive_speed = 2.0 # Speed in meters per second
        self.steering_angle = 0.0  # Drive straight
       
        self.get_logger().info('Starting movement indefinitely...')
       
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.count = 0

    def timer_callback(self):
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = self.drive_speed
            drive_msg.drive.steering_angle = self.steering_angle
            self.drive_pub.publish(drive_msg)
            self.get_logger().info(f"Publishing: speed= {self.drive_speed:.2f}, steering_angle=0.0")


def main(args=None):
    rclpy.init(args=args)
    node = DriveStraightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
