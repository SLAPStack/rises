#!/usr/bin/env python3

"""
Test script for the laser preprocessor node.
Publishes simulated laser scan data for testing.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class TestScanPublisher(Node):
    def __init__(self):
        super().__init__('test_scan_publisher')
        
        # Parameters
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('frame_id', 'test_laser')
        self.declare_parameter('num_points', 180)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('obstacle_start_angle', -30)
        self.declare_parameter('obstacle_end_angle', 30)
        self.declare_parameter('obstacle_distance', 2.0)
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.num_points = self.get_parameter('num_points').value
        self.max_range = self.get_parameter('max_range').value
        self.obstacle_start = math.radians(self.get_parameter('obstacle_start_angle').value)
        self.obstacle_end = math.radians(self.get_parameter('obstacle_end_angle').value)
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        
        # Publisher
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_scan)
        
        self.get_logger().info(f'Test scan publisher started for frame {self.frame_id}')
        self.get_logger().info(f'Publishing at {self.publish_rate} Hz with {self.num_points} points')
    
    def publish_scan(self):
        """Publish a test laser scan with simulated obstacles."""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        
        scan.angle_min = -math.pi / 2
        scan.angle_max = math.pi / 2
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (self.num_points - 1)
        scan.range_min = 0.1
        scan.range_max = self.max_range
        
        # Generate ranges
        ranges = []
        for i in range(self.num_points):
            angle = scan.angle_min + i * scan.angle_increment
            
            # Create obstacles in specified angular range
            if self.obstacle_start <= angle <= self.obstacle_end:
                # Add some noise to make it more realistic
                noise = np.random.normal(0, 0.05)
                range_val = self.obstacle_distance + noise
            else:
                # Far background
                range_val = self.max_range - 1.0
            
            ranges.append(max(scan.range_min, min(range_val, scan.range_max)))
        
        scan.ranges = ranges
        scan.intensities = [100.0] * len(ranges)
        
        self.scan_pub.publish(scan)

def main():
    rclpy.init()
    
    node = TestScanPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Test scan publisher stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()