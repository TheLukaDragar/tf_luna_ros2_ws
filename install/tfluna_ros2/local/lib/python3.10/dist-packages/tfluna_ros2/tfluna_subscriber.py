#!/usr/bin/env python3

"""
TF-Luna LiDAR ROS2 Subscriber Node
Subscribes to sensor_msgs/Range messages from TF-Luna LiDAR sensor
Displays range data for testing and visualization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import math

class TFLunaSubscriber(Node):
    def __init__(self):
        super().__init__('tfluna_subscriber')
        
        # Create subscription
        self.subscription = self.create_subscription(
            Range,
            'tfluna/range',
            self.range_callback,
            10
        )
        
        self.get_logger().info('TF-Luna subscriber started, listening to tfluna/range')
        
    def range_callback(self, msg):
        """Callback function for range messages"""
        range_str = f"{msg.range:.2f}m"
        
        # Handle special values
        if math.isinf(msg.range):
            if msg.range > 0:
                range_str = "OUT OF RANGE (too far)"
            else:
                range_str = "OUT OF RANGE (too close)"
        elif math.isnan(msg.range):
            range_str = "NO READING"
            
        self.get_logger().info(
            f'Range: {range_str}, '
            f'Frame: {msg.header.frame_id}, '
            f'FOV: {math.degrees(msg.field_of_view):.1f}°, '
            f'Min: {msg.min_range:.1f}m, '
            f'Max: {msg.max_range:.1f}m'
        )

def main(args=None):
    rclpy.init(args=args)
    
    try:
        tfluna_subscriber = TFLunaSubscriber()
        rclpy.spin(tfluna_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        if 'tfluna_subscriber' in locals():
            tfluna_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
