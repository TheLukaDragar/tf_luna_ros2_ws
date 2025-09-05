#!/usr/bin/env python3

"""
TF-Luna LiDAR ROS2 Publisher Node
Publishes sensor_msgs/Range messages from TF-Luna LiDAR sensor
Compatible with Jetson Orin Nano via UART
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, PointCloud2, PointField
from std_msgs.msg import Header
import struct
import numpy as np
import serial
import time
import math

class TFLunaPublisher(Node):
    def __init__(self):
        super().__init__('tfluna_publisher')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyTHS1')  # Jetson Orin Nano UART1 (pins 8/10)
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'tfluna_link')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('field_of_view', 0.0349)  # 2 degrees in radians
        self.declare_parameter('min_range', 0.2)  # meters
        self.declare_parameter('max_range', 8.0)  # meters
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.field_of_view = self.get_parameter('field_of_view').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        
        # Create publishers
        self.range_publisher = self.create_publisher(Range, 'tfluna/range', 10)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, 'tfluna/pointcloud', 10)
        
        # Initialize serial connection
        self.serial_conn = None
        self.init_serial()
        
        # Create timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'TF-Luna publisher started on {self.serial_port} at {self.baud_rate} baud')
        self.get_logger().info(f'Publishing at {self.publish_rate} Hz on topics: tfluna/range and tfluna/pointcloud')
        
    def init_serial(self):
        """Initialize serial connection to TF-Luna sensor"""
        try:
            self.serial_conn = serial.Serial(
                self.serial_port, 
                self.baud_rate, 
                timeout=1.0
            )
            
            if not self.serial_conn.is_open:
                self.serial_conn.open()
                
            # Clear any existing data in buffer
            self.serial_conn.reset_input_buffer()
            
            self.get_logger().info(f'Serial connection established: {self.serial_port}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize serial connection: {e}')
            self.serial_conn = None
            
    def read_tfluna_data(self):
        """
        Read distance data from TF-Luna sensor
        Returns: (distance_m, strength, temperature_c) or None if error
        """
        if not self.serial_conn:
            return None
            
        try:
            # Check if data is available
            if self.serial_conn.in_waiting > 8:
                # Read 9 bytes of data
                bytes_data = self.serial_conn.read(9)
                self.serial_conn.reset_input_buffer()
                
                # Check for valid header (0x59, 0x59)
                if len(bytes_data) == 9 and bytes_data[0] == 0x59 and bytes_data[1] == 0x59:
                    # Extract distance (cm -> m)
                    distance_cm = bytes_data[2] + bytes_data[3] * 256
                    distance_m = distance_cm / 100.0
                    
                    # Extract signal strength
                    strength = bytes_data[4] + bytes_data[5] * 256
                    
                    # Extract temperature (°C)
                    temp_raw = bytes_data[6] + bytes_data[7] * 256
                    temperature_c = (temp_raw / 8.0) - 256.0
                    
                    return distance_m, strength, temperature_c
                    
        except Exception as e:
            self.get_logger().error(f'Error reading sensor data: {e}')
            
        return None
    
    def create_pointcloud2(self, distance_m, strength, temperature_c, header):
        """
        Create a PointCloud2 message from TF-Luna data
        Creates a single point at the measured distance with intensity data
        """
        # Calculate point coordinates (sensor points forward along X-axis)
        x = distance_m
        y = 0.0
        z = 0.0
        
        # Normalize strength to 0-1 range for intensity
        intensity = min(strength / 65535.0, 1.0)
        
        # Create point cloud fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='temperature', offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Pack point data
        point_data = struct.pack('fffff', x, y, z, intensity, temperature_c)
        
        # Create PointCloud2 message
        pointcloud_msg = PointCloud2()
        pointcloud_msg.header = header
        pointcloud_msg.height = 1
        pointcloud_msg.width = 1
        pointcloud_msg.fields = fields
        pointcloud_msg.is_bigendian = False
        pointcloud_msg.point_step = 20  # 5 fields * 4 bytes each
        pointcloud_msg.row_step = 20
        pointcloud_msg.data = point_data
        pointcloud_msg.is_dense = True
        
        return pointcloud_msg
        
    def timer_callback(self):
        """Timer callback to read sensor and publish data"""
        sensor_data = self.read_tfluna_data()
        
        if sensor_data is None:
            return
            
        distance_m, strength, temperature_c = sensor_data
        
        # Create Range message
        range_msg = Range()
        range_msg.header = Header()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = self.frame_id
        
        # Set range message fields
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = self.field_of_view
        range_msg.min_range = self.min_range
        range_msg.max_range = self.max_range
        
        # Validate and set range
        if self.min_range <= distance_m <= self.max_range:
            range_msg.range = distance_m
        else:
            # Out of range - set to infinity or -infinity
            if distance_m > self.max_range:
                range_msg.range = float('inf')
            else:
                range_msg.range = -float('inf')
        
        # Publish Range message
        self.range_publisher.publish(range_msg)
        
        # Create and publish PointCloud2 message
        pointcloud_msg = self.create_pointcloud2(distance_m, strength, temperature_c, range_msg.header)
        self.pointcloud_publisher.publish(pointcloud_msg)
        
        # Log data occasionally (every 2 seconds)
        if int(time.time() * self.publish_rate) % (self.publish_rate * 2) == 0:
            self.get_logger().info(
                f'Distance: {distance_m:.2f}m, Strength: {strength}, Temp: {temperature_c:.1f}°C'
            )
            
    def destroy_node(self):
        """Clean up when node is destroyed"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info('Serial connection closed')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        tfluna_publisher = TFLunaPublisher()
        rclpy.spin(tfluna_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in TF-Luna publisher: {e}')
    finally:
        if 'tfluna_publisher' in locals():
            tfluna_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
