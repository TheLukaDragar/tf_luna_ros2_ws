#!/usr/bin/env python3
"""
High-performance C++ TF-Luna I2C launch file
Optimized for Jetson Orin Nano with Foxglove compatibility
Uses I2C Bus 7 (pins 3/5) - separate from LD19 LiDAR
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments with I2C defaults
        DeclareLaunchArgument(
            'i2c_bus',
            default_value='7',
            description='I2C bus number (7 = pins 3/5 on Jetson Orin Nano)'
        ),
        
        DeclareLaunchArgument(
            'i2c_address',
            default_value='0x10',
            description='TF-Luna I2C address (default: 0x10)'
        ),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value='tfluna_link',
            description='TF frame ID for sensor data'
        ),
        
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='Publishing rate in Hz'
        ),
        
        DeclareLaunchArgument(
            'field_of_view',
            default_value='0.0349',  # 2 degrees
            description='Sensor field of view in radians'
        ),
        
        DeclareLaunchArgument(
            'min_range',
            default_value='0.2',
            description='Minimum detection range in meters'
        ),
        
        DeclareLaunchArgument(
            'max_range',
            default_value='8.0',
            description='Maximum detection range in meters (90% reflectivity)'
        ),
        
        # High-performance C++ TF-Luna I2C Publisher
        Node(
            package='tfluna_ros2',
            executable='tfluna_i2c_publisher_cpp',
            name='tfluna_i2c_publisher_cpp',
            output='screen',
            parameters=[{
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'i2c_address': LaunchConfiguration('i2c_address'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'field_of_view': LaunchConfiguration('field_of_view'),
                'min_range': LaunchConfiguration('min_range'),
                'max_range': LaunchConfiguration('max_range'),
                'read_timeout_ms': 50,      # Fast timeout for high performance
                'max_read_attempts': 2,     # Quick retry for robustness
            }],
            # Set process priority for real-time performance
            additional_env={'RCUTILS_LOGGING_BUFFERED_STREAM': '1'},
        ),
        
        # Static Transform Publisher for TF-Luna positioning
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tfluna_static_transform_publisher',
            output='screen',
            arguments=[
                '0.1', '0', '0.05',    # Translation: 10cm forward, 5cm up
                '0', '0', '0',         # Rotation: no rotation
                'base_link',           # Parent frame
                LaunchConfiguration('frame_id')  # Child frame
            ]
        )
    ])

