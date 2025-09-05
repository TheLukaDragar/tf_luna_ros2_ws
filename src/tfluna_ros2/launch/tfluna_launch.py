#!/usr/bin/env python3
"""
High-performance C++ TF-Luna launch file
Optimized for Jetson Orin Nano with Foxglove compatibility
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments with optimized defaults
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyTHS2',
            description='Serial port for TF-Luna sensor (Jetson UART2 pins 32/33 - separate from LD19)'
        ),
        
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Serial baud rate (TF-Luna default: 115200)'
        ),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value='tfluna_link',
            description='TF frame ID for sensor data'
        ),
        
        DeclareLaunchArgument(
            'publish_rate',
            default_value='100.0',  # Higher rate for C++ performance
            description='Publishing rate in Hz (C++ can handle higher rates)'
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
        
        # High-performance C++ TF-Luna Publisher
        Node(
            package='tfluna_ros2',
            executable='tfluna_publisher_cpp',
            name='tfluna_publisher_cpp',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
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
