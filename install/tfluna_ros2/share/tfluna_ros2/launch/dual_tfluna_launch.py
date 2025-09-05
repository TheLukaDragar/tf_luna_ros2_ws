#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments for first TF-Luna
        DeclareLaunchArgument(
            'tfluna1_serial_port',
            default_value='/dev/ttyTHS2',
            description='Serial port for first TF-Luna sensor'
        ),
        
        DeclareLaunchArgument(
            'tfluna1_frame_id',
            default_value='tfluna1_link',
            description='Frame ID for first TF-Luna sensor'
        ),
        
        # Declare launch arguments for second TF-Luna
        DeclareLaunchArgument(
            'tfluna2_serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for second TF-Luna sensor (USB-to-serial adapter)'
        ),
        
        DeclareLaunchArgument(
            'tfluna2_frame_id',
            default_value='tfluna2_link',
            description='Frame ID for second TF-Luna sensor'
        ),
        
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for both TF-Luna sensors'
        ),
        
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='Publishing rate in Hz for both sensors'
        ),
        
        # First TF-Luna publisher node
        Node(
            package='tfluna_ros2',
            executable='tfluna_publisher',
            name='tfluna1_publisher',
            namespace='tfluna1',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('tfluna1_serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'frame_id': LaunchConfiguration('tfluna1_frame_id'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'field_of_view': 0.0349,  # 2 degrees in radians
                'min_range': 0.2,  # meters
                'max_range': 8.0,  # meters
            }],
            remappings=[
                ('tfluna/range', '/tfluna1/range'),
            ]
        ),
        
        # Second TF-Luna publisher node
        Node(
            package='tfluna_ros2',
            executable='tfluna_publisher',
            name='tfluna2_publisher',
            namespace='tfluna2',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('tfluna2_serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'frame_id': LaunchConfiguration('tfluna2_frame_id'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'field_of_view': 0.0349,  # 2 degrees in radians
                'min_range': 0.2,  # meters
                'max_range': 8.0,  # meters
            }],
            remappings=[
                ('tfluna/range', '/tfluna2/range'),
            ]
        ),
    ])
