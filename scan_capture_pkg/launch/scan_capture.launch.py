#!/usr/bin/env python3
"""
Launch file for scan capture service.

This launch file starts only the scan capture service node.
The localization node (EKF/UKF) should be launched separately.

Usage:
    ros2 launch scan_capture_pkg scan_capture.launch.py
    
Prerequisites:
    - TurtleBot3 bringup running (publishing /scan, /odom, /imu)
    - Your localization node running (publishing /localization/pose)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='data/captures',
        description='Directory to save captured scans'
    )
    
    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/localization/pose',
        description='Topic to subscribe for pose estimates'
    )
    
    # Get launch configurations
    output_dir = LaunchConfiguration('output_dir')
    pose_topic = LaunchConfiguration('pose_topic')
    
    # Scan capture service node
    scan_capture_node = Node(
        package='scan_capture_pkg',
        executable='scan_capture_node.py',
        name='scan_capture_node',
        output='screen',
        parameters=[{
            'output_dir': output_dir,
            'pose_topic': pose_topic,
            'scan_topic': '/scan'
        }]
    )
    
    return LaunchDescription([
        output_dir_arg,
        pose_topic_arg,
        scan_capture_node,
    ])
