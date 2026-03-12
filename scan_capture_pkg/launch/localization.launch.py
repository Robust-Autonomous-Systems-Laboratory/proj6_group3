#!/usr/bin/env python3
"""
Localization launch file for Project 6.

Starts the EKF localization node, which fuses wheel encoders and IMU
and publishes directly to /localization/pose
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jasmitte_proj4',
            executable='localization_node',
            name='localization_node',
            output='screen',
        )
    ])
