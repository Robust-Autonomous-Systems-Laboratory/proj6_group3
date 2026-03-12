#!/usr/bin/env python3

"""
Scan Capture Service Node

This node provides a service to capture laser scans at waypoints.
It subscribes to /scan and /localization/pose, and when triggered,
saves the current scan as a PointCloud2 along with the pose estimate.

Author: [Student Team]
Course: EE5531 Introduction to Robotics
Project: 6 - Waypoint Mapping
"""

import math
import struct
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from scan_capture_pkg.srv import CaptureScan

import numpy as np
import os
import yaml


class ScanCaptureNode(Node):
    """
    ROS2 node providing scan capture service for waypoint mapping.

    Subscribes to /scan (LaserScan) and /localization/pose (PoseStamped),
    provides a /scan_capture/capture service (CaptureScan), and when triggered
    converts the latest scan to PointCloud2, publishes it, and saves scan data
    and pose to files in output_dir.
    """

    def __init__(self):
        super().__init__('scan_capture_node')

        # Parameters
        self.declare_parameter('output_dir', 'data/captures')
        self.declare_parameter('pose_topic', '/localization/pose')
        self.declare_parameter('scan_topic', '/scan')

        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

        os.makedirs(self.output_dir, exist_ok=True)

        # State variables
        self.latest_scan = None
        self.latest_pose = None
        self.capture_count = 0

        # Subscribers
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            scan_qos
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publishers
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/scan_capture/pointcloud',
            10
        )

        # Service
        self.capture_service = self.create_service(
            CaptureScan,
            '/scan_capture/capture',
            self.capture_callback
        )

        self.get_logger().info(
            f'Scan Capture Node started. Saving captures to: {self.output_dir}'
        )

    def scan_callback(self, msg: LaserScan):
        """Store the latest laser scan."""
        self.latest_scan = msg

    def pose_callback(self, msg: PoseStamped):
        """Store the latest pose estimate."""
        self.latest_pose = msg

    def odom_callback(self, msg: Odometry):
        """Fallback: use odometry pose if no localization pose is available."""
        if self.latest_pose is None:
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.pose = msg.pose.pose
            self.latest_pose = pose_msg

    def quaternion_to_yaw(self, q) -> float:
        """Convert quaternion orientation to yaw angle in radians."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def laserscan_to_pointcloud2(self, scan: LaserScan) -> PointCloud2:
        """Convert a LaserScan message to PointCloud2 with XYZ float32 fields."""
        points = []
        angle = scan.angle_min

        for r in scan.ranges:
            if np.isfinite(r) and scan.range_min <= r <= scan.range_max:
                x = float(r * math.cos(angle))
                y = float(r * math.sin(angle))
                z = 0.0
                points.append((x, y, z))
            angle += scan.angle_increment

        cloud = PointCloud2()
        cloud.header = scan.header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12  # 3 float32 values
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True

        buffer = bytearray()
        for x, y, z in points:
            buffer.extend(struct.pack('fff', x, y, z))
        cloud.data = bytes(buffer)

        return cloud

    def save_capture(self, waypoint_id: int, description: str,
                     scan: LaserScan, pose: PoseStamped) -> str:
        """Save captured scan and pose to a timestamped YAML + .npy file pair."""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        base_name = f'wp_{waypoint_id:02d}_{timestamp}'

        yaml_path = os.path.join(self.output_dir, f'{base_name}.yaml')
        npy_path = os.path.join(self.output_dir, f'{base_name}.npy')

        yaw = self.quaternion_to_yaw(pose.pose.orientation)

        capture_data = {
            'waypoint_id': int(waypoint_id),
            'description': description,
            'timestamp': timestamp,
            'pose': {
                'frame_id': pose.header.frame_id,
                'x': float(pose.pose.position.x),
                'y': float(pose.pose.position.y),
                'z': float(pose.pose.position.z),
                'yaw_rad': float(yaw),
            },
            'scan': {
                'frame_id': scan.header.frame_id,
                'angle_min': float(scan.angle_min),
                'angle_max': float(scan.angle_max),
                'angle_increment': float(scan.angle_increment),
                'range_min': float(scan.range_min),
                'range_max': float(scan.range_max),
                'num_ranges': int(len(scan.ranges)),
                'ranges_file': os.path.basename(npy_path),
            }
        }

        with open(yaml_path, 'w') as f:
            yaml.safe_dump(capture_data, f, sort_keys=False)

        np.save(npy_path, np.array(scan.ranges, dtype=np.float32))

        return yaml_path

    def capture_callback(self, request, response):
        """Service callback: capture the current scan and pose."""
        if self.latest_scan is None:
            response.success = False
            response.message = 'No LaserScan received yet.'
            response.filename = ''
            response.pose = PoseStamped()
            return response

        if self.latest_pose is None:
            response.success = False
            response.message = 'No pose received yet from localization or odometry.'
            response.filename = ''
            response.pose = PoseStamped()
            return response

        try:
            pointcloud_msg = self.laserscan_to_pointcloud2(self.latest_scan)
            self.pointcloud_pub.publish(pointcloud_msg)

            saved_file = self.save_capture(
                request.waypoint_id,
                request.description,
                self.latest_scan,
                self.latest_pose
            )

            self.capture_count += 1

            response.success = True
            response.message = (
                f'Captured waypoint {request.waypoint_id} successfully.'
            )
            response.filename = saved_file
            response.pose = self.latest_pose
            return response

        except Exception as e:
            response.success = False
            response.message = f'Capture failed: {str(e)}'
            response.filename = ''
            response.pose = PoseStamped()
            return response


def main(args=None):
    rclpy.init(args=args)
    node = ScanCaptureNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()