#!/usr/bin/env python3
"""
Keyboard Capture Interface

Simple keyboard interface for triggering scan captures at waypoints.

Controls:
    1-9: Capture scan with that waypoint ID
    s:   Capture scan with auto-incrementing ID
    q:   Quit

Author: [Student Team]
Course: EE5531 Introduction to Robotics
Project: 6 - Waypoint Mapping
"""

import rclpy
from rclpy.node import Node
from scan_capture_pkg.srv import CaptureScan

import sys
import termios
import tty
import select


class KeyboardCaptureNode(Node):
    """
    Node providing keyboard interface for scan capture.
    """

    def __init__(self):
        super().__init__('keyboard_capture')
        
        # Service client
        self.client = self.create_client(CaptureScan, '/scan_capture/capture')
        
        # Wait for service
        self.get_logger().info('Waiting for /scan_capture/capture service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.get_logger().info('Service available!')
        
        # Auto-increment counter
        self.auto_id = 1
        
        # Print instructions
        self.print_instructions()

    def print_instructions(self):
        """Print usage instructions."""
        print('')
        print('=' * 50)
        print('KEYBOARD SCAN CAPTURE INTERFACE')
        print('=' * 50)
        print('')
        print('Controls:')
        print('  1-9  : Capture scan with that waypoint ID')
        print('  s    : Capture scan with auto-increment ID')
        print('  q    : Quit')
        print('')
        print('=' * 50)
        print(f'Next auto ID: {self.auto_id}')
        print('')

    def capture_waypoint(self, waypoint_id: int, description: str = ''):
        """
        Send capture request to service.
        
        Args:
            waypoint_id: ID of waypoint
            description: Optional description
        """
        request = CaptureScan.Request()
        request.waypoint_id = waypoint_id
        request.description = description
        
        print(f'\nCapturing waypoint {waypoint_id}...')
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                print(f'✓ {response.message}')
                print(f'  File: {response.filename}')
                pos = response.pose.pose.position
                print(f'  Pose: ({pos.x:.3f}, {pos.y:.3f})')
            else:
                print(f'✗ {response.message}')
        else:
            print('✗ Service call failed or timed out')
        
        print(f'\nNext auto ID: {self.auto_id}')

    def get_key(self, timeout=0.1):
        """
        Get a single keypress with timeout.
        
        Args:
            timeout: Time to wait for input
            
        Returns:
            Key pressed or empty string if timeout
        """
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.setraw(sys.stdin.fileno())
            
            # Check if input is available
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            
            if rlist:
                key = sys.stdin.read(1)
                return key
            else:
                return ''
                
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def run(self):
        """Main loop for keyboard input."""
        try:
            while True:
                key = self.get_key()
                
                if key == '':
                    # No input, continue
                    continue
                elif key == 'q':
                    print('\nQuitting...')
                    break
                elif key == 's':
                    self.capture_waypoint(self.auto_id)
                    self.auto_id += 1
                elif key.isdigit() and key != '0':
                    waypoint_id = int(key)
                    self.capture_waypoint(waypoint_id)
                    # Update auto_id to be one more than captured
                    self.auto_id = max(self.auto_id, waypoint_id + 1)
                else:
                    print(f'\nUnknown key: {repr(key)}')
                    print('Use 1-9, s, or q')
                    
        except KeyboardInterrupt:
            print('\nInterrupted')


def main(args=None):
    rclpy.init(args=args)
    
    node = KeyboardCaptureNode()
    
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
