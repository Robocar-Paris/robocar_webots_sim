#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import os

class WebotsCommandBridge(Node):
    def __init__(self):
        super().__init__('webots_bridge')
        
        self.declare_parameter('fifo_path', '/tmp/robocar_input_fifo')
        fifo_path = self.get_parameter('fifo_path').value

        try:
            self.fifo_file = open(fifo_path, 'w')
            self.get_logger().info(f'Connected to FIFO: {fifo_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to open FIFO: {e}')
            raise
        

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.command_callback,
            10)
    
    def command_callback(self, msg):
        """Envoyer les commandes au contrôleur C++ via le FIFO"""
        command = f"{msg.linear.x},{msg.angular.z}\n"
        try:
            self.fifo_file.write(command)
            self.fifo_file.flush()
            self.get_logger().info(f'Sent: {command.strip()}')
        except Exception as e:
            self.get_logger().error(f'Failed to write to FIFO: {e}')
    
    def __del__(self):
        if hasattr(self, 'fifo_file'):
            self.fifo_file.close()

def main():
    rclpy.init()
    bridge = WebotsCommandBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()