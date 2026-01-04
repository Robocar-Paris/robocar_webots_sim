#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('🚗 Robocar Driver - Test Node démarré !')
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.counter = 0
    
    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'⏱️  Tick #{self.counter}')

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
