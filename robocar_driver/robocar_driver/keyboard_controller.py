#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        self.max_speed = 10.0
        self.speed_increment = 1.0
        self.max_turn_speed = 3.0
        
        self.linear_acceleration = 0.2
        self.angular_acceleration = 0.25

        self.angular_decay = 0.88
        
        self.timer = self.create_timer(0.02, self.update_and_publish)  # 50Hz
        
        self.last_linear = 0.0
        self.last_angular = 0.0
        
        self.get_logger().info('Keyboard Controller started!')
        self.get_logger().info('''
Controls:
  W/S - Forward/Backward (increase/decrease speed)
  A/D - Turn left/right (HOLD to maintain turn)
  Q/E - Decrease/Increase max speed
  SPACE - Stop all
  ESC - Exit
''')

    def smooth_transition(self, current, target, acceleration):
        """Transition progressive vers la vitesse cible"""
        diff = target - current
        
        if abs(diff) < acceleration:
            return target
        elif diff > 0:
            return current + acceleration
        else:
            return current - acceleration

    def update_and_publish(self):
        """Mettre à jour les vitesses avec accélération progressive et publier"""
        self.current_linear = self.smooth_transition(
            self.current_linear, 
            self.target_linear, 
            self.linear_acceleration
        )
        
        self.current_angular = self.smooth_transition(
            self.current_angular, 
            self.target_angular, 
            self.angular_acceleration
        )
        
        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular
        self.publisher.publish(twist)
        
        if (abs(self.current_linear - self.last_linear) > 0.2 or
            abs(self.current_angular - self.last_angular) > 0.2):
            self.get_logger().info(
                f'Linear: {self.current_linear:.1f} | Angular: {self.current_angular:.1f}'
            )
            self.last_linear = self.current_linear
            self.last_angular = self.current_angular

    def run(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            
            last_key_time = {}
            
            while rclpy.ok():
                key_pressed = False
                
                if select.select([sys.stdin], [], [], 0.01)[0]:
                    key = sys.stdin.read(1)
                    key_pressed = True
                    
                    if key == '\x1b':  # ESC
                        self.get_logger().info('Exiting...')
                        break
                    
                    elif key == 'w' or key == 'W':
                        self.target_linear = min(self.max_speed,
                                                self.target_linear + self.speed_increment)
                    
                    elif key == 's' or key == 'S':
                        self.target_linear = max(-self.max_speed,
                                                self.target_linear - self.speed_increment)
                    
                    elif key == 'a' or key == 'A':
                        self.target_angular = self.max_turn_speed
                    
                    elif key == 'd' or key == 'D':
                        self.target_angular = -self.max_turn_speed
                    
                    elif key == 'q' or key == 'Q':
                        self.max_speed = max(1.0, self.max_speed - 1.0)
                        self.get_logger().info(f'Max speed: {self.max_speed:.1f}')
                    
                    elif key == 'e' or key == 'E':
                        self.max_speed = min(15.0, self.max_speed + 1.0)
                        self.get_logger().info(f'Max speed: {self.max_speed:.1f}')
                    
                    elif key == ' ':
                        self.target_linear = 0.0
                        self.target_angular = 0.0
                        self.get_logger().info('STOP')
                
                if not key_pressed or (key_pressed and key.lower() not in ['a', 'd']):
                    if abs(self.target_angular) > 0.01:
                        self.target_angular *= self.angular_decay
                        if abs(self.target_angular) < 0.01:
                            self.target_angular = 0.0

                rclpy.spin_once(self, timeout_sec=0)
                
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            twist = Twist()
            self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    controller = KeyboardController()
    
    try:
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
