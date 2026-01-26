#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select
import threading
import time

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
        self.max_turn_speed = 5.0
        
        self.linear_acceleration = 0.3
        self.angular_acceleration = 0.4
        
        self.angular_decay = 0.88
        
        self.turn_active = False
        
        self.timer = self.create_timer(0.01, self.update_and_publish)
        
        self.last_linear = 0.0
        self.last_angular = 0.0
        
        self.get_logger().info('Keyboard Controller started!')
        self.get_logger().info('''
Controls:
  W/S - Forward/Backward
  A/D - Turn left/right (HOLD)
  Q/E - Speed adjustment
  SPACE - Stop
  ESC - Exit
''')

    def smooth_transition(self, current, target, acceleration):
        """Transition progressive"""
        diff = target - current
        
        if abs(diff) < acceleration:
            return target
        elif diff > 0:
            return current + acceleration
        else:
            return current - acceleration

    def update_and_publish(self):
        """Mettre à jour et publier rapidement"""
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

    def keyboard_thread(self):
        """Thread séparé pour la lecture du clavier"""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.001)[0]:
                    key = sys.stdin.read(1)
                    
                    if key == '\x1b':  # ESC
                        break
                    
                    elif key == 'w' or key == 'W':
                        self.target_linear = min(self.max_speed,
                                                self.target_linear + self.speed_increment)
                    
                    elif key == 's' or key == 'S':
                        self.target_linear = max(-self.max_speed,
                                                self.target_linear - self.speed_increment)
                    
                    elif key == 'a' or key == 'A':
                        self.target_angular = self.max_turn_speed
                        self.turn_active = True
                    
                    elif key == 'd' or key == 'D':
                        self.target_angular = -self.max_turn_speed
                        self.turn_active = True
                    
                    elif key == 'q' or key == 'Q':
                        self.max_speed = max(1.0, self.max_speed - 1.0)
                        self.get_logger().info(f'Max speed: {self.max_speed:.1f}')
                    
                    elif key == 'e' or key == 'E':
                        self.max_speed = min(15.0, self.max_speed + 1.0)
                        self.get_logger().info(f'Max speed: {self.max_speed:.1f}')
                    
                    elif key == ' ':
                        self.target_linear = 0.0
                        self.target_angular = 0.0
                        self.turn_active = False
                        self.get_logger().info('STOP')
                
                if self.turn_active:
                    self.turn_active = False
                else:
                    if abs(self.target_angular) > 0.01:
                        self.target_angular *= self.angular_decay
                    else:
                        self.target_angular = 0.0
                
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def run(self):
        """Démarrer le contrôle au clavier"""
        kb_thread = threading.Thread(target=self.keyboard_thread, daemon=True)
        kb_thread.start()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)
            time.sleep(0.001)

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
