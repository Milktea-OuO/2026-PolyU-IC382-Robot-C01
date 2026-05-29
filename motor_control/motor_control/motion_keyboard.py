#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select
import os

class MotionKeyboard(Node):
    def __init__(self):
        super().__init__('motion_keyboard')
        
        # Parameters with defaults
        self.declare_parameter('linear_vel', 0.5)
        self.declare_parameter('angular_vel', 1.0)
        self.declare_parameter('linear_step', 0.05)
        self.declare_parameter('angular_step', 0.1)
        self.declare_parameter('linear_max', 2.0)
        self.declare_parameter('angular_max', 3.0)
        
        self.linear_vel = self.get_parameter('linear_vel').value
        self.angular_vel = self.get_parameter('angular_vel').value
        self.linear_step = self.get_parameter('linear_step').value
        self.angular_step = self.get_parameter('angular_step').value
        self.linear_max = self.get_parameter('linear_max').value
        self.angular_max = self.get_parameter('angular_max').value
        
        # Publisher
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer for heartbeat (optional)
        self.timer = self.create_timer(0.1, self.heartbeat_callback)
        self.last_command_time = self.get_clock().now()
        self.timeout_seconds = 1.0
        
        # Key bindings: [linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]
        self.key_bindings = {
            'q': [1, 0, 0, 0, 0, 1],
            'w': [1, 0, 0, 0, 0, 0],
            'e': [1, 0, 0, 0, 0, -1],
            'a': [0, 0, 0, 0, 0, 1],
            's': [0, 0, 0, 0, 0, 0],
            'd': [0, 0, 0, 0, 0, -1],
            'z': [-1, 0, 0, 0, 0, -1],
            'x': [-1, 0, 0, 0, 0, 0],
            'c': [-1, 0, 0, 0, 0, 1],
        }
        
        self.speed_bindings = {
            'i': self.increase_linear,
            'o': self.decrease_linear,
            'k': self.increase_angular,
            'l': self.decrease_angular,
        }
        
        self.get_logger().info('Teleop Twist Keyboard Node Started')
        self.print_instructions()
        
    def print_instructions(self):
        print("\n" + "="*60)
        print("Teleop Twist Keyboard Control")
        print("="*60)
        print("Moving around:")
        print("   q    w    e")
        print("   a    s    d")
        print("   z    x    c")
        print("\nSpeed controls:")
        print(f"  i/o : increase/decrease linear speed (current: {self.linear_vel:.2f} m/s)")
        print(f"  j/k : increase/decrease angular speed (current: {self.angular_vel:.2f} rad/s)")
        print(f"  r   : reset speeds to default")
        print(f"  t   : print current speeds")
        print("\nControls:")
        print("  m   : stop robot")
        print("  space: stop robot and exit")
        print("  CTRL-C: quit")
        print("="*60)
        
    def increase_linear(self):
        self.linear_vel = min(self.linear_vel + self.linear_step, self.linear_max)
        self.get_logger().info(f'Linear velocity increased to {self.linear_vel:.2f} m/s')
        
    def decrease_linear(self):
        self.linear_vel = max(0, self.linear_vel - self.linear_step)
        self.get_logger().info(f'Linear velocity decreased to {self.linear_vel:.2f} m/s')
        
    def increase_angular(self):
        self.angular_vel = min(self.angular_vel + self.angular_step, self.angular_max)
        self.get_logger().info(f'Angular velocity increased to {self.angular_vel:.2f} rad/s')
        
    def decrease_angular(self):
        self.angular_vel = max(0, self.angular_vel - self.angular_step)
        self.get_logger().info(f'Angular velocity decreased to {self.angular_vel:.2f} rad/s')
        
    def reset_speeds(self):
        self.linear_vel = 0.5
        self.angular_vel = 1.0
        self.get_logger().info(f'Speeds reset to linear: {self.linear_vel:.2f} m/s, angular: {self.angular_vel:.2f} rad/s')
        
    def print_speeds(self):
        self.get_logger().info(f'Current speeds - Linear: {self.linear_vel:.2f} m/s, Angular: {self.angular_vel:.2f} rad/s')
        
    def heartbeat_callback(self):
        """Auto-stop if no command received for timeout period"""
        time_since_last = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        if time_since_last > self.timeout_seconds:
            self.publish_stop()
            
    def publish_twist(self, key):
        if key in self.key_bindings:
            twist = Twist()
            binding = self.key_bindings[key]
            
            twist.linear.x = binding[0] * self.linear_vel
            twist.linear.y = binding[1] * self.linear_vel
            twist.linear.z = binding[2] * self.linear_vel
            twist.angular.x = binding[3] * self.angular_vel
            twist.angular.y = binding[4] * self.angular_vel
            twist.angular.z = binding[5] * self.angular_vel
            
            self.publisher.publish(twist)
            self.last_command_time = self.get_clock().now()
            
            # Only show debug info for non-zero commands
            if abs(twist.linear.x) > 0 or abs(twist.angular.z) > 0:
                self.get_logger().debug(f'Published: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}')
            
        elif key in self.speed_bindings:
            self.speed_bindings[key]()
            
        elif key == 'r':
            self.reset_speeds()
            
        elif key == 't':
            self.print_speeds()
            
        elif key == 'm':
            # Stop and exit
            self.publish_stop()
            return False
            
        return True
        
    def publish_stop(self):
        twist = Twist()
        self.publisher.publish(twist)
        # Don't log too frequently
        # self.get_logger().debug('Stop command published')
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key
        
    def run(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == '\x03':  # CTRL-C
                    break
                    
                if not self.publish_twist(key):
                    break
                    
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            
        finally:
            self.publish_stop()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            self.get_logger().info('Shutting down teleop node')

def main(args=None):
    rclpy.init(args=args)
    node = MotionKeyboard()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()