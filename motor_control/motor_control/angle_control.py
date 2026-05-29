import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from database.msg import MotorAngle, MotorsAngle

from database.srv import SetMotorAngle, SetMultipleMotors, GetMotorStatus

import threading
import sys

class MotorAngleControlServer(Node):
    """
    ROS2 Node that provides services for motor angle control
    and publishes commands to topics for Arduino using database.msg and database.srv
    """
    
    def __init__(self):
        super().__init__('angle_control_server')
        
        # Declare parameters
        self.declare_parameter('publish_frequency', 10.0)
        self.declare_parameter('motor_count', 2)
        
        # Get parameters
        self.publish_freq = self.get_parameter('publish_frequency').value
        self.motor_count = self.get_parameter('motor_count').value
        
        # Current state for each motor
        self.current_angles = [0.0] * self.motor_count
        self.target_angles = [0.0] * self.motor_count
        self.target_ticks = [0] * self.motor_count
        self.current_ticks = [0] * self.motor_count
        self.motor_busy = [False] * self.motor_count
        
        # QoS profile for reliable communication with Arduino
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Create publishers for individual motors using MotorAngle message
        self.motor_publishers = []
        for i in range(self.motor_count):
            motor_id = i + 1
            publisher = self.create_publisher(
                MotorAngle,
                f'/motor{motor_id}/angle',
                qos
            )
            self.motor_publishers.append(publisher)
        
        # Create multi-motor publisher using MotorsAngle message
        self.multi_publisher = self.create_publisher(
            MotorsAngle,
            '/motors/angles',
            qos
        )
        
        # Create service servers
        self.set_angle_service = self.create_service(
            SetMotorAngle,
            '/motor/set_angle',
            self.set_motor_angle_callback
        )
        
        self.set_multiple_service = self.create_service(
            SetMultipleMotors,
            '/motor/set_multiple',
            self.set_multiple_motors_callback
        )
        
        self.get_status_service = self.create_service(
            GetMotorStatus,
            '/motor/get_status',
            self.get_motor_status_callback
        )
        
        # Create timer for periodic publishing
        timer_period = 1.0 / self.publish_freq
        self.timer = self.create_timer(timer_period, self.publish_angles)
        
        # Flag to control input thread
        self.running = True
        
        # Start interactive input thread
        self.input_thread = threading.Thread(target=self.handle_input, daemon=True)
        self.input_thread.start()
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Motor Angle Control Server Started')
        self.get_logger().info(f'Controlling {self.motor_count} motor(s)')
        self.get_logger().info(f'Publishing frequency: {self.publish_freq} Hz')
        self.get_logger().info('Publishing to topics:')
        for i in range(self.motor_count):
            self.get_logger().info(f'  /motor{i+1}/angle [database.msg/MotorAngle]')
        self.get_logger().info('  /motors/angles [database.msg/MotorsAngle]')
        self.get_logger().info('-' * 60)
        self.get_logger().info('Services available:')
        self.get_logger().info('  /motor/set_angle [database.srv/SetMotorAngle]')
        self.get_logger().info('  /motor/set_multiple [database.srv/SetMultipleMotors]')
        self.get_logger().info('  /motor/get_status [database.srv/GetMotorStatus]')
        self.get_logger().info('-' * 60)
        self.get_logger().info('Commands:')
        self.get_logger().info('  <angle>              - Set motor 1 angle')
        self.get_logger().info('  <motor_id> <angle>   - Set specific motor')
        self.get_logger().info('  all <a1> <a2>...     - Set all motors')
        self.get_logger().info('  status               - Show status')
        self.get_logger().info('  stop                 - Stop all motors')
        self.get_logger().info('  q                    - Quit')
        self.get_logger().info('=' * 60)
        
    def publish_angles(self):
        """Publish current target angles to topics for Arduino using custom messages"""
        # Publish individual motor topics using MotorAngle
        for i, publisher in enumerate(self.motor_publishers):
            msg = MotorAngle()
            msg.motor_id = i + 1
            msg.angle_deg = self.target_angles[i]
            msg.ticks = self.target_ticks[i]
            publisher.publish(msg)
        
        # Publish multi-motor topic using MotorsAngle
        multi_msg = MotorsAngle()
        multi_msg.motor_ids = list(range(1, self.motor_count + 1))
        multi_msg.angles_deg = self.target_angles.copy()
        multi_msg.ticks = self.target_ticks.copy()
        self.multi_publisher.publish(multi_msg)
        
    def set_motor_angle_callback(self, request, response):
        """Service callback for setting single motor angle"""
        motor_id = request.motor_id
        angle = request.angle_deg
        
        if 1 <= motor_id <= self.motor_count:
            idx = motor_id - 1
            self.target_angles[idx] = angle
            # Calculate ticks based on angle (example: 1 degree = 10 ticks)
            self.target_ticks[idx] = int(angle * 10)
            response.success = True
            response.message = f"Motor {motor_id} set to {angle}°"
            response.current_angle = self.current_angles[idx]
            self.get_logger().info(f'Service: Motor {motor_id} target set to {angle}°')
        else:
            response.success = False
            response.message = f"Invalid motor ID: {motor_id}. Valid range: 1-{self.motor_count}"
            response.current_angle = self.current_angles[0] if self.motor_count > 0 else 0.0
            self.get_logger().warn(f'Service: Invalid motor ID {motor_id}')
            
        return response
        
    def set_multiple_motors_callback(self, request, response):
        """Service callback for setting multiple motor angles"""
        # Validate input lengths
        if len(request.motor_ids) != len(request.angles_deg):
            response.success = [False] * max(len(request.motor_ids), len(request.angles_deg))
            response.messages = ["Length mismatch between motor_ids and angles_deg"] * max(len(request.motor_ids), len(request.angles_deg))
            response.current_angles = [0.0] * max(len(request.motor_ids), len(request.angles_deg))
            self.get_logger().error('Service: Motor ID and angle list length mismatch')
            return response
        
        # Initialize response lists
        response.success = []
        response.messages = []
        response.current_angles = []
        
        # Process each motor
        for motor_id, angle in zip(request.motor_ids, request.angles_deg):
            if 1 <= motor_id <= self.motor_count:
                idx = motor_id - 1
                self.target_angles[idx] = angle
                self.target_ticks[idx] = int(angle * 10)  # Calculate ticks
                response.success.append(True)
                response.messages.append(f"Motor {motor_id} set to {angle}°")
                response.current_angles.append(self.current_angles[idx])
                self.get_logger().info(f'Service: Motor {motor_id} target set to {angle}°')
            else:
                response.success.append(False)
                response.messages.append(f"Invalid motor ID: {motor_id}. Valid range: 1-{self.motor_count}")
                response.current_angles.append(self.current_angles[motor_id - 1] if 1 <= motor_id <= self.motor_count else 0.0)
                self.get_logger().warn(f'Service: Invalid motor ID {motor_id}')
                
        return response
        
    def get_motor_status_callback(self, request, response):
        """Service callback for getting motor status"""
        response.success = True
        response.message = "Status retrieved successfully"
        
        if request.motor_id == 0:  # All motors
            response.motor_ids = list(range(1, self.motor_count + 1))
            response.current_angles = self.current_angles.copy()
            response.target_angles = self.target_angles.copy()
            response.busy_status = self.motor_busy.copy()
            response.current_ticks = self.current_ticks.copy()
            response.target_ticks = self.target_ticks.copy()
            self.get_logger().debug(f'Service: Status requested for all {self.motor_count} motors')
            
        elif 1 <= request.motor_id <= self.motor_count:
            idx = request.motor_id - 1
            response.motor_ids = [request.motor_id]
            response.current_angles = [self.current_angles[idx]]
            response.target_angles = [self.target_angles[idx]]
            response.busy_status = [self.motor_busy[idx]]
            response.current_ticks = [self.current_ticks[idx]]
            response.target_ticks = [self.target_ticks[idx]]
            self.get_logger().debug(f'Service: Status requested for motor {request.motor_id}')
            
        else:
            response.success = False
            response.message = f"Invalid motor ID: {request.motor_id}. Use 0 for all motors or 1-{self.motor_count} for specific motor"
            response.motor_ids = []
            response.current_angles = []
            response.target_angles = []
            response.busy_status = []
            response.current_ticks = []
            response.target_ticks = []
            self.get_logger().warn(f'Service: Invalid motor ID {request.motor_id}')
            
        return response
        
    def handle_input(self):
        """Handle user input from command line"""
        while self.running and rclpy.ok():
            try:
                user_input = input().strip()
                
                if not user_input:
                    continue
                    
                if user_input.lower() == 'q':
                    self.get_logger().info('Quitting...')
                    self.running = False
                    break
                    
                elif user_input.lower() == 'status':
                    self.show_status()
                    
                elif user_input.lower() == 'stop':
                    self.stop_all_motors()
                    
                elif user_input.lower().startswith('all'):
                    parts = user_input.split()
                    if len(parts) == self.motor_count + 1:
                        try:
                            angles = [float(p) for p in parts[1:]]
                            for i, angle in enumerate(angles):
                                self.target_angles[i] = angle
                                self.target_ticks[i] = int(angle * 10)
                            self.get_logger().info(f'✓ All motors set to: {angles}')
                        except ValueError:
                            self.get_logger().error('Invalid angle values')
                    else:
                        self.get_logger().error(f'Expected {self.motor_count} angles, got {len(parts)-1}')
                        
                else:
                    parts = user_input.split()
                    
                    if len(parts) == 1:
                        try:
                            angle = float(parts[0])
                            self.target_angles[0] = angle
                            self.target_ticks[0] = int(angle * 10)
                            self.get_logger().info(f'✓ Motor 1 set to: {angle}°')
                        except ValueError:
                            self.get_logger().error('Invalid input. Use: <angle>, <motor_id> <angle>, all <a1> <a2>, status, stop, or q')
                            
                    elif len(parts) == 2:
                        try:
                            motor_id = int(parts[0])
                            angle = float(parts[1])
                            if 1 <= motor_id <= self.motor_count:
                                idx = motor_id - 1
                                self.target_angles[idx] = angle
                                self.target_ticks[idx] = int(angle * 10)
                                self.get_logger().info(f'✓ Motor {motor_id} set to: {angle}°')
                            else:
                                self.get_logger().error(f'Invalid motor ID: {motor_id}. Valid range: 1-{self.motor_count}')
                        except ValueError:
                            self.get_logger().error('Invalid input. Use: <motor_id> <angle> (both numbers)')
                            
                    else:
                        self.get_logger().error('Invalid command format')
                        
            except EOFError:
                break
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f'Error in input handler: {e}')
                
    def stop_all_motors(self):
        """Stop all motors"""
        for i in range(self.motor_count):
            self.target_angles[i] = 0.0
            self.target_ticks[i] = 0
        self.get_logger().info('✓ All motors stopped (target angle set to 0°)')
        
    def show_status(self):
        """Display current motor status"""
        self.get_logger().info('=' * 40)
        self.get_logger().info('Current Motor Status:')
        self.get_logger().info('-' * 40)
        for i in range(self.motor_count):
            self.get_logger().info(f'  Motor {i+1}:')
            self.get_logger().info(f'    Current: {self.current_angles[i]:.2f}°')
            self.get_logger().info(f'    Target:  {self.target_angles[i]:.2f}°')
            self.get_logger().info(f'    Busy:    {self.motor_busy[i]}')
            self.get_logger().info(f'    Ticks:   {self.current_ticks[i]}/{self.target_ticks[i]}')
        self.get_logger().info('=' * 40)
        
    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorAngleControlServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()