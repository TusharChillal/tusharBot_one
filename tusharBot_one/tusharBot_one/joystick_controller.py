#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
import time

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')
        
        # Subscribe to joystick input
        self.joy_subscription = self.create_subscription(
            Joy, 
            '/joy', 
            self.joy_callback, 
            10
        )
        
        # Publishers to communicate with ESP32 serial bridge
        self.motor_publisher = self.create_publisher(
            Twist, 
            '/motor_commands', 
            10
        )
        
        self.servo_publisher = self.create_publisher(
            Int32MultiArray, 
            '/servo_commands', 
            10
        )
        
        # Control parameters
        self.turbo_mode = False
        self.linear_scale = 0.6      # Normal speed scaling
        self.linear_scale_turbo = 1.0  # Turbo speed scaling
        self.angular_scale = 0.4     # Normal turn scaling
        self.angular_scale_turbo = 0.7 # Turbo turn scaling
        
        # Servo positions
        self.servo1_pos = 90    # Center position
        self.servo2_pos = 90    # Center position
        self.servo_step = 10    # Degrees per button press
        
        # D-pad state tracking
        self.last_dpad_time = time.time()
        self.dpad_cooldown = 0.2  # Prevent rapid servo movement
        
        self.get_logger().info("Joystick Controller initialized")
        self.get_logger().info("Controls:")
        self.get_logger().info("  Left Stick: Drive (forward/back/turn)")
        self.get_logger().info("  D-Pad: Servo control (up/down = servo2, left/right = servo1)")
        self.get_logger().info("  A Button: Emergency stop")
        self.get_logger().info("  X Button: Turbo mode (hold)")

    def joy_callback(self, msg):
        """Process joystick input and send commands to ESP32 bridge"""
        
        # Read joystick inputs (Xbox controller layout)
        left_stick_y = msg.axes[1]      # Forward/backward
        left_stick_x = msg.axes[0]      # Left/right (for turning)
        
        # Buttons
        button_a = msg.buttons[0]       # Emergency stop
        button_x = msg.buttons[2]       # Turbo mode
        
        # D-pad for servo control
        dpad_horizontal = msg.axes[6]   # Left/right
        dpad_vertical = msg.axes[7]     # Up/down
        
        # Emergency stop - immediately send zero commands
        if button_a:
            self.send_motor_stop()
            self.get_logger().info("EMERGENCY STOP - A button pressed")
            return
        
        # Update turbo mode
        self.turbo_mode = (button_x == 1)
        
        # Send motor commands
        self.send_motor_commands(left_stick_y, left_stick_x)
        
        # Handle servo control with D-pad
        self.handle_servo_control(dpad_horizontal, dpad_vertical)

    def send_motor_commands(self, linear_input, angular_input):
        """Convert joystick input to motor commands"""
        
        # Apply deadzone
        if abs(linear_input) < 0.1:
            linear_input = 0.0
        if abs(angular_input) < 0.1:
            angular_input = 0.0
        
        # Scale inputs based on turbo mode
        if self.turbo_mode:
            linear_vel = linear_input * self.linear_scale_turbo
            angular_vel = angular_input * self.angular_scale_turbo
        else:
            linear_vel = linear_input * self.linear_scale
            angular_vel = angular_input * self.angular_scale
        
        # Create and publish Twist message
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        
        self.motor_publisher.publish(twist_msg)
        
        # Log occasionally (not every message to avoid spam)
        if hasattr(self, '_motor_log_counter'):
            self._motor_log_counter += 1
        else:
            self._motor_log_counter = 0
            
        if self._motor_log_counter % 20 == 0:  # Log every 2 seconds at 10Hz
            mode = "TURBO" if self.turbo_mode else "NORMAL"
            self.get_logger().info(f"[{mode}] Motor: linear={linear_vel:.2f}, angular={angular_vel:.2f}")

    def handle_servo_control(self, dpad_h, dpad_v):
        """Handle servo control using D-pad"""
        current_time = time.time()
        
        # Prevent rapid servo movement
        if current_time - self.last_dpad_time < self.dpad_cooldown:
            return
        
        servo_changed = False
        
        # Servo 1 control (left/right)
        if dpad_h > 0.5:  # Right
            self.servo1_pos = min(180, self.servo1_pos + self.servo_step)
            servo_changed = True
        elif dpad_h < -0.5:  # Left
            self.servo1_pos = max(0, self.servo1_pos - self.servo_step)
            servo_changed = True
        
        # Servo 2 control (up/down)
        if dpad_v > 0.5:  # Up
            self.servo2_pos = min(180, self.servo2_pos + self.servo_step)
            servo_changed = True
        elif dpad_v < -0.5:  # Down
            self.servo2_pos = max(0, self.servo2_pos - self.servo_step)
            servo_changed = True
        
        # Send servo command if position changed
        if servo_changed:
            servo_msg = Int32MultiArray()
            servo_msg.data = [self.servo1_pos, self.servo2_pos]
            self.servo_publisher.publish(servo_msg)
            
            self.get_logger().info(f"Servo positions: S1={self.servo1_pos}°, S2={self.servo2_pos}°")
            self.last_dpad_time = current_time

    def send_motor_stop(self):
        """Send emergency stop command"""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.motor_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = JoystickController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested")
    finally:
        # Send stop command before shutting down
        node.send_motor_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
