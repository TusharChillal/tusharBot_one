#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

class ESP32SerialBridge(Node):
    def __init__(self):
        super().__init__('esp32_serial_bridge')
        
        # Serial connection to ESP32
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Connected to ESP32 over serial on /dev/ttyACM0")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            self.ser = None
            return
        
        # ROS2 subscribers to receive commands from other nodes
        self.motor_sub = self.create_subscription(
            Twist, 
            '/motor_commands', 
            self.motor_callback, 
            10
        )
        
        self.servo_sub = self.create_subscription(
            Int32MultiArray, 
            '/servo_commands', 
            self.servo_callback, 
            10
        )
        
        # Initialize default values
        self.left_pwm = 0.0
        self.right_pwm = 0.0
        self.servo1_pos = 90
        self.servo2_pos = 90
        
        # Timer to regularly send commands to ESP32 (10Hz)
        self.timer = self.create_timer(0.1, self.send_to_esp32)
        
        self.get_logger().info("ESP32 Serial Bridge ready - listening for commands on:")
        self.get_logger().info("  - /motor_commands (geometry_msgs/Twist)")
        self.get_logger().info("  - /servo_commands (std_msgs/Int32MultiArray)")

    def motor_callback(self, msg):
        """Receive motor commands from other nodes"""
        linear = msg.linear.x    # Forward/backward (-1.0 to 1.0)
        angular = msg.angular.z  # Left/right turn (-1.0 to 1.0)
        
        # Convert to differential drive PWM values
        left_speed = linear - angular
        right_speed = linear + angular
        
        # Scale to PWM range (-255 to 255) and clamp
        self.left_pwm = max(-255, min(255, left_speed * 255))
        self.right_pwm = max(-255, min(255, right_speed * 255))
        
        self.get_logger().info(f"Motor command received: L={self.left_pwm:.1f}, R={self.right_pwm:.1f}")

    def servo_callback(self, msg):
        """Receive servo commands from other nodes"""
        if len(msg.data) >= 2:
            # Clamp servo positions to valid range (0-180)
            self.servo1_pos = max(0, min(180, msg.data[0]))
            self.servo2_pos = max(0, min(180, msg.data[1]))
            
            self.get_logger().info(f"Servo command received: S1={self.servo1_pos}, S2={self.servo2_pos}")
        else:
            self.get_logger().warning("Servo command must contain at least 2 values")

    def send_to_esp32(self):
        """Send current commands to ESP32 via serial"""
        if self.ser and self.ser.is_open:
            # Format: "v left_pwm right_pwm servo1_pos servo2_pos\n"
            cmd = f"v {self.left_pwm:.0f} {self.right_pwm:.0f} {self.servo1_pos} {self.servo2_pos}\n"
            
            try:
                self.ser.write(cmd.encode('ascii'))
                
                # Log occasionally to avoid spam (every 5 seconds)
                if not hasattr(self, '_log_counter'):
                    self._log_counter = 0
                self._log_counter += 1
                
                if self._log_counter % 50 == 0:
                    self.get_logger().info(f"Sent to ESP32: {cmd.strip()}")
                    
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write failed: {e}")
        else:
            if not hasattr(self, '_error_logged'):
                self.get_logger().error("Serial port not available!")
                self._error_logged = True

    def destroy_node(self):
        """Clean shutdown"""
        if self.ser and self.ser.is_open:
            # Send stop command before closing
            stop_cmd = "v 0 0 90 90\n"
            self.ser.write(stop_cmd.encode('ascii'))
            self.ser.close()
            self.get_logger().info("Serial connection closed")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ESP32SerialBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
