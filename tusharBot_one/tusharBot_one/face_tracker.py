#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import mediapipe as mp

class FaceTracker(Node):
    def __init__(self):
        super().__init__('face_tracker')
        
        # ROS2 setup
        self.bridge = CvBridge()
        
        # Subscribe to camera feed
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for servo commands
        self.servo_publisher = self.create_publisher(
            Int32MultiArray,
            '/servo_commands',
            10
        )
        
        # Declare parameters
        self.declare_parameter('target_x', 320)  # Center X for 640 width
        self.declare_parameter('target_y', 240)  # Center Y for 480 height  
        self.declare_parameter('pan_gain', 0.05)
        self.declare_parameter('tilt_gain', 0.05)
        self.declare_parameter('detection_confidence', 0.6)
        
        # Get parameters
        self.target_x_param = self.get_parameter('target_x').value
        self.target_y_param = self.get_parameter('target_y').value
        self.pan_k = self.get_parameter('pan_gain').value
        self.tilt_k = self.get_parameter('tilt_gain').value
        detection_confidence = self.get_parameter('detection_confidence').value
        
        # Servo state
        self.pan_angle = 90
        self.tilt_angle = 90
        
        # Initialize MediaPipe face detector
        mp_face = mp.solutions.face_detection
        self.face_detector = mp_face.FaceDetection(
            model_selection=0, 
            min_detection_confidence=detection_confidence
        )
        
        # Face tracking state
        self.face_lost_counter = 0
        self.max_face_lost = 30  # Stop tracking after 30 frames without face
        
        self.get_logger().info("📷 Face Tracker initialized")
        self.get_logger().info(f"Target: ({self.target_x_param}, {self.target_y_param})")
        self.get_logger().info(f"Control gains: pan={self.pan_k}, tilt={self.tilt_k}")
        self.get_logger().info(f"Detection confidence: {detection_confidence}")

    def image_callback(self, msg):
        """Process incoming camera images for face detection"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = cv_image.shape[:2]
            
            # Use dynamic center if using default parameters, otherwise use set parameters
            target_x = self.target_x_param if self.target_x_param != 320 else width // 2
            target_y = self.target_y_param if self.target_y_param != 240 else height // 2
            
            # Convert BGR to RGB for MediaPipe
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Perform face detection
            results = self.face_detector.process(rgb_image)
            
            if results.detections:
                self.process_face_detections(results.detections, width, height, target_x, target_y)
                self.face_lost_counter = 0
            else:
                self.handle_no_face_detected()
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def process_face_detections(self, detections, width, height, target_x, target_y):
        """Process detected faces and control servos"""
        # Use the first (most confident) detection
        detection = detections[0]
        bbox = detection.location_data.relative_bounding_box
        
        # Calculate face center
        cx = int((bbox.xmin + bbox.width / 2) * width)
        cy = int((bbox.ymin + bbox.height / 2) * height)
        
        # Calculate error from target position
        error_x = cx - target_x
        error_y = cy - target_y
        
        # Proportional control
        pan_adjustment = error_x * self.pan_k
        tilt_adjustment = error_y * self.tilt_k
        
        # Update servo angles
        self.pan_angle -= pan_adjustment
        self.tilt_angle += tilt_adjustment  # Positive tilt moves down
        
        # Clamp angles to servo limits
        self.pan_angle = max(0, min(180, int(self.pan_angle)))
        self.tilt_angle = max(0, min(180, int(self.tilt_angle)))
        
        # Send servo commands
        self.send_servo_command()
        
        # Log tracking info
        confidence = detection.score[0] if detection.score else 0.0
        self.get_logger().info(
            f"[Face] cx={cx}, cy={cy} → Pan: {self.pan_angle}, Tilt: {self.tilt_angle} (conf={confidence:.2f})"
        )

    def handle_no_face_detected(self):
        """Handle case when no face is detected"""
        self.face_lost_counter += 1
        
        if self.face_lost_counter % 15 == 0:  # Log every ~1 second at 15fps
            self.get_logger().info(f"No face detected for {self.face_lost_counter} frames")
        
        # Return to center position after losing face
        if self.face_lost_counter > self.max_face_lost:
            self.return_to_center()

    def return_to_center(self):
        """Return servos to center position when face is lost"""
        center_pan = 90
        center_tilt = 90
        
        # Gradually move to center
        if abs(self.pan_angle - center_pan) > 2:
            self.pan_angle += 2 if center_pan > self.pan_angle else -2
        
        if abs(self.tilt_angle - center_tilt) > 2:
            self.tilt_angle += 2 if center_tilt > self.tilt_angle else -2
        
        self.send_servo_command()
        
        if self.face_lost_counter % 30 == 0:  # Log every 2 seconds
            self.get_logger().info(f"Returning to center: ({self.pan_angle}, {self.tilt_angle})")

    def send_servo_command(self):
        """Send servo positions to ESP32 bridge"""
        servo_msg = Int32MultiArray()
        servo_msg.data = [self.pan_angle, self.tilt_angle]
        self.servo_publisher.publish(servo_msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = FaceTracker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("👋 Face tracker shutdown requested")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
