#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import mediapipe as mp

class HandRoverController(Node):
    def __init__(self):
        super().__init__('hand_rover_controller')
        
        # ROS2 setup
        self.bridge = CvBridge()
        
        # Subscribe to camera feed
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers for robot control
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
        
        # Declare parameters
        self.declare_parameter('detection_confidence', 0.7)
        self.declare_parameter('tracking_confidence', 0.7)
        self.declare_parameter('pan_tolerance', 5)         # Pan angle tolerance (degrees)
        self.declare_parameter('rover_turn_speed', 0.3)    # Base turning speed
        self.declare_parameter('pan_compensation_gain', 0.02)  # How aggressively to compensate
        
        # Get parameters
        detection_conf = self.get_parameter('detection_confidence').value
        tracking_conf = self.get_parameter('tracking_confidence').value
        self.pan_tolerance = self.get_parameter('pan_tolerance').value
        self.rover_turn_speed = self.get_parameter('rover_turn_speed').value
        self.pan_compensation_gain = self.get_parameter('pan_compensation_gain').value
        
        # Initialize MediaPipe modules
        mp_hands = mp.solutions.hands
        self.hands = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=detection_conf,
            min_tracking_confidence=tracking_conf
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_hands = mp_hands
        
        # Face detection
        mp_face = mp.solutions.face_detection
        self.face_detector = mp_face.FaceDetection(
            model_selection=0,
            min_detection_confidence=0.6
        )
        
        # Control state variables
        self.single_hand_detected = False
        self.frame_width = 640
        self.frame_center_x = 320
        
        # Servo tracking variables
        self.current_pan_angle = 90   # Track current pan position
        self.target_pan_angle = 90    # Target is always center
        self.tilt_angle = 90
        
        # Face tracking gains (for servo movement)
        self.face_pan_gain = 0.05
        self.face_tilt_gain = 0.05
        
        self.get_logger().info("🤚 Hand-controlled rover with pan compensation initialized")
        self.get_logger().info(f"Pan tolerance: ±{self.pan_tolerance}°")
        self.get_logger().info(f"Pan compensation gain: {self.pan_compensation_gain}")

    def image_callback(self, msg):
        """Process camera images for hand and face detection"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = cv_image.shape[:2]
            self.frame_width = width
            self.frame_center_x = width // 2
            
            # Flip frame for natural viewing
            frame = cv2.flip(cv_image, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Process hand detection
            hand_result = self.hands.process(rgb)
            self.process_hand_detection(hand_result, frame)
            
            # If single hand detected, perform face tracking and rover compensation
            if self.single_hand_detected:
                self.process_face_tracking_with_compensation(rgb, frame, width, height)
            else:
                self.stop_rover_movement()
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def process_hand_detection(self, hand_result, frame):
        """Process hand detection results"""
        self.single_hand_detected = False
        
        if hand_result.multi_hand_landmarks:
            num_hands = len(hand_result.multi_hand_landmarks)
            
            if num_hands == 1:
                self.single_hand_detected = True
                
                # Draw hand landmarks
                for hand_landmarks, hand_info in zip(hand_result.multi_hand_landmarks, 
                                                   hand_result.multi_handedness):
                    hand_label = hand_info.classification[0].label
                    
                    self.mp_draw.draw_landmarks(frame, hand_landmarks, 
                                              self.mp_hands.HAND_CONNECTIONS)
                    
                    coords = hand_landmarks.landmark[0]
                    h, w, _ = frame.shape
                    cx, cy = int(coords.x * w), int(coords.y * h)
                    cv2.putText(frame, f"{hand_label} - COMPENSATING", 
                              (cx - 50, cy - 20), cv2.FONT_HERSHEY_SIMPLEX,
                              1, (0, 255, 0), 2)

    def process_face_tracking_with_compensation(self, rgb_image, display_frame, width, height):
        """Track face with servo AND compensate rover movement to keep pan at 90°"""
        face_results = self.face_detector.process(rgb_image)
        
        if face_results.detections:
            detection = face_results.detections[0]
            bbox = detection.location_data.relative_bounding_box
            
            # Calculate face center
            face_cx = int((bbox.xmin + bbox.width / 2) * width)
            face_cy = int((bbox.ymin + bbox.height / 2) * height)
            
            # Calculate face tracking error
            face_error_x = face_cx - self.frame_center_x
            face_error_y = face_cy - (height // 2)
            
            # Update servo positions for face tracking
            self.update_servo_positions(face_error_x, face_error_y)
            
            # **KEY PART**: Compensate rover movement to bring pan back to 90°
            self.compensate_rover_for_pan_angle()
            
            # Draw face detection
            x = int(bbox.xmin * width)
            y = int(bbox.ymin * height)
            w = int(bbox.width * width) 
            h = int(bbox.height * height)
            cv2.rectangle(display_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            
            # Display current pan angle and compensation status
            pan_error = self.current_pan_angle - self.target_pan_angle
            cv2.putText(display_frame, f"Pan: {self.current_pan_angle}° (Error: {pan_error:+.1f}°)", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
        else:
            # No face - stop rover but keep current servo positions
            self.stop_rover_movement()

    def update_servo_positions(self, face_error_x, face_error_y):
        """Update servo positions based on face tracking"""
        # Pan servo adjustment for face tracking
        pan_adjustment = face_error_x * self.face_pan_gain
        tilt_adjustment = face_error_y * self.face_tilt_gain
        
        # Update servo angles
        self.current_pan_angle -= pan_adjustment
        self.tilt_angle += tilt_adjustment
        
        # Clamp servo angles
        self.current_pan_angle = max(0, min(180, self.current_pan_angle))
        self.tilt_angle = max(0, min(180, self.tilt_angle))
        
        # Send servo commands
        servo_msg = Int32MultiArray()
        servo_msg.data = [int(self.current_pan_angle), int(self.tilt_angle)]
        self.servo_publisher.publish(servo_msg)

    def compensate_rover_for_pan_angle(self):
        """Move rover to compensate for pan servo angle and bring it back to 90°"""
        # Calculate how far the pan is from center (90°)
        pan_error = self.current_pan_angle - self.target_pan_angle
        
        # If pan error is within tolerance, stop rover movement
        if abs(pan_error) <= self.pan_tolerance:
            self.stop_rover_movement()
            self.get_logger().info(f"Pan centered at {self.current_pan_angle:.1f}° - Rover stopped")
            return
        
        # Calculate rover compensation movement
        # If pan > 90°, rover should turn right to compensate
        # If pan < 90°, rover should turn left to compensate
        compensation_speed = abs(pan_error) * self.pan_compensation_gain
        compensation_speed = min(compensation_speed, self.rover_turn_speed)  # Limit max speed
        
        if pan_error > 0:
            # Pan is pointing right, turn rover right to compensate
            angular_velocity = -compensation_speed
            direction = "RIGHT"
        else:
            # Pan is pointing left, turn rover left to compensate  
            angular_velocity = compensation_speed
            direction = "LEFT"
        
        # Send rover movement command
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = angular_velocity
        self.motor_publisher.publish(twist_msg)
        
        self.get_logger().info(
            f"Compensating rover {direction} - Pan: {self.current_pan_angle:.1f}° "
            f"(Error: {pan_error:+.1f}°, Speed: {compensation_speed:.2f})"
        )

    def stop_rover_movement(self):
        """Stop all rover movement"""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.motor_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = HandRoverController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("👋 Hand rover controller shutdown")
        node.stop_rover_movement()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
