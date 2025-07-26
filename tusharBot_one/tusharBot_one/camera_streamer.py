#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import subprocess
import cv2
import numpy as np

class CameraStreamer(Node):
    def __init__(self):
        super().__init__('camera_streamer')
        
        # Declare parameters for configuration
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('framerate', 15)
        
        # Get parameters
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.framerate = self.get_parameter('framerate').value
        
        # Initialize publisher and CV bridge
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # Start camera process
        self.start_camera_process()
        
        # Create timer for frame processing
        self.timer = self.create_timer(1.0 / self.framerate, self.timer_callback)
        
        self.get_logger().info(f"📷 Camera streamer initialized")
        self.get_logger().info(f"Resolution: {self.width}x{self.height} @ {self.framerate}fps")
        self.get_logger().info(f"Publishing to: /camera/image_raw")

    def start_camera_process(self):
        """Initialize the rpicam-vid subprocess"""
        try:
            self.process = subprocess.Popen(
                [
                    'rpicam-vid', '-t', '0', '-o', '-', 
                    '--width', str(self.width),
                    '--height', str(self.height), 
                    '--codec', 'yuv420', 
                    '--framerate', str(self.framerate)
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=self.width * self.height * 3 // 2
            )
            self.get_logger().info("Camera process started successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to start camera process: {e}")
            self.process = None

    def timer_callback(self):
        """Process camera frames and publish ROS2 messages"""
        if not self.process:
            return
            
        try:
            # Calculate YUV420p frame size
            yuv_size = int(self.width * self.height * 3 / 2)
            
            # Read raw data from camera process
            raw_data = self.process.stdout.read(yuv_size)
            
            # Validate frame data
            if not raw_data:
                self.get_logger().warning("No data received from camera")
                return
                
            if len(raw_data) < yuv_size:
                self.get_logger().warning(f"Incomplete frame: {len(raw_data)}/{yuv_size} bytes")
                return
            
            # Convert YUV420 to BGR
            yuv = np.frombuffer(raw_data, dtype=np.uint8).reshape((int(self.height * 1.5), self.width))
            bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
            
            # Create ROS2 image message
            msg = self.bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            
            # Publish the image
            self.publisher_.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing camera frame: {e}")

    def destroy_node(self):
        """Clean shutdown of camera and resources"""
        if hasattr(self, 'process') and self.process:
            try:
                self.process.terminate()
                self.process.wait(timeout=5)
                self.get_logger().info("Camera process terminated successfully")
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.get_logger().warning("Camera process killed (timeout)")
            except Exception as e:
                self.get_logger().error(f"Error terminating camera process: {e}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = CameraStreamer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
