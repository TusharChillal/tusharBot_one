#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for ESP32 communication'
    )
    
    camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='640',
        description='Camera frame width'
    )
    
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='480',
        description='Camera frame height'
    )
    
    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='15',
        description='Camera frame rate'
    )
    
    # Face tracking parameters
    face_confidence_arg = DeclareLaunchArgument(
        'face_confidence',
        default_value='0.6',
        description='Face detection confidence threshold'
    )
    
    face_pan_gain_arg = DeclareLaunchArgument(
        'face_pan_gain',
        default_value='0.05',
        description='Face tracking pan servo control gain'
    )
    
    face_tilt_gain_arg = DeclareLaunchArgument(
        'face_tilt_gain',
        default_value='0.05',
        description='Face tracking tilt servo control gain'
    )
    
    # Hand tracking parameters
    hand_detection_confidence_arg = DeclareLaunchArgument(
        'hand_detection_confidence',
        default_value='0.7',
        description='Hand detection confidence threshold'
    )
    
    hand_tracking_confidence_arg = DeclareLaunchArgument(
        'hand_tracking_confidence',
        default_value='0.7',
        description='Hand tracking confidence threshold'
    )
    
    pan_tolerance_arg = DeclareLaunchArgument(
        'pan_tolerance',
        default_value='5',
        description='Pan angle tolerance in degrees'
    )
    
    rover_turn_speed_arg = DeclareLaunchArgument(
        'rover_turn_speed',
        default_value='0.3',
        description='Rover turning speed for alignment'
    )
    
    pan_compensation_gain_arg = DeclareLaunchArgument(
        'pan_compensation_gain',
        default_value='0.02',
        description='Pan compensation control gain'
    )
    
    # Control mode selection
    enable_face_tracking_arg = DeclareLaunchArgument(
        'enable_face_tracking',
        default_value='true',
        description='Enable face tracking node'
    )
    
    enable_hand_control_arg = DeclareLaunchArgument(
        'enable_hand_control',
        default_value='false',
        description='Enable hand rover controller node'
    )
    
    # Get launch configurations
    serial_port = LaunchConfiguration('serial_port')
    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')
    camera_fps = LaunchConfiguration('camera_fps')
    face_confidence = LaunchConfiguration('face_confidence')
    face_pan_gain = LaunchConfiguration('face_pan_gain')
    face_tilt_gain = LaunchConfiguration('face_tilt_gain')
    hand_detection_confidence = LaunchConfiguration('hand_detection_confidence')
    hand_tracking_confidence = LaunchConfiguration('hand_tracking_confidence')
    pan_tolerance = LaunchConfiguration('pan_tolerance')
    rover_turn_speed = LaunchConfiguration('rover_turn_speed')
    pan_compensation_gain = LaunchConfiguration('pan_compensation_gain')
    enable_face_tracking = LaunchConfiguration('enable_face_tracking')
    enable_hand_control = LaunchConfiguration('enable_hand_control')
    
    return LaunchDescription([
        # Launch arguments
        serial_port_arg,
        camera_width_arg,
        camera_height_arg,
        camera_fps_arg,
        face_confidence_arg,
        face_pan_gain_arg,
        face_tilt_gain_arg,
        hand_detection_confidence_arg,
        hand_tracking_confidence_arg,
        pan_tolerance_arg,
        rover_turn_speed_arg,
        pan_compensation_gain_arg,
        enable_face_tracking_arg,
        enable_hand_control_arg,
        
        # ESP32 Serial Bridge Node (always running)
        Node(
            package='tusharBot_one',
            executable='esp32_serial_bridge',
            name='esp32_serial_bridge',
            output='screen',
            parameters=[{
                'serial_port': serial_port
            }]
        ),
        
        # Camera Streamer Node (always running)
        Node(
            package='tusharBot_one',
            executable='camera_streamer',
            name='camera_streamer',
            output='screen',
            parameters=[{
                'width': camera_width,
                'height': camera_height,
                'framerate': camera_fps
            }]
        ),
        
        # Face Tracker Node (conditional)
        Node(
            package='tusharBot_one',
            executable='face_tracker',
            name='face_tracker',
            output='screen',
            condition=IfCondition(enable_face_tracking),
            parameters=[{
                'target_x': 320,
                'target_y': 240,
                'pan_gain': face_pan_gain,
                'tilt_gain': face_tilt_gain,
                'detection_confidence': face_confidence
            }]
        ),
        
        # Hand Rover Controller Node (conditional)
        Node(
            package='tusharBot_one',
            executable='hand_rover_controller',
            name='hand_rover_controller',
            output='screen',
            condition=IfCondition(enable_hand_control),
            parameters=[{
                'detection_confidence': hand_detection_confidence,
                'tracking_confidence': hand_tracking_confidence,
                'pan_tolerance': pan_tolerance,
                'rover_turn_speed': rover_turn_speed,
                'pan_compensation_gain': pan_compensation_gain
            }]
        ),
    ])
