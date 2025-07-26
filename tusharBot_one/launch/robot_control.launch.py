#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments
    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick',
        default_value='true',
        description='Whether to start the joystick node'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for ESP32 communication'
    )
    
    # Get launch configuration
    use_joystick = LaunchConfiguration('use_joystick')
    serial_port = LaunchConfiguration('serial_port')
    
    return LaunchDescription([
        # Launch arguments
        use_joystick_arg,
        serial_port_arg,
        
        # ESP32 Serial Bridge Node (always starts)
        Node(
            package='tusharBot_one',
            executable='esp32_serial_bridge',
            name='esp32_serial_bridge',
            output='screen',
            parameters=[{
                'serial_port': serial_port
            }],
            remappings=[
                # You can remap topics here if needed
            ]
        ),
        
        # Joy Node (ROS2 joystick driver)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            condition=IfCondition(use_joystick),
            parameters=[{
                'device_id': 0,
                'deadzone': 0.1,
                'autorepeat_rate': 10.0
            }]
        ),
        
        # Joystick Controller Node
        Node(
            package='tusharBot_one',
            executable='joystick_controller',
            name='joystick_controller',
            output='screen',
            condition=IfCondition(use_joystick),
            parameters=[{
                'linear_scale': 0.6,
                'linear_scale_turbo': 1.0,
                'angular_scale': 0.4,
                'angular_scale_turbo': 0.7,
                'servo_step': 10
            }]
        ),
    ])
