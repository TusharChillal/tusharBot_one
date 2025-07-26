from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tusharBot_one'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tushar',
    maintainer_email='tushar@todo.todo',
    description='ROS2 package for TusharBot with ESP32 serial communication, joystick control, camera streaming, face tracking, and hand-controlled rover alignment',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_serial_bridge = tusharBot_one.esp32_serial_bridge:main',
            'joystick_controller = tusharBot_one.joystick_controller:main',
            'camera_streamer = tusharBot_one.camera_streamer:main',
            'face_tracker = tusharBot_one.face_tracker:main',
            'hand_rover_controller = tusharBot_one.hand_rover_controller:main',  # Added new node
        ],
    },
)
