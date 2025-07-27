# 🤖 TusharBot One – ROS2 Modular Rover Platform

**TusharBot One** is a modular, ROS2-powered differential drive rover platform designed for rapid experimentation in robotics, teleoperation, and perception-based control. It is built with a scalable architecture, efficient power distribution, and clear separation of hardware/software components — making it ideal for extending into SLAM, GPS navigation, or autonomous exploration.

---

## 🚀 Key Highlights

- 🔧 **Modular ROS2 Architecture** – Independent nodes for control, serial communication, and vision
- 🔌 **Custom Power System** – Li-ion battery + BMS + buck converters for clean, reliable power
- 🎮 **Joystick & Keyboard Teleoperation** – Integrates `teleop_twist_joy` and `teleop_twist_keyboard`
- 📷 **Vision-Based Tracking** – Face and hand tracking nodes included for interactive robotics
- ⚙️ **Microcontroller Flexibility** – ESP32 handles motor control via serial PWM bridge
- 🧠 **Sensor-Ready Design** – Future expansion: GPS, IMU, Lidar, SLAM, servo pan/tilt camera

---

## 🛠️ Hardware Components

| Component              | Description                                     |
|------------------------|-------------------------------------------------|
| **Main SBC**           | Raspberry Pi 5 (with ROS 2 Jazzy)               |
| **Motor Controller**   | L298N H-Bridge (MOSFET driver recommended)      |
| **Microcontroller**    | ESP32 (handles motor PWM via serial bridge)     |
| **Motors**             | 4x DC Geared Motors                             |
| **Power Supply**       | 3x 18650 Li-ion Cells (3S) + BMS                |
| **Buck Converters**    | XL4015 for Pi, Mini560 for fans/servos          |
| **Cooling**            | 40mm 5V DC Fan                                  |
| **Controller**         | USB or Bluetooth Gamepad / Joystick             |
| **Future Add-ons**     | Camera, Lidar, GPS, IMU, 2x Servo Motors        |

---

🧭 System Flow (ROS2 + ESP32 + Perception)

# Joystick Mode
[Joystick/Gamepad]
      ↓
[teleop_twist_joy Node]
      ↓
[joystick_controller.py]
      ↓
[esp32_serial_bridge.py]
      ↓
[ESP32 PWM Logic]
      ↓
[L298N Motor Driver] → Motors

# Optional Camera Tracking
[USB Camera]
      ↓
[face_tracker.py / hand_rover_controller.py]
      ↓
[Twist Messages]
      ↓
[ESP32] → Motors

---

## 📁 Project Folder Structure

```bash
tusharBot_one/
├── README.md                  # This file
├── LICENSE                    # Apache-2.0 license
├── .gitignore                 # Ignore ROS2/Python/build files
├── docs/                      # Media, diagrams, and demo video
│   ├── rover_top.jpg
│   ├── rover_side.jpg
│   ├── wiring_diagram.png
│   └── demo_video.mp4
├── firmware/                  # ESP32 motor control firmware
│   └── esp32_motor_controller.ino
├── config/                    # ROS2 param config files
│   └── joy.yaml
├── src/
│   └── tusharBot_one/         # ROS2 Python package
│       ├── launch/            # ROS2 launch files
│       │   ├── robot_control.launch.py
│       │   └── tracking.launch.py
│       ├── resource/          # Package resource index
│       │   └── tusharBot_one
│       ├── tusharBot_one/     # ROS2 nodes
│       │   ├── __init__.py
│       │   ├── esp32_serial_bridge.py
│       │   ├── joystick_controller.py
│       │   ├── hand_rover_controller.py
│       │   ├── camera_streamer.py
│       │   └── face_tracker.py
│       ├── test/              # Unit test skeletons (optional)
│       ├── package.xml        # ROS2 package manifest
│       ├── setup.py           # Colcon build setup
│       └── setup.cfg          # Python metadata
