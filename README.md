# ROS 2 Rover5 Teleoperation

## Overview
A ROS 2-based teleoperation robot using a Rover5 2-motor chassis and two Raspberry Pis. A PS4 controller connected to the base station Pi drives the robot and rotates a servo-mounted camera, while the robot Pi streams live video back to the operator. The system uses ROS 2 Jazzy, `joy`, `teleop_twist_joy`, and custom motor/servo nodes for differential drive control, camera panning, and real-time video streaming over Wi-Fi using `ROS_DOMAIN_ID`.

## Features
* **Wireless PS4 joystick control**
* **Differential drive** for Rover5 motors
* **Servo-based camera panning** (controlled by right stick)
* **Live USB camera feed** via ROS 2
* **Distributed control** with two Raspberry Pis
* **ROS 2 multi-machine communication** over Wi-Fi
* **Modular Python nodes** for motors, camera, and servo

## Hardware
* **Chassis:** Rover5 2-motor chassis (no encoders)
* **Compute:** Raspberry Pi 4/5 (×2)
* **Input:** PS4 DualShock Controller
* **Vision:** USB Camera
* **Actuation:** Servo motor (SG90) for camera pan
* **Driver:** Motor driver (TB6612FNG / L298N)
* **Power:** Battery pack

## Software Stack
* **OS:** Ubuntu 24.04
* **Middleware:** ROS 2 Jazzy
* **Nodes:**
    * Custom Teleop using PS4 Controller
    * Custom `motor_node` (GPIO control)
    * Custom `servo_driver_node`

---

## Demo
[![Watch the video](https://img.youtube.com/vi/hqYxNxqYv4o/0.jpg)](https://www.youtube.com/watch?v=hqYxNxqYv4o)

## Quick Start / Installation

### 1. Setup Workspace
On Raspberry Pi 1 linked to Chassis, create a workspace and clone this repository:

```bash
# Create workspace and clone the repository
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone [https://github.com/AmitChy77/project.git](https://github.com/AmitChy77/project.git)

# Install dependencies (optional but recommended)
sudo apt update
sudo apt install ros-jazzy-teleop-twist-keyboard

# Build the workspace
cd ~/ros2_ws
colcon build --packages-select project
source install/setup.bash

# Set Domain ID (Ensure this matches on all machines)
export ROS_DOMAIN_ID=0
2. Running the Code
You can launch the full system or run individual nodes using the commands below:

# --- OPTION A: Launch Full System (Recommended) ---
# Starts motors, camera, servo, and RViz together
ros2 launch project robot_with_camera.launch.py


# --- OPTION B: Run Nodes Individually ---
# 1. Run Video Streamer
ros2 run project livestream

# 2. Run Motor Controller (Subscribes to /cmd_vel)
ros2 run project motor_control

# 3. Run Servo Controller
ros2 run project servo_control


# --- OPTION C: Keyboard Control (No PS4 Controller), You need to run the motor control node and livestream node individually. First, teleop_key, then livestream node and then motor control node ---
# If you don't have a joystick, use your keyboard to drive:
ros2 run teleop_twist_keyboard teleop_twist_keyboard


