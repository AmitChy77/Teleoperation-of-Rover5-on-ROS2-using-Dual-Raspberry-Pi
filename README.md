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

[![Watch the video](https://img.youtube.com/vi/FQSc3KhBokg/0.jpg)](https://youtu.be/FQSc3KhBokg)

---

## Quick Start / Installation

### 1. Setup Workspace

On Raspberry Pi 1 (linked to the chassis), create a workspace and clone this repository:

```bash
mkdir -p ~/ros2_ws/src
```

```bash
cd ~/ros2_ws/src
git clone https://github.com/AmitChy77/project.git
```

Install dependencies (optional but recommended):

```bash
sudo apt update
sudo apt install ros-jazzy-teleop-twist-keyboard
```

Build the workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select project
```

Source the workspace:

```bash
source install/setup.bash
```

Set the ROS Domain ID (ensure this matches on **all machines**, Ignore this if using Keyboard only):

```bash
export ROS_DOMAIN_ID=0
```

---

### 2. Running the Code

You can launch the full system or run individual nodes using the commands below.

#### OPTION A: Launch Full System (Recommended)

Starts motors, camera, servo, and RViz together:

```bash
ros2 launch project robot_with_camera.launch.py
```

---

#### OPTION B: Run Nodes Individually

Run the video streamer:

```bash
ros2 run project livestream
```

Run the motor controller (subscribes to `/cmd_vel`):

```bash
ros2 run project motor_control
```

Run the servo controller:

```bash
ros2 run project servo_control
```

---

#### OPTION C: Keyboard Control (No PS4 Controller)

If you don’t have a joystick, you can use your keyboard. In this case, run the motor control node and livestream node **individually**.

Run keyboard teleoperation:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Then, in separate terminals, run:

```bash
ros2 run project livestream
```

```bash
ros2 run project motor_control
```
