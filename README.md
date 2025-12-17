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

## Quick Start / Installation

### 1. Setup Workspace
On Raspberry Pi 1 linked to Chassis, create a workspace and clone this repository:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone [https://github.com/AmitChy77/project.git](https://github.com/AmitChy77/project.git)

