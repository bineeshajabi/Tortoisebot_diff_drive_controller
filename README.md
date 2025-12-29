# TortoiseBot Differential Drive Controller (ROS 2)

## Overview

This repository demonstrates a **ROS 2 differential-drive mobile robot simulation** using **ROS 2 Control** and **Gazebo**.  
The project focuses on clean separation between **robot description**, **simulation**, and **control**, and verifies robot motion using velocity commands (`/cmd_vel`).

The robot is:
- Modeled using URDF/Xacro
- Visualized in RViz
- Simulated in Gazebo
- Controlled using a `diff_drive_controller`

---

## Repository Structure

The project is organized into two ROS 2 packages:

### `tortoisebot_description`
Responsible for **robot modeling and visualization**.

Includes:
- URDF/Xacro robot description
- Wheel joints and inertial properties
- ROS 2 Control interfaces
- RViz configuration and launch files

---

### `tortoisebot_gazebo`
Responsible for **simulation and controller integration**.

Includes:
- Empty Gazebo world
- Robot spawn logic
- Gazebo + ROS 2 Control integration
- Controller loading and activation

---

## Control Flow

Teleop Tool → /cmd_vel → diff_drive_controller → Wheel Joints → Gazebo


- `/cmd_vel` is handled entirely within ROS 2
- Gazebo receives only joint-level commands
- No direct Gazebo subscription to `/cmd_vel` is required

---

## Teleoperation

### Keyboard Teleoperation (Primary)
- Uses `teleop_twist_keyboard`
- Publishes `TwistStamped` messages to `/cmd_vel`
- Used to verify forward, backward, and rotational motion

### GUI Teleoperation (Optional)
- `rqt_robot_steering` was used only as an **alternative visualization tool**
- Confirms the controller accepts velocity commands from multiple ROS 2 tools
- Not required for core functionality
