# Joystick Interface

This ROS2 package provides joystick-based teleoperation interfaces for robot control. It includes nodes for processing joystick inputs and controlling robot end-effectors, as well as gripper control for compatible robots.

## Overview

The `joystick_interface` package enables teleoperation of robots using standard joysticks or 3D mice (SpaceMouse). It processes input from joystick devices and publishes structured teleoperation commands that can be consumed by robot controllers.

## Nodes

### joystick_input_node

The main teleoperation node that subscribes to joystick topics and publishes teleoperation commands.

**Subscribed Topics:**
- `/joy` (sensor_msgs/Joy) - 3D joystick input
- `/spacenav/joy` (sensor_msgs/Joy) - SpaceMouse input

**Published Topics:**
- `/teleop_cmd` (extender_msgs/TeleopCommand) - Teleoperation commands with twist and mode

### franka_gripper_node

A specialized node for controlling Franka robot grippers using joystick buttons.

**Subscribed Topics:**
- `/joy` (sensor_msgs/Joy) - Joystick button inputs
- `/spacenav/joy` (sensor_msgs/Joy) - SpaceMouse button inputs

**Action Clients:**
- `/fr3_gripper/grasp` (franka_msgs/action/Grasp) - Franka gripper control

## Control Modes

The package supports different teleoperation modes:

- **TRANSLATION_ROTATION** (0) - Combined translation and rotation control
- **ROTATION** (1) - Rotation-only control
- **TRANSLATION** (2) - Translation-only control
- **BOTH** (3) - Full 6DOF control (requires `allow_full_mode: true`)

## Parameters

### Common Parameters
- `allow_full_mode` (bool, default: false) - Enable full 6DOF mode
- `start_mode` (string) - Initial control mode ("TRANSLATION_ROTATION", "ROTATION", "TRANSLATION", "BOTH")
- `linear_scale` (double) - Scale factor for linear velocity commands
- `z_axis_scale` (double) - Scale factor for Z-axis (often more sensitive)
- `angular_scale` (double) - Scale factor for angular velocity commands

### Joystick Mapping Parameters
- `joy_translation_axes` (int[]) - Axis indices for translation (X, Y, Z)
- `joy_translation_signs` (double[]) - Sign multipliers for translation axes
- `joy_rotation_axes` (int[]) - Axis indices for rotation (Roll, Pitch, Yaw)
- `joy_rotation_signs` (double[]) - Sign multipliers for rotation axes
- `mode_button_joy` (int) - Button index for mode switching

### SpaceMouse Mapping Parameters
- `spacenav_translation_axes` (int[]) - Axis indices for SpaceMouse translation
- `spacenav_translation_signs` (double[]) - Sign multipliers for SpaceMouse translation
- `spacenav_rotation_axes` (int[]) - Axis indices for SpaceMouse rotation
- `spacenav_rotation_signs` (double[]) - Sign multipliers for SpaceMouse rotation
- `mode_button_spacenav` (int) - Button index for mode switching on SpaceMouse

### Gripper Parameters (franka_gripper_node)
- `gripper_closed_width` (double) - Width when gripper is closed
- `gripper_open_width` (double) - Width when gripper is open
- `gripper_closing_speed` (double) - Speed for closing motion
- `gripper_closing_force` (double) - Force applied when closing

## Configuration Files

Pre-configured parameter files are available in the `config/` directory:
- `franka_joystick_parameters.yaml` - Parameters for Franka robots
- `kinova_joystick_parameters.yaml` - Parameters for Kinova robots
- `franka_gripper_parameters.yaml` - Gripper-specific parameters

## Usage

### Running the Teleoperation Node

```bash
ros2 run joystick_interface joystick_input_node --ros-args --params-file config/franka_joystick_parameters.yaml
```

### Running the Gripper Node

```bash
ros2 run joystick_interface franka_gripper_node --ros-args --params-file config/franka_gripper_parameters.yaml
```

## Dependencies

- ROS2 Humble
- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `extender_msgs`
- `franka_msgs` (for Franka gripper control)