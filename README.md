# Robotic Arm Project

This repository contains a robotic arm hobby project that runs both in simulation and on a physical robot, and it can be controlled through Amazon Alexa using its API.

## Project Overview

The project is organized as a ROS 2 workspace (`robotic_arm_ws`) containing multiple packages that together provide the arm description, control interfaces, motion planning, firmware communication, and remote/Alexa integration. The system supports running entirely in Gazebo simulation or connecting to hardware over serial, making it possible to iterate on software before deploying to the real robot.

## Key Packages and Responsibilities

- **robotic_arm_description** – URDF/Xacro models, meshes, and RViz configurations that describe the robot's kinematics and visuals for simulation and visualization.
- **robotic_arm_controllers** – ROS 2 control interface implementation and configuration, including controller manager launch files and hardware interface logic.
- **robotic_arm_moveit** – MoveIt 2 configuration files, kinematics parameters, motion planning plugins, and launch files for interactive and programmatic motion planning.
- **robotic_arm_bringup** – Launch files for starting either the simulated or physical robot stack, wiring together description, controllers, MoveIt, and optional visualization tools.
- **robotic_arm_firmware** – Arduino sketches and accompanying C++ nodes for the microcontroller that drives the physical servos, along with sample serial utilities.
- **robotic_arm_remote** – Remote command utilities, including an Alexa integration module (`alexa_interface.py`) and ROS nodes for handling task requests.
- **robotic_arm_msgs** – Custom ROS 2 interfaces (actions and services) that define the command and feedback API for the arm.
- **robotic_arm_utils** – Shared helper nodes and utilities such as angle conversions between Euler and quaternion representations.
- **sample_cpp** – Example ROS 2 nodes demonstrating usage patterns (publishers, services, actions, MoveIt interface) that can be used as references when extending the project.

## How the System Works

1. **Robot Description** – The URDF/Xacro files in `robotic_arm_description/urdf` define the robot model, which is consumed by both Gazebo and RViz. Meshes in the same package provide realistic visuals.
2. **Control Pipeline** – `robotic_arm_controllers` configures ROS 2 control and implements the hardware interface required to translate ROS commands into servo positions or simulation joint states.
3. **Motion Planning** – MoveIt 2, configured via `robotic_arm_moveit`, enables motion planning, collision checking, and trajectory execution for both the simulated and physical arm.
4. **Firmware Communication** – The firmware package bridges ROS 2 nodes and the Arduino-based controller through serial communication, ensuring the physical actuators follow planned trajectories.
5. **Alexa Integration** – The `robotic_arm_remote` package exposes ROS services/actions that can be triggered via Alexa skill intents. The `alexa_interface.py` module translates Alexa API requests into ROS commands, allowing voice control.
6. **Bringup** – Launch files in `robotic_arm_bringup/launch` start the appropriate combination of nodes depending on whether you target simulation (`simulated_robot.launch.py`) or hardware (`physical_robot.launch.py`).

## Running the Project

1. **Set up the workspace**
   ```bash
   cd robotic_arm_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build
   source install/setup.bash
   ```

2. **Launch in simulation**
   ```bash
   ros2 launch robotic_arm_bringup simulated_robot.launch.py
   ```
   This starts Gazebo with the robot model, controllers, and MoveIt for planning.

3. **Launch on hardware**
   ```bash
   ros2 launch robotic_arm_bringup physical_robot.launch.py serial_port:=/dev/ttyUSB0
   ```
   Replace `serial_port` with the device connected to the Arduino that runs the firmware from `robotic_arm_firmware/arduino_firmware/robot_control`.

4. **Enable Alexa control**
   ```bash
   ros2 launch robotic_arm_remote remote_interface.launch.py
   ```
   Ensure your Alexa skill is configured to send intents to the API exposed by `alexa_interface.py`, which forwards commands to the ROS action/service interfaces provided by `robotic_arm_msgs`.

## Development Tips

- Use the sample C++ nodes in `sample_cpp/src` as templates when writing new ROS 2 components.
- RViz configuration files in `robotic_arm_description/rviz` can be loaded to visualize the robot and debug motion planning results.
- Configuration files in `robotic_arm_moveit/config` and `robotic_arm_controllers/config` are good starting points for tuning joint limits, controllers, and planning parameters.
- Firmware changes should be tested on a development board using the sketches in `robotic_arm_firmware/arduino_firmware` before deploying to the main controller.

## Directory Structure

```
robotic_arm_ws/
  src/
    robotic_arm_bringup/        # Launch files for simulation and hardware bringup
    robotic_arm_controllers/    # ROS 2 control interface and configs
    robotic_arm_description/    # URDF, meshes, and visualization assets
    robotic_arm_firmware/       # Arduino code and firmware bridge nodes
    robotic_arm_moveit/         # MoveIt configuration and planning tools
    robotic_arm_msgs/           # Custom ROS 2 messages, services, and actions
    robotic_arm_remote/         # Alexa integration and remote command utilities
    robotic_arm_utils/          # Shared utility nodes
    sample_cpp/                 # Example ROS 2 nodes demonstrating APIs
```

This README provides a roadmap for exploring the project and extending it with new capabilities, whether you are working purely in simulation or deploying to the physical robotic arm.
