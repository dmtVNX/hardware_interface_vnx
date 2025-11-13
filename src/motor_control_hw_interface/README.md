# README for Motor Control Hardware Interface

## Overview

The `motor_control_hw_interface` package provides the hardware interface for the Robstride quadruped robot. It integrates with the ROS 2 control framework to manage the robot's actuators and sensors, enabling effective communication between the control nodes and the hardware.

## Structure

The package consists of the following key components:

- **src/**: Contains the implementation files for the hardware interface and system management.
  - `robstride_hardware_interface.cpp`: Implements the `RobstrideHardwareInterface` class, which handles hardware interactions.
  - `robstride_system.cpp`: Implements the `RobstrideSystem` class, which coordinates the overall system behavior.

- **include/**: Contains header files for the classes defined in the `src` directory.
  - `robstride_hardware_interface.hpp`: Declares the `RobstrideHardwareInterface` class and its methods.
  - `robstride_system.hpp`: Declares the `RobstrideSystem` class and its methods.

- **config/**: Contains configuration files for the robot's controllers and the ROS 2 control framework.
  - `quadruped_controllers.yaml`: Defines parameters for the quadruped controllers.
  - `quadruped_ros2_control.yaml`: Specifies settings for the `ros2_control` framework.

- **urdf/**: Contains the URDF model of the quadruped robot.
  - `quadruped.ros2_control.xacro`: Describes the robot's physical properties and structure.

- **package.xml**: Lists package dependencies and metadata.

- **CMakeLists.txt**: Specifies how to build the package.

## Installation

To build the package, navigate to the root of your ROS 2 workspace and run the following commands:

```bash
colcon build --packages-select motor_control_hw_interface
source install/setup.bash
```

## Usage

After building the package, you can launch the robot's hardware interface and control nodes using the provided launch files. Ensure that your robot's hardware is properly connected and configured.

## Dependencies

This package depends on the following ROS 2 packages:

- `ros2_control`
- `controller_interface`
- `hardware_interface`
- `rclcpp`

Make sure these dependencies are installed in your ROS 2 environment.

## Contributing

Contributions to the `motor_control_hw_interface` package are welcome. Please submit issues and pull requests on the project's repository.

## License

This project is licensed under the MIT License. See the LICENSE file for details.