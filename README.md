# quadcopter

Quadcopter simulator in ROS

## Overview

This package provides a lightweight quadcopter simulator for ROS (catkin). It implements a physics model and a simple controller, defines a small set of ROS messages, and includes example launch/config files to get started. The project is intentionally minimal so it is easy to extend — expect sensors, controllers, and launch setups to evolve.

High-level components:
- src/: C++ implementation of the physics model.
- include/: Public headers for C++ classes.
- msg/: ROS message definitions.
- launch/: Example launch files.
- config/: YAML parameter files for settings.
- doc/: Design notes and derivations.

## Requirements

- ROS (catkin) — tested with ROS 1 (Noetic)
- cmake >= 3.0.2
- Standard ROS messages (geometry_msgs)

## Building

1. Use or create a catkin workspace:

```bash 
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

2. Clone the repository into the workspace:

```bash
git clone https://github.com/dinay-kingkiller/quadcopter.git
```

3. Build the workspace:

```bash
cd ~/catkin_ws
catkin_make
# or `catkin build` if you use catkin_tools
source devel/setup.bash
```

## Running (example)

To run the simple demo that loads model and sensor params and runs the constant controller:

```bash
roslaunch quadcopter launch_constant.launch
```

This launch is basic and intended for quick debug runs. See `launch/` for other examples or to create more complex scenarios.

## ROS interface (summary)

Messages (in `msg/`):
- Motor.msg
  - float64 front
  - float64 right
  - float64 left
  - float64 back

- Sensor.msg
  - geometry_msgs/Vector3 gyroscope
  - geometry_msgs/Vector3 accelerometer

- ICM20948.msg
  - Raw register-like fields for accelerometer, gyroscope, magnetometer outputs.

Topics (example):
- Subscribe: `motor_input` (quadcopter/Motor)
- Publish: `pose` (geometry_msgs/Pose), `ICM20948` (quadcopter/ICM20948) — actual topics and additional sensor topics depend on runtime configuration.

## Configuration & parameters

Config files in `config/`:
- `model.yaml` — model parameters
- `sensor.yaml` — sensor scale/noise settings.

## Design notes

A longer design/derivation document exists at `doc/quadcopter.pdf`. It covers:
- Quaternion kinematics and orientation handling
- Rigid-body dynamics used in the model node
- Sensor modelling and how simulated physical quantities are converted into sensor outputs

## License

BSD 3-Clause — see `LICENSE`.