# Reactive Robot

A ROS 2 package for a reactive mobile robot simulation using Gazebo and `ros_gz_sim`.

## Authors

- João Proença
- Joana Pimenta
- Martinas Mihai Andrei

## Project Overview

This package provides a simple reactive robot system with:

- `radar_node` for laser scan processing
- `decision_node` for control decision making
- `movement_node` for sending velocity commands
- `sim.launch.py` for spawning the robot in Gazebo and bridging topics through `ros_gz_bridge`

The package also includes world files and the robot model used for the simulation.

## Repository Structure

- `launch/`
  - `sim.launch.py` - launch file for Gazebo simulation and ROS2 nodes
- `reactive_robot/`
  - `__init__.py`
  - `decision_node.py`
  - `movement_node.py`
  - `plot_res.py`
  - `plot_trajectory.py`
  - `radar_node.py`
  - `trajectory_logger.py`
- `resource/` - package resource marker
- `test/` - tests for code style and package validation
- `worlds/` - Gazebo world files and robot model files
- `package.xml` - ROS package manifest
- `setup.py` - Python packaging and entry points
- `setup.cfg` - install configuration
- `cmds.txt` - helper commands or notes

## Requirements

The package depends on ROS 2 and related Gazebo integration packages.

Base dependencies from `package.xml` and `setup.py`:

- `rclpy`
- `geometry_msgs`
- `sensor_msgs`
- `launch`
- `launch_ros`
- `ros_gz_sim`
- `ros_gz_bridge`
- `setuptools`

Test dependencies:

- `pytest`
- `ament_copyright`
- `ament_flake8`
- `ament_pep257`
- `python3-pytest`

## Installation

1. Source your ROS 2 installation:

```bash
source /opt/ros/<distro>/setup.bash
```

2. Place this package in a ROS 2 workspace `src/` directory.

```bash
cd /path/to/ros2_ws
cp -r /path/to/reactive_robot src/
```

3. Build the workspace:

```bash
colcon build --packages-select reactive_robot
```

4. Source the workspace install overlay:

```bash
source install/setup.bash
```

## Running the Simulation

Launch the default Gazebo world and the reactive robot stack:

```bash
ros2 launch reactive_robot sim.launch.py
```

Or explicitly launch the default world with:

```bash
ros2 launch reactive_robot sim.launch.py old_5:=false
```

To launch the older world variant:

```bash
ros2 launch reactive_robot sim.launch.py old_5:=true
```

The launch file starts:

- Gazebo simulation via `ros_gz_sim`
- `ros_gz_bridge` for `/clock`, `/scan`, and `/cmd_vel`
- `radar_node`
- `decision_node`
- `movement_node`

## Running Individual Nodes

You can also run nodes individually after building and sourcing the workspace:

```bash
ros2 run reactive_robot radar_node
ros2 run reactive_robot decision_node
ros2 run reactive_robot movement_node
```

## Notes

- `sim.launch.py` uses random safe spawn zones for the robot.
- The Gazebo bridge configuration is defined in `launch/sim.launch.py`.