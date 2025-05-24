# Catch Them All - TurtleSim

## Overview
This project is part of a ROS2 course for beginners offered on Udemy. The objective is of the project utilizing the Turtlesim package to simulate a scenario where a main turtle catches other randomly spawned turtles. Three custom nodes will be created: turtle_spawner for turtle management, turtle_controller for controlling the main turtle using a P controller, and leveraging the existing turtlesim_node.

## Packages
The project workspace contains the following packages:
- **turtle_bringup**
  - Launch files (`turtle_catch_app.launch.xml`) for starting and configuring the ROS2 nodes.
- **turtle_interfaces**
  - Custom ROS2 message and service definitions.
- **turtlesim_catch_them_all**
  - Contains C++ executables for spawning turtles (`turtle_spawner`) and controlling the main turtle (`turtle_controller`).


## Usage

### CPP Packages

1. **To Launch All Three Nodes**
   ```bash
   ros2 launch turtle_bringup turtle_catch_app.launch.xml
2. **Run Turtlesim**
   ```bash
   ros2 run turtlesim turtlesim_node
3. **To Spawn Turtles**
   ```bash
   ros2 run turtlesim_catch_them_all turtle_spawner
4. **To Run Turtle Controller**
   ```bash
   ros2 run turtlesim_catch_them_all turtle_controller

## Parameters for `/turtle_spawner`

- **`spawn_frequency`**
  - Description: Specifies the frequency (in Hz) at which new turtles are spawned.
  - Type: Float
  - Default Value: 1.0

