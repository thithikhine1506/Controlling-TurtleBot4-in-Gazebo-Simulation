# Controlling-TurtleBot4-in-Gazebo-Simulation

**TurtleBot 4 Autonomous Navigation: Open-Loop Diamond Path**

This repository contains a ROS 2 implementation for controlling a TurtleBot 4 robot in a simulated (Gazebo/Rviz) environment. The goal of the project was to navigate the robot through a series of waypoints to form a precise 5-meter-per-side diamond shape using open-loop control logic.

**Project Description**
The TurtleBot4 robot is a differential wheeled robot built on top of the mobile base. It is customized to include more sensors (Lidar, camera and etc).

The task is to make the robot move in a diamond shape using open-loop control (i.e. sending commands only; no feedback). The waypoints to visit are (0, 0), (3, -4), (6, 0), and (3, 4). In other words, the robot should turn and move towards a waypoint four times. Note that the robot is supposed to stop at the origin after completing this movement, and the Python script should exit gracefully.

**Technical Implementation**

**ROS 2 Node Architecture**

Node Name: turtlebot_move

Communication: Utilized a Publisher on the cmd_vel topic using geometry_msgs/TwistStamped to send velocity commands.

Action Client: Implemented an Undock action client to handle the initial robot deployment from its charging station.

Node Name: turtlebot_move

Communication: Utilized a Publisher on the cmd_vel topic using geometry_msgs/TwistStamped to send velocity commands.

Action Client: Implemented an Undock action client to handle the initial robot deployment from its charging station.

**Open-Loop Control Logic**

Since sensors (like IMU or Odometry) were not used for feedback in this specific lab, the movement was calculated based on Time × Velocity.

Angular Movement: Calculated using atan2(dy,dx) to determine the precise turn angle between segments.

Calibration: Applied calibration factors (linear_calib: 0.94 and angular_calib: 1.05) to account for simulator friction and drag.

**Tech Stack**

Middleware: ROS 2 (Jazzy/Humble)

Language: Python

Simulation: Gazebo (Physics) & Rviz (Visualization)

**How to Run**

1.Ensure you have a ROS 2 environment sourced.

2.Clone this repository into your colcon workspace.

3.Build the package: colcon build --packages-select <pkg_name>.

4.Run the node:
ros2 run <pkg_name> open_loop_node


