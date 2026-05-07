# TurtleBot3 Semi-Autonomous Color Follower with ROS 2 and OpenCV

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Python-ROS2-yellow)
![OpenCV](https://img.shields.io/badge/OpenCV-Computer_Vision-green)
![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-orange)
![License](https://img.shields.io/badge/License-MIT-brightgreen)

---

# Overview

This project demonstrates a semi-autonomous TurtleBot3 robot behavior using ROS 2 Humble, OpenCV, Gazebo, and Python.

The robot is manually driven through a simulated house environment. While moving, it continuously processes the camera stream to detect a red object. When the red object appears in view, the robot switches into an object-following behavior by aligning with the object and publishing velocity commands.

This project focuses on perception-triggered robot control, computer vision integration, and reactive robotic behavior using ROS 2.

---

# Project Motivation

The goal of this project was to build a practical perception-based robotic behavior where a mobile robot can use camera input to understand its environment and react autonomously.

The project combines:

- Manual robot teleoperation
- Real-time camera processing
- OpenCV computer vision
- ROS 2 communication
- Reactive robot control
- Gazebo simulation workflows

---

# Demo Features

The system demonstrates:

- TurtleBot3 simulation in Gazebo
- Manual robot teleoperation
- Real-time camera image processing
- Red object detection using OpenCV
- HSV color filtering
- Object centroid tracking
- Velocity command publishing
- Perception-triggered object-following behavior
- ROS 2 Python node development
- RQt and RViz2 debugging workflow

---

# Technologies Used

| Technology | Purpose |
|---|---|
| ROS 2 Humble | Robotics middleware |
| Python | Robot perception and control node |
| OpenCV | Image processing and color detection |
| Gazebo | Robot simulation |
| TurtleBot3 | Mobile robot platform |
| cv_bridge | ROS image to OpenCV image conversion |
| RQt Image View | Camera stream debugging |
| RViz2 | Robot and sensor visualization |
| Colcon | ROS 2 build system |
| Git & GitHub | Version control |

---

# System Architecture

```text
Gazebo Camera Sensor
        |
        v
ROS 2 Image Topic
        |
        v
cv_bridge
        |
        v
OpenCV HSV Color Filtering
        |
        v
Object Centroid Detection
        |
        v
Motion Control Logic
        |
        v
/cmd_vel Velocity Commands
        |
        v
TurtleBot3 Robot Movement
```

---

# Robot Behavior

The robot follows this perception-control workflow:

1. Manually drive the TurtleBot3 through the Gazebo house environment
2. Receive live camera images from the robot camera
3. Convert ROS image messages into OpenCV format
4. Convert image from BGR to HSV color space
5. Apply red color threshold mask
6. Detect the largest red object
7. Calculate object center position
8. Rotate robot toward detected object
9. Move forward when object is centered
10. Stop when object is lost

---

# Workspace Structure

```text
turtlebot3_color_follower/
├── README.md
├── .gitignore
└── src/
    └── turtlebot3_color_follower/
        ├── launch/
        │   └── color_detector.launch.py
        ├── package.xml
        ├── resource/
        │   └── turtlebot3_color_follower
        ├── setup.cfg
        ├── setup.py
        └── turtlebot3_color_follower/
            ├── __init__.py
            └── color_detector.py
```

---

# Important Package

## turtlebot3_color_follower

This package contains the main computer vision and robot control logic.

Main files:

```text
color_detector.py
color_detector.launch.py
package.xml
setup.py
```

---

# Main Node

## color_detector.py

The main Python ROS 2 node performs:

- Camera topic subscription
- Image conversion using cv_bridge
- HSV filtering for red object detection
- Object contour detection
- Centroid calculation
- Robot motion control
- Velocity command publishing to `/cmd_vel`

---

# Topics Used

| Topic | Type | Purpose |
|---|---|---|
| /camera/image_raw | sensor_msgs/Image | Receives camera frames |
| /cmd_vel | geometry_msgs/Twist | Sends velocity commands to robot |

---

# How to Build

```bash
cd ~/projects/turtlebot3_color_follower

source /opt/ros/humble/setup.bash

colcon build

source install/setup.bash
```

---

# Run the Project

## Terminal 1 — Launch TurtleBot3 Gazebo Simulation

```bash
cd ~/projects/turtlebot3_color_follower

source /opt/ros/humble/setup.bash

export TURTLEBOT3_MODEL=waffle_pi

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

---

## Terminal 2 — Run Color Follower Node

```bash
cd ~/projects/turtlebot3_color_follower

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch turtlebot3_color_follower color_detector.launch.py
```

---

## Terminal 3 — Manual Teleoperation

Use keyboard teleoperation to manually drive the TurtleBot3 through the house environment.

```bash
source /opt/ros/humble/setup.bash

export TURTLEBOT3_MODEL=waffle_pi

ros2 run turtlebot3_teleop teleop_keyboard
```

While the robot is manually driven, the color follower node continuously monitors the camera stream.

When a red object appears in the camera view, the robot switches into object-following behavior by publishing velocity commands to `/cmd_vel`.

---

## Terminal 4 — View Camera Feed

```bash
source /opt/ros/humble/setup.bash

rqt_image_view /camera/image_raw
```

---

# Debugging Commands

## Check camera topic

```bash
ros2 topic hz /camera/image_raw
```

## Check velocity commands

```bash
ros2 topic echo /cmd_vel
```

## View ROS 2 node graph

```bash
ros2 run rqt_graph rqt_graph
```

## Launch RViz2

```bash
ros2 launch turtlebot3_bringup rviz2.launch.py
```

---

# Example Behavior

```text
Camera image received
Red object detected
Object center calculated
Robot rotating toward object
Robot moving forward
Object lost
Robot stopping
```

---

# Challenges Solved During Development

This project involved solving several robotics and computer vision integration challenges:

- Connecting Gazebo camera output to a ROS 2 perception node
- Converting ROS image messages into OpenCV images
- Tuning HSV thresholds for red object detection
- Filtering image noise and identifying the largest valid object
- Translating image position into robot motion commands
- Debugging `/cmd_vel` publishing behavior
- Testing perception behavior in simulation
- Running ROS 2 Humble with Gazebo inside WSL2 + WSLg

---

# Project Preview

Screenshots##

- Gazebo TurtleBot3 house simulation
- Manual teleoperation through the house
- Camera stream with red object detection
- Robot switching into object-following behavior
- RQt image viewer output
- RViz2 visualization
- Terminal output showing active ROS 2 node execution

---

# Learning Outcomes

This project provided hands-on experience with:

- ROS 2 Python node development
- ROS 2 publisher/subscriber communication
- Computer vision for robotics
- OpenCV HSV color filtering
- Camera-based robot behavior
- TurtleBot3 simulation
- Gazebo camera sensor workflow
- Velocity command control
- Robotics debugging tools
- Perception-control integration

---

# Key Engineering Skills Demonstrated

- ROS 2 package development
- Python robotics programming
- OpenCV computer vision
- Real-time image processing
- Mobile robot control
- Sensor-based robot behavior
- Gazebo simulation
- ROS 2 topic debugging
- Linux and WSL2 robotics workflow
- Git and GitHub version control
- Robotics system integration

---

# Future Improvements

Potential future upgrades:

- Add support for multiple target colors
- Add dynamic parameter tuning for HSV thresholds
- Add distance estimation from object size
- Add autonomous room exploration

---

# Author

Sara Esmaeili

Electrical and Control Engineer focused on Robotics, Autonomous Navigation, ROS 2, AI, and Machine Learning.

GitHub:
https://github.com/Sara-Esm

---

# License

This project is licensed under the MIT License.