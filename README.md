# TurtleBot3 Color Follower with ROS 2 and OpenCV

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.10-yellow)
![OpenCV](https://img.shields.io/badge/OpenCV-Computer_Vision-green)
![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-orange)
![License](https://img.shields.io/badge/License-MIT-brightgreen)

---

# Overview

This project demonstrates a TurtleBot3 robot detecting and following a red object using ROS 2 Humble, OpenCV, Gazebo simulation, and Python.

The robot processes live camera images, detects a target color using HSV filtering, calculates the object position in the image frame, and publishes velocity commands to guide the robot toward the detected object.

This project focuses on robot perception, computer vision, and reactive robot control.

---

# Project Motivation

The goal of this project was to build a practical perception-based robotic behavior where a mobile robot can use camera input to understand its environment and react autonomously.

This project was developed as part of a robotics engineering portfolio focused on ROS 2, computer vision, autonomous mobile robots, and intelligent robotic systems.

---

# Demo Features

The system demonstrates:

- TurtleBot3 simulation in Gazebo
- Real-time camera image processing
- Red object detection using OpenCV
- HSV color filtering
- Object centroid tracking
- Velocity command publishing
- Autonomous object-following behavior
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
| Git & GitHub | Version control and portfolio |

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

1. Receive live camera image from TurtleBot3
2. Convert ROS image message to OpenCV format
3. Convert image from BGR to HSV color space
4. Apply red color threshold mask
5. Find the largest detected red object
6. Calculate the object center position
7. Rotate the robot to align with the object
8. Move forward when the object is centered
9. Stop or slow down when the object is not detected

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
| `/camera/image_raw` | sensor_msgs/Image | Receives camera frames |
| `/cmd_vel` | geometry_msgs/Twist | Sends velocity commands to robot |

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

## Terminal 3 — View Camera Feed

```bash
source /opt/ros/humble/setup.bash

rqt_image_view /camera/image_raw
```

---

# Debugging Commands

Check camera topic:

```bash
ros2 topic hz /camera/image_raw
```

Check velocity commands:

```bash
ros2 topic echo /cmd_vel
```

View node graph:

```bash
ros2 run rqt_graph rqt_graph
```

Launch RViz2:

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

This project involved solving several real robotics and computer vision integration challenges:

- Connecting Gazebo camera output to a ROS 2 perception node
- Converting ROS image messages into OpenCV images
- Tuning HSV thresholds for red object detection
- Filtering noise and identifying the largest valid object
- Translating image position into robot motion commands
- Debugging `/cmd_vel` publishing behavior
- Testing perception behavior in simulation
- Running ROS 2 Humble with Gazebo inside WSL2 + WSLg
- Cleaning and organizing the repository for GitHub portfolio use

---

# Project Preview

Screenshots and demo visuals will be added here:

- Gazebo TurtleBot3 simulation
- Camera stream with red object detection
- RQt image view output
- Robot following the red object
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
- Sensor-based autonomous behavior
- Gazebo simulation
- ROS 2 topic debugging
- Linux and WSL2 robotics workflow
- Git and GitHub version control
- Robotics system integration

---

# Relationship to Other Portfolio Projects

This project focuses on robot perception and reactive control.

It complements my autonomous warehouse navigation project, which focuses on Nav2, AMCL localization, global path planning, and multi-goal autonomous navigation.

Together, these projects demonstrate two important areas of robotics engineering:

| Project | Main Focus |
|---|---|
| TurtleBot3 Color Follower | Perception + control |
| Autonomous Warehouse Navigation | Navigation + localization |

---

# Future Improvements

Potential future upgrades:

- Add support for multiple target colors
- Add dynamic parameter tuning for HSV thresholds
- Add distance estimation from object size
- Add smoother PID-based following control
- Add object search behavior when target is lost
- Add camera visualization overlay
- Add launch file for full simulation + follower system
- Add demo video and GIFs

---

# Author

Sara Esmaeili

Electrical and Control Engineer focused on Robotics, Autonomous Navigation, ROS 2, AI, and Machine Learning.

GitHub:
https://github.com/Sara-Esm

---

# License

This project is licensed under the MIT License.