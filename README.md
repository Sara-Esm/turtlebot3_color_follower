# TurtleBot3 Color Follower

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![ROS](https://img.shields.io/badge/ROS-2%20Humble-brightgreen)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-blue)](https://www.python.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-11-orange)](https://gazebosim.org/)
[![Colcon](https://img.shields.io/badge/Build-Colcon-blue)](https://colcon.readthedocs.io/)

---

## Overview

This ROS 2 package enables the TurtleBot3 robot to detect and follow objects of a specific color (red) using computer vision techniques.  
It processes camera feed frames in real-time and controls the robot’s motion accordingly, combining manual teleoperation with autonomous behavior for a seamless and robust user experience.

The project is tested in Ubuntu 22.04 with ROS 2 Humble running inside WSL2 + WSLg, leveraging Gazebo simulation, RQt, and RViz2 for visualization and debugging.

---

## 🎯 Key Features

- **Manual Navigation Mode** — Control the TurtleBot3 with keyboard teleoperation.
- **Real-Time Object Detection** — OpenCV HSV filtering to detect and track the largest red object in the camera feed.
- **Autonomous Following** — Robot autonomously aligns and follows the detected object smoothly.
- **Semi-Autonomous Mode Switching** — Manual control switches automatically to autonomous mode upon detection.
- **Custom Gazebo World** — Simulated house environment.
- **Visualization and Debugging** — Supports RQt for monitoring nodes/topics, RViz2 for sensor visualization.
- **Cross-Platform Compatibility** — Tested on Ubuntu 22.04 with WSL2 + WSLg, no additional X server needed.

---

## ⚙️ Prerequisites

- Ubuntu 22.04 LTS (WSL2 + WSLg recommended for Windows users)
- ROS 2 Humble Hawksbill
- TurtleBot3 robot packages (`turtlebot3`, `turtlebot3_simulations`)
- Python OpenCV (`opencv-python`) and `cv_bridge`
- Colcon build tool for ROS 2 package compilation

---

## 🛠️ Installation and Setup

```bash
# Clone the repository
git clone https://github.com/Sara-Esm/turtlebot3_color_follower.git
cd turtlebot3_color_follower

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the setup script
source install/setup.bash
```


---


## Usage

Launch the simulation environment and color follower node:

```bash
# Set TurtleBot3 model environment variable
export TURTLEBOT3_MODEL=waffle_pi

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Launch Gazebo house world with TurtleBot3
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

# In a new terminal, launch the color detector node
ros2 launch turtlebot3_color_follower color_detector.launch.py
```

## How it works:
1. Teleoperation: Use your keyboard to manually drive the TurtleBot3 around the Gazebo house environment.
2. Detection: The node processes camera frames, applying HSV filtering to detect red objects.
3. Autonomous Follow: Upon detecting the red object, the robot autonomously follows it by adjusting its velocity commands.
4. Mode Switching: When no red object is present, control returns to manual teleoperation.


---

## 🔧 Commands & Debugging Tools

These commands help you monitor and debug your project while running:

```bash
# View raw camera feed
rqt_image_view /camera/image_raw

# Visualize ROS 2 node graph (nodes and topics)
ros2 run rqt_graph rqt_graph

# RViz2 for sensor and robot visualization
ros2 launch turtlebot3_bringup rviz2.launch.py

# Monitor camera topic publishing rate
ros2 topic hz /camera/image_raw

# Echo velocity commands being published
ros2 topic echo /cmd_vel

# Dynamically adjust linear speed parameter of the color detector node
ros2 param set /color_detector_node linear_speed 0.2
```

---


## 📂 Project Structure
```bash
turtlebot3_color_detector/
├── src/
│   └── turtlebot3_color_follower/
│       ├── launch/
│       │   └── color_detector.launch.py      # Launch file for starting the node
│       ├── turtlebot3_color_follower/
│       │   ├── __init__.py
│       │   └── color_detector.py              # Main detection and control node
│       ├── package.xml                        # ROS2 package manifest
│       ├── setup.py                           # Python package setup script
│       └── setup.cfg                          # Python setup configuration
├── .gitignore
├── README.md
└── LICENSE
```

---


## 🎓 Learning Outcomes & Skills Demonstrated
- Integration of ROS 2 and OpenCV for real-time robotic perception.
- HSV color space filtering for robust object detection and tracking.
- ROS2 publisher/subscriber model and parameter management.
- Writing clean, modular Python ROS 2 nodes following best practices.
- Building and launching ROS 2 packages using Colcon.
- Practical robotics control via velocity command publishing.
- Proficient use of RQt and RViz2 for runtime debugging and visualization.
- Simulation experience with Gazebo and cross-platform development on WSL2 + WSLg.


---


## 🔗 Useful Links

- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [TurtleBot3 Official](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- [Gazebo Simulator](https://gazebosim.org/)
- [OpenCV](https://opencv.org/)

---


## 📜 License
This project is licensed under the MIT License.
