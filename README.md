# TurtleBot3 Color Follower

This ROS2 package enables a TurtleBot3 robot to detect and follow objects of a specific color using computer vision.  
It processes camera feed frames in real-time and controls robot motion accordingly.

---

## Features
- Detects objects based on HSV color filtering.
- Publishes velocity commands for smooth tracking.
- ROS2 launch file for easy startup.
- Modular, extendable Python code.

---

## Requirements
- ROS 2 Humble (or compatible version)
- TurtleBot3
- OpenCV for Python
- Colcon build system

---

## Installation & Build

```bash
# Clone the repository
git clone https://github.com/Sara-Esm/turtlebot3_color_detector.git
cd turtlebot3_color_detector

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build
source install/setup.bash

# Launch the color follower node
ros2 launch turtlebot3_color_follower color_detector.launch.py

src/turtlebot3_color_follower/
├── launch/
│   └── color_detector.launch.py
├── turtlebot3_color_follower/
│   ├── __init__.py
│   └── color_detector.py
├── setup.py
├── setup.cfg
└── package.xml

License
This project is licensed under the MIT License.
