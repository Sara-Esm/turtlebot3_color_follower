# TurtleBot3 Color Follower


This ROS 2 package empowers the TurtleBot3 robot to detect and follow objects of a specific color using computer vision techniques. It processes camera feed frames in real-time and controls the robot’s motion accordingly, combining manual teleoperation with autonomous behaviour for a seamless user experience.


---


## 🎯 Key Features
- **Manual Navigation Mode** — Control the TurtleBot3 with your keyboard.
- **Real-Time Object Detection** — Utilizes OpenCV to detect the largest red object in the camera feed with high accuracy.
- **Autonomous Following** — Once detected, the robot autonomously aligns and follows the target object smoothly.
- **Semi-Autonomous Switching** — Teleoperation remains active until the red object is detected, enabling dynamic mode switching.
- **Custom Gazebo World** — Includes a simulated house environment for realistic and safe navigation testing.
- **ROS 2 Humble + WSLg Compatible** — Fully tested on Ubuntu 22.04 within WSL2 using WSLg, eliminating the need for additional X servers.


---


## ⚙️ Prerequisites

- Ubuntu 22.04 LTS  
- ROS 2 Humble Hawksbill  
- TurtleBot3 robot packages  
- OpenCV for Python (`opencv-python`) and `cv_bridge`  
- Colcon build tool for building ROS 2 packages  


---


## 🛠️ Installation and Setup

```bash
# Clone this repository
git clone https://github.com/Sara-Esm/turtlebot3_color_follower.git
cd turtlebot3_color_follower

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the workspace setup script
source install/setup.bash
```


---


## Usage
Launch the color detector node using the provided launch file:
```bash
ros2 launch turtlebot3_color_follower color_detector.launch.py
```
## How it works:
1. Teleoperation: Use your keyboard to manually drive the TurtleBot3 around the Gazebo house environment.
2. Detection: The node processes camera frames, applying HSV filtering to detect red objects.
3. Autonomous Follow: Upon detecting the red object, the robot autonomously follows it by adjusting its velocity commands.
4. Mode Switching: When no red object is present, control returns to manual teleoperation.


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
│       ├── package.xml                        # ROS 2 package manifest
│       ├── setup.py                           # Python package setup script
│       └── setup.cfg                          # Python setup configuration
├── .gitignore
├── README.md
└── LICENSE
```

---


## 🎓 Learning Outcomes & Skills Demonstrated
- Seamless integration of ROS 2 and OpenCV for real-time computer vision on a robot
- HSV color space filtering for robust object detection
- Effective publishing and subscribing to ROS 2 topics and dynamic parameter handling
- Writing modular, maintainable Python ROS 2 nodes following best practices
- Building and launching ROS 2 packages using the Colcon build system
- Practical application of robotics control by generating velocity commands based on visual input
- Experience with simulation environments (Gazebo) and cross-platform setups (WSLg)

---


## 📜 License
This project is licensed under the MIT License.
