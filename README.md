# 🐢 dubosim — A Custom Turtlesim Featuring BRACU Duburi Bot 4.2

### 🧠 Overview
**dubosim** is a customized version of the classic ROS2 *Turtlesim* package.  
It replaces the original turtle with the **BRACU Duburi Bot 4.2** and features a **pool-themed background**, offering a fun and BRACU Duburi-inspired way to visualize robot motion and path control in ROS2.

> All command-line instructions, topics, and parameters are kept identical to the original **Turtlesim** for simplicity.

---

### ⚙️ Features
- 🐢 Replaces Turtlesim’s turtle with **BRACU Duburi Bot 4.2**
- 🌊 Added **pool-style background**
- 💬 Maintains all original Turtlesim commands (except the running command — now use ros2 run dubosim dubosim_node)
- 🚀 Fully compatible with **ROS2**
- 🎯 Ideal for beginner-friendly simulation and BRACU Duburi demos

---

### 🖼️ Preview
<img width="493" height="494" alt="dubosim" src="https://github.com/user-attachments/assets/b70ffd63-10b0-4f2f-aca4-6c2394048ba2" />

---

### 📦 Installation

Clone and build the package in your ROS2 workspace:
```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/WasikZubair/dubosim.git

# Build the package
cd ~/ros2_ws
colcon build

# Source the setup
source install/setup.bash

```
# 🐢 Start the dubosim simulator
ros2 run dubosim dubosim_node

# 🎮 Control the Duburi bot (same teleop as Turtlesim)
ros2 run turtlesim turtle_teleop_key

Inspired by ROS’s Turtlesim, dubosim brings the underwater robotics spirit of BRACU Duburi into the ROS2 ecosystem — combining simplicity, creativity, and academic pride.










👨‍💻 Developer
Mohammad Wasik Zubair Abrar
(BRACU Duburi Robotics Team)


If you like this project, consider giving it a ⭐
