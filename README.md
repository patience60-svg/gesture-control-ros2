# Gesture Control ROS2 🤖👋

A ROS2 package for controlling robots using intuitive hand gestures. This project enables natural human-robot interaction by translating hand gestures into velocity commands for mobile robots.

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-brightgreen)
![Python](https://img.shields.io/badge/Python-3.8%2B-blue)
![OpenCV](https://img.shields.io/badge/OpenCV-4.5%2B-orange)
![MediaPipe](https://img.shields.io/badge/MediaPipe-Latest-yellow)

## 🎯 Features

- **Natural Gesture Control**: Use intuitive hand gestures to control robot movement
- **Real-time Processing**: Low-latency hand tracking with MediaPipe
- **ROS2 Integration**: Seamlessly integrates with ROS2 robotic systems
- **Multiple Gesture Support**: 
  - 👉 **POINTING** - Move Forward
  - 👍 **THUMBS UP** - Move Backward  
  - ✌️ **VICTORY** - Turn Left
  - 🤘 **ROCK SIGN** - Turn Right
  - 🖐️ **OPEN HAND** - Stop

## 🚀 Quick Start

### Prerequisites
- ROS2 Jazzy (or compatible distribution)
- Python 3.8+
- Webcam
- TurtleBot3 packages (for simulation)

### Installation

1. **Clone the repository**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/gesture-control-ros2.git
