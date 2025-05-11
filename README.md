# 🚁 PYMAVLINK and ROS2 Interface for Drone Control

This project showcases my work integrating MAVLink with ROS2 to control a drone in a simulated environment using SITL (Software-In-The-Loop). It demonstrates how to interact with a drone using two custom ROS2 nodes and the pymavlink library.

## 🔧 Task Description

I developed two ROS2 nodes from scratch:

1. **`battery_gps_node.py`**  
   📡 Publishes the current battery percentage on the `battery_status` topic.  
   🌍 Subscribes to the `target_gps` topic to receive GPS coordinates and send them to the drone.

2. **`mission_control_node.py`**  
   🧭 Accepts a GPS coordinate as a command-line argument.  
   🚀 Publishes the coordinate to the `target_gps` topic.  
   🛬 Monitors battery status and commands the drone to land if the battery drops below 20%.

## ✅ Requirements

- ROS2 Humble (or newer)
- `pymavlink` and `mavlink`
- A drone simulator (e.g., ArduPilot SITL)

## ▶️ How to Run

1. Set up SITL simulation with ArduPilot.
2. Launch `battery_gps_node.py`.
3. Run `mission_control_node.py` with a target GPS position.
4. Watch the drone move to the target and land when battery is low.

## 🎥 Results

You can find a video demonstration of the full system inside this repository 👉 [Click here to find the video](./ros-framework.mp4) — feel free to check it out and see the project in action!
