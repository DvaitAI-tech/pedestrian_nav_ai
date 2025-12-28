
# 🧠 PEDESTRIAN_NAV_AI
### Pedestrian-Aware Autonomous Navigation using ROS2 & Gazebo

> From sensing → understanding → risk estimation → safe decision making.

---

## 📘 Project Overview

This repository implements a **Pedestrian-Aware Safety System** for autonomous vehicles using **ROS 2 Humble**, **Gazebo**, **LiDAR**, **Camera**, and **deep-learning-based perception**.

The project has evolved beyond basic object detection into a **world-aware safety framework** capable of:
- Simulating an autonomous vehicle
- Perceiving pedestrians using sensors
- Tracking pedestrian motion
- Estimating collision risk (TTC)
- Visualizing and explaining safety-critical situations

This work forms the **core perception + safety layer** of an autonomous navigation stack.

---

## 🏗️ System Architecture

```
Gazebo Simulation
   ├── Ego Vehicle (URDF)
   │    ├── LiDAR
   │    └── Camera
   │
   ├── Sensor Topics
   │    ├── /ego/lidar/scan
   │    └── /camera/image_raw
   │
   ├── Perception & Fusion
   │    └── Pedestrian world positions (PoseArray)
   │
   ├── Tracking & Risk Estimation
   │    ├── Velocity estimation
   │    └── Time-To-Collision (TTC)
   │
   └── Visualization (RViz + Matplotlib)
```

---

## 🧱 Directory Structure

```
PEDESTRIAN_NAV_AI/
├── build/
├── install/
├── log/
├── resource/
├── doc/
├── src/
│   ├── av_gazebo_bringup/
│   ├── av_sensor_package/
│   └── cmds/
├── test/
├── yolov8n.pt
└── README.md
```

---

## ✅ Completed Features

- Gazebo simulation with road, pedestrians, and ego vehicle
- LiDAR & Camera sensor integration
- Stable TF tree and RViz visualization
- World-frame pedestrian perception
- Tracking with stable IDs and velocity estimation
- Collision-aware TTC calculation
- Explainable top-down visualization

---

## 🔴 Pending Work

### 🚦 Safety Decision-Making Layer

The system currently **detects and evaluates risk**, but does not yet **control vehicle behavior**.

Next step:
- Implement a `/safety/decision` node
- Output decisions: `STOP`, `SLOW_DOWN`, `GO`
- Based on TTC and risk persistence

---

## 🧭 Roadmap

| Phase | Description | Status |
|-----|------------|--------|
| Phase 1 | Gazebo + ROS2 setup | ✅ Done |
| Phase 2 | Sensor integration | ✅ Done |
| Phase 3 | Tracking & TTC risk | ✅ Done |
| Phase 4 | Safety decision logic | 🔜 Next |
| Phase 5 | Planning & control | 🧩 Planned |

---

## 🎓 Academic Value

- M.Tech Final Project
- Autonomous Driving Research
- ROS2 / AV Portfolio

---

## 📜 License

MIT License (Academic & Research Use)