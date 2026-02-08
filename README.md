# 🧠 PEDESTRIAN_NAV_AI
### Pedestrian-Aware Autonomous Navigation using ROS2, Gazebo & Deep Learning

> From sensing → understanding → tracking → risk estimation → safe decision making.

---

## 📘 Project Overview

**PEDESTRIAN_NAV_AI** is a complete **end-to-end pedestrian safety framework** for autonomous vehicles built using:

- ROS 2 Humble
- Gazebo Simulation
- LiDAR + Camera fusion
- YOLOv8 perception
- BEV (Bird’s-Eye-View) world modeling
- Tracking + velocity smoothing
- Time-To-Collision (TTC) risk logic
- Stop-and-Go autonomous safety control

This system goes **beyond object detection** and implements a **world-aware, safety-critical decision layer** capable of:

✔ Detecting pedestrians  
✔ Tracking motion in world frame  
✔ Estimating collision risk  
✔ Making real-time braking decisions  
✔ Explaining decisions visually  

---

## 🎓 Academic Context

**M.Tech Final Dissertation – BITS Pilani (WILP)**  
**Title:** Pedestrian-Aware Autonomous Navigation: Integrating Deep Vision, Multi-Modal Fusion, and Uncertainty-Aware Planning  

This repository contains the **actual implementation used in the thesis experiments**, not a toy demo.

---

## 🚀 Final System Capabilities (Current Status)

### ✅ Fully Implemented

### Perception
- YOLOv8 pedestrian detection
- Custom optimized model (3.5ms inference)
- Camera → bounding boxes → tracked targets

### Sensor Fusion
- Geometric projection (baseline)
- **BEV Fusion (primary production pipeline)**
- World-frame Cartesian coordinates

### Tracking
- Nearest Neighbor association
- Stable ID persistence
- EMA velocity smoothing (α = 0.3)

### Risk Estimation
- Time-To-Collision (TTC)
- Lane filtering
- Collision radius modeling
- Multi-threshold safety margins

### Safety Decision Layer (COMPLETE)
- `/safety_decision_node`
- Outputs:
  - GO
  - SLOW
  - STOP
- Real-time braking logic validated at **35 m/s**

### Visualization
- RViz
- BEV radar view
- Health dashboard
- Live decision state monitor

### Evaluation
- ADE / FDE metrics
- Fusion comparison (Geometry vs BEV)
- EMA vs LSTM benchmarking
- CSV logging

### Reproducibility
- Docker support
- Full ROS2 workspace
- Modular packages
- Plug-and-play design

---

## 🏗️ Final Architecture

```
Gazebo Simulation
↓
Sensors (Camera + LiDAR)
↓
YOLOv8 Perception
↓
BEV Fusion (World Frame)
↓
Tracking + EMA smoothing
↓
TTC Risk Estimation
↓
Safety Decision Node
↓
GO / SLOW / STOP
```

---

## 📂 Workspace Structure

```
PEDESTRAN_NAV_AI/
├── src/
│ ├── av_gazebo_bringup/ # Simulation, worlds, launch files
│ ├── av_sensor_package/ # YOLO + fusion + tracking
│ └── risk_assesment/ # TTC + Stop-Go + evaluation
│
├── models/
├── docs/
├── Dockerfile
├── docker-compose.yml
└── results
```

---

## 📦 Package Responsibilities

### av_gazebo_bringup
- World files
- URDF ego vehicle
- Pedestrian spawners
- Scenario evaluator
- System launchers

### av_sensor_package
- YOLO node
- BEV fusion
- World projection
- Tracking
- Visualizers

### risk_assesment
- TTC computation
- EMA smoothing
- Safety decision logic
- Metrics + benchmarking

## 📚 Package & Module READMEs

For quick navigation to package-level and module-level documentation, see:

- [src/av_gazebo_bringup/README.md](src/av_gazebo_bringup/README.md)
- [src/av_sensor_package/README.md](src/av_sensor_package/README.md)
- [src/av_sensor_package/av_sensor_package/README.md](src/av_sensor_package/av_sensor_package/README.md)
- [src/risk_assesment/README.md](src/risk_assesment/README.md)
- [src/risk_assesment/risk_assesment/README.md](src/risk_assesment/risk_assesment/README.md)
- [doc/av_gazebo_bringup/README.md](doc/av_gazebo_bringup/README.md)
- [doc/ros_yolo_setup.md](doc/ros_yolo_setup.md)
---

## ⚙️ Build & Run

### Native

```bash
colcon build
source install/setup.bash
ros2 launch av_gazebo_bringup system_integration.launch.py
```
### Only world
```
ros2 launch av_gazebo_bringup av_world.launch.py
```
### Individual nodes
```
python -m av_sensor_package.yolo_node
python -m risk_assesment.safety_decision_node
```
### 🐳 Docker (Recommended)
```
Build:

docker build -t pedestrian_nav_ai .


Run:

docker-compose up --build

```
### Supports:

* GUI Gazebo

* GPU acceleration (optional)

* Reproducible environment



### 📊 Key Experimental Results
**Fusion Comparison**
| Metric              | Geometry     | BEV        |
| ------------------- | ------------ | ---------- |
| Stability           | Flicker      | Stable     |
| Occlusion handling  | Poor         | Excellent  |
| High-speed (35 m/s) | Delayed stop | Early stop |

**Tracking**
| Model | ADE   |
| ----- | ----- |
| EMA   | 0.7 m |
| LSTM  | 17+ m |

**➡ EMA chosen for safety-critical reliability**
| Model   | mAP      | Speed      |
| ------- | -------- | ---------- |
| YOLOv8n | 0.58     | 4.4 ms     |
| Custom  | **0.69** | **3.5 ms** |

### 🧠 Design Decisions
Why BEV Fusion?

      Better spatial reasoning + occlusion robustness

Why EMA over LSTM?

* Deterministic

* Stable

* No training instability

* Lower latency

* Safer for real-time braking

Why Modular ROS2?

* Simulator independent

* Easy hardware porting

* Debuggable safety nodes

🔮 Future Improvements (Optional Research)

* Transformer-based intent prediction

* Radar fusion

* Probabilistic planning

* Lateral avoidance (swerve)

* Factor-graph state estimation (GTSAM)

### 👨‍💻 Author
Nripender Kumar
M.Tech AI/ML – BITS Pilani
Autonomous Systems & Robotics

📜 License

MIT License – Academic & Research Use

