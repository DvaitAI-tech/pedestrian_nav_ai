# 🧠 DvaitAI – Day 8 Documentation

## 📅 Overview
**Date:** November 9, 2025  
**Phase:** M.Tech Project – *Pedestrian-Aware Autonomous Navigation (DvaitAI Phase 2)*  
**Focus:** Project Setup + Dataset Preparation + YOLOv5 Integration  
**Status:** ✅ All core goals completed

---

## 🚀 Completed Tasks

| # | Task | Description | Output | Duration | Status |
|---|------|--------------|---------|-----------|--------|
| **1️⃣** | **Project Setup & Architecture** | Created new ROS2 workspace `ros2_ws_nav` and initialized package `pedestrian_nav_ai`. | ROS2 package + `setup.py`, `package.xml`, and node structure ready. | 1 hr | ✅ |
| **2️⃣** | **Dataset & Framework Prep** | Set up datasets PIE, KITTI, and nuScenes. Configured PyTorch, OpenCV, and YOLOv5 environments. | Dataset pipeline verified and tested. | 2 hrs | ✅ |
| **3️⃣** | **YOLOv5 Pedestrian Detection Node** | Integrated pre-trained YOLOv5 model into ROS2 node that publishes detection results. | `/bounding_boxes` topic publishing successfully with live inference. | 2 hrs | ✅ |
| **4️⃣** | **Documentation & GitHub Update** | Created documentation, updated repo README, and committed setup scripts. | Synced with GitHub under `pedestrian_nav_ai` repo. | 1 hr | ✅ |

---

## ⚙️ Technical Setup Process

### 🧩 ROS2 Workspace Setup
```bash
# Create a new ROS2 workspace
mkdir -p ~/Music/DvaitAI/ros2_ws/src
cd ~/Music/DvaitAI/ros2_ws

# Initialize pedestrian navigation package
cd src
ros2 pkg create --build-type ament_python pedestrian_nav_ai
cd ..

# Build workspace
colcon build --symlink-install
source install/setup.bash
```

### 📦 Dependencies Installed
```bash
sudo apt install ros-humble-rviz2 ros-humble-cv-bridge python3-colcon-common-extensions
pip install torch torchvision opencv-python ultralytics pandas matplotlib
```

### 🧠 YOLOv5 Integration
```bash
# Clone YOLOv5 into the project folder
cd ~/ros2_ws_nav/src/pedestrian_nav_ai
git clone https://github.com/ultralytics/yolov5.git

# Load pretrained model in ROS2 node (Python)
from ultralytics import YOLO

model = YOLO('yolov5s.pt')
results = model(source='0')  # webcam or video stream
```
The model publishes detection results to ROS2 topic `/bounding_boxes` as `std_msgs/String` JSON-encoded messages.

## 📁 Repository Structure
```
pedestrian_nav_ai/
├── pedestrian_nav_ai/
│   ├── __init__.py
│   ├── detection_node.py      # YOLOv5 + ROS2 integration
│   ├── utils/
│   │   └── converter.py       # KITTI/PIE dataset format converter
│   └── launch/
│       └── detection.launch.py
├── package.xml
├── setup.py
└── README.md
```

---

## 🧠 Learnings & Observations
- YOLOv5 performs well for pedestrian detection even on limited compute.  
- ROS2 + PyTorch integration requires multi-threaded callbacks to avoid message lag.  
- Dataset preprocessing pipelines differ between KITTI and PIE — custom converters were built.  

---

## 📊 Results Summary
- ✅ Live pedestrian detection using YOLOv5 integrated with ROS2 topics.  
- ✅ Dataset preprocessing and environment setup completed.  
- ✅ Visualization pipeline validated using RViz2.  

---

## 🔗 Next Steps (Day 9)
1️⃣ Add real-time bounding box visualization overlay on live video stream.  
2️⃣ Begin trajectory prediction module (LSTM/Kalman filter).  
3️⃣ Start drafting M.Tech report section: *“Pedestrian Detection & Dataset Integration.”*  

---

## 💪 Motivation of the Day
> “Clarity comes from structure. Every setup you complete today becomes the skeleton of tomorrow’s intelligence.”

---

## 📜 Summary
Day 8 marks the successful initiation of the M.Tech AI Navigation Project.  
ROS2 workspace, dataset preprocessing, and YOLOv5 integration have been completed.  
The system can now detect pedestrians live and publish detection data for further trajectory prediction.

---
