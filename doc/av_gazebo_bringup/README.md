# 🚗 av_gazebo_bringup

Gazebo simulation bringup package for the **Pedestrian Navigation AI System**.

This package launches the complete simulation environment including:
- 🌍 Gazebo world
- 🚙 Ego vehicle
- 🚶 Pedestrians
- 🛣️ Road textures
- 🎯 System integration nodes

It acts as the **entry point** for testing perception, prediction, and navigation.

---

# 🎯 Purpose

Before adding AI or perception,
we need a realistic **simulation environment**.

This package provides:

✔ Physics  
✔ Vehicle spawning  
✔ Pedestrian spawning  
✔ Road/world setup  
✔ Sensor-ready environment  

Without this → no testing is possible.

---

# 📁 Folder Structure

```
av_gazebo_bringup/
│
├── launch/
│ ├── av_world.launch.py
│ ├── system_integration.launch.py
│
├── worlds/
│ ├── av_test.world
│ ├── av_test_old.world
│
├── urdf/
│ ├── ego_vehicle.urdf.xacro
│ ├── pedestrian.xacro
│
├── scripts/
│ ├── spawn_ego.py
│ ├── ped_randomizer.py
│ ├── movevehicle.py
│ ├── driver.py
│ ├── advanced_evaluator.py
│
├── materials/
│ ├── scripts/
│ ├── textures/
│
├── model_training/
│ ├── train_lstm.py
│ ├── download_data.py
│
├── package.xml
├── CMakeLists.txt
```

---

# 🧩 Components Explained

## 🔹 launch/
Launch files to start simulation

### av_world.launch.py
Starts:
- Gazebo world
- physics engine
- basic environment

### system_integration.launch.py
Starts:
- world
- ego vehicle
- pedestrians
- control scripts

👉 Use this for full testing

---

## 🔹 worlds/
Gazebo world definitions

### av_test.world
Main environment with:
- road
- ground
- obstacles

---

## 🔹 urdf/
Robot models

### ego_vehicle.urdf.xacro
Defines:
- vehicle body
- wheels
- collision
- sensor mounts

### pedestrian.xacro
Defines:
- pedestrian agent model

---

## 🔹 scripts/

### spawn_ego.py
Spawns ego vehicle into Gazebo

### ped_randomizer.py
Randomly generates pedestrian positions

### movevehicle.py
Controls ego vehicle motion

### driver.py
Basic driving logic

### advanced_evaluator.py
Evaluation + metrics

---

## 🔹 materials/
Textures for realistic environment
- asphalt
- grass

Improves simulation realism

---

## 🔹 model_training/
Early prediction experiments
(LSTM-based trajectory prediction)

Not required for simulation start.

---

# ▶️ How To Run

## 1️⃣ Build workspace
```
colcon build
source install/setup.bash
```

## 2️⃣ Start simulation
```
ros2 launch av_gazebo_bringup system_integration.launch.py
```

Gazebo will open with:
✔ vehicle
✔ pedestrians
✔ world

🧪 Typical Workflow

Launch Gazebo

Spawn vehicle

Add perception nodes

Add prediction

Add planner

Evaluate

This package = Step 1

### 🧠 Design Philosophy

Simulation first.
Intelligence later.

A stable environment makes AI debugging easier.

### 📌 Future Improvements

* Sensor plugins (camera/lidar)

* Better pedestrian behavior

* Traffic rules

* Weather simulation

* Domain randomization

## 👨‍💻 Maintainer

DvaitAI
Where Intelligence Meets Duality