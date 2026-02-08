# av_gazebo_bringup

Gazebo simulation bringup package for the Pedestrian Navigation AI System. This README summarizes what each file and folder does and links to the more detailed project documentation.

See the full package documentation: [doc/av_gazebo_bringup/README.md](../../doc/av_gazebo_bringup/README.md)

## Overview

This package provides a reproducible Gazebo environment with an ego vehicle, pedestrians, and helper scripts to spawn and control agents. It is intended as the simulation base for perception, prediction, and control experiments.

## Directory structure

```
av_gazebo_bringup/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── av_world.launch.py
│   └── system_integration.launch.py
├── worlds/
│   ├── av_test.world
│   └── av_test_old.world
├── urdf/
│   ├── ego_vehicle.urdf.xacro
│   └── pedestrian.xacro
├── scripts/
│   ├── spawn_ego.py
│   ├── ped_randomizer.py
│   ├── movevehical.py
│   ├── driver.py
│   ├── advanced_evaluator.py
│   └── benchmark_results.csv
├── materials/
│   ├── scripts/av.material
│   └── textures/
└── model_training/
    ├── download_data.py
    ├── lstm_dataformator.py
    ├── train_lstm.py
    ├── ped_lstm_model.pth
    └── processed_pedestrian_data.npz
```

## File roles (quick reference)

- `launch/av_world.launch.py` — Start Gazebo with the scene only (no actors).
- `launch/system_integration.launch.py` — Full simulation: world + spawners + control and evaluation scripts.
- `worlds/*.world` — Scene definitions used by Gazebo.
- `urdf/*.xacro` — Vehicle and pedestrian models.
- `scripts/spawn_ego.py` — Spawns the ego vehicle into Gazebo.
- `scripts/ped_randomizer.py` — Randomizes pedestrian spawn positions for domain randomization.
- `scripts/movevehical.py` — Low-level vehicle command interface used by higher-level controllers.
- `scripts/driver.py` — Higher-level driving logic or demo controller.
- `scripts/advanced_evaluator.py` — Logging and metrics for experiments.

## Runtime flow diagram

Below is a high-level flow diagram showing how components interact during a `system_integration.launch.py` run.

```mermaid
flowchart LR
  subgraph Launch
    L[system_integration.launch.py]
  end

  L --> G[Gazebo World (av_test.world)]
  L --> SE[Spawn Ego (`spawn_ego.py`)]
  L --> SP[Spawn Pedestrians (`ped_randomizer.py`)]
  L --> CTRL[Driver / Control scripts]
  L --> EVAL[advanced_evaluator.py]

  SE --> Ego[Ego Vehicle (URDF)]
  SP --> Peds[Pedestrian Models]
  G -->|physics & sensors| Ego
  G -->|physics & collisions| Peds

  CTRL -->|cmd_vel / control| Ego
  Ego -->|state / odom| CTRL
  Peds -->|state| EVAL
  Ego -->|pose / collisions| EVAL
  EVAL -->|metrics.csv| results[(results/ folder)]

  classDef nodes fill:#f9f,stroke:#333,stroke-width:1px;
  class L,SE,SP,CTRL,EVAL nodes;
```

## Typical workflows

- Start only the world (for manual spawning or testing):

```bash
ros2 launch av_gazebo_bringup av_world.launch.py
```

- Full integration (world + actors + controllers + evaluator):

```bash
ros2 launch av_gazebo_bringup system_integration.launch.py
```

- Spawn or respawn an ego vehicle manually at a different pose:

```bash
python3 scripts/spawn_ego.py --x 1.0 --y 0.5 --yaw 0.0
```

## Notes and extensions

- Add sensor plugins (camera, lidar) to `urdf/ego_vehicle.urdf.xacro` to enable perception nodes.
- Use `ped_randomizer.py` to run domain randomized experiments for robust model evaluation.
- `model_training/` contains research scripts for LSTM-based pedestrian prediction; these are optional for simulation runs.

---
Maintainer: DvaitAI

## Top-level files

- `CMakeLists.txt`: build instructions for the ROS2 package (install targets, scripts, and resource files).
- `package.xml`: ROS2 package metadata (dependencies, maintainers, version).

## Folders and their workings

- `launch/`
  - `av_world.launch.py` — Launches the Gazebo world only (loads `av_test.world`, physics, and basic plugins).
  - `system_integration.launch.py` — Full-system launch: starts the world, spawns the ego vehicle, spawns pedestrians, and launches integration scripts for control and evaluation.

- `worlds/`
  - `av_test.world` — Main Gazebo world file containing terrain, roads, and static scene elements used during testing.
  - `av_test_old.world` — Legacy version of the world (kept for reference).

- `urdf/`
  - `ego_vehicle.urdf.xacro` — XACRO model describing the ego vehicle geometry, collision, links, joints, and sensor mount points. Used by spawn scripts to instantiate the vehicle in Gazebo.
  - `pedestrian.xacro` — Pedestrian model (visual/collision) used for spawning pedestrian agents.

- `scripts/`
  - `spawn_ego.py` — Spawns the ego vehicle into Gazebo using the ego URDF/XACRO. Typically called by the launch file or run manually to add the vehicle at a specified pose.
  - `ped_randomizer.py` — Generates randomized pedestrian spawn positions and orientations; useful for domain/randomized testing. Can be used to respawn or vary pedestrian distribution between runs.
  - `movevehical.py` *(note: filename retained as in repo)* — Low-level script that applies motion commands to the ego vehicle in simulation (velocity/steering commands). Used by higher-level drivers or test harnesses.
  - `driver.py` — Basic driving logic for the ego vehicle. Reads commanded trajectories or waypoints and issues control commands (often a higher-level wrapper that uses `movevehical.py`).
  - `advanced_evaluator.py` — Evaluation utilities and metrics collection (collision detection, trajectory error, benchmark logging). Outputs CSV or other summary metrics used by the `results/` folder.
  - `benchmark_results.csv` — Example or output file collected by evaluators (kept in the package for convenience).
  - `spawn_ego.py`, `ped_randomizer.py` and other scripts can be launched individually for targeted experiments.

- `materials/`
  - `scripts/av.material` and `textures/` — Custom Gazebo materials and textures (asphalt, grass) referenced by world and models to improve visual realism.

- `model_training/`
  - `download_data.py` — Utilities to fetch training data used for model experiments.
  - `lstm_dataformator.py` — Data formatting helper to prepare training sequences for LSTM training.
  - `train_lstm.py` — Example training script for a pedestrian LSTM prediction model (not required to run the simulator).
  - `ped_lstm_model.pth`, `processed_pedestrian_data.npz` — Example artifacts from experiments.

## Usage

Build and source the workspace, then launch the integration file for a full simulation:

```bash
colcon build
source install/setup.bash
ros2 launch av_gazebo_bringup system_integration.launch.py
```

- Use `av_world.launch.py` to start the world without spawning actors.
- Run `spawn_ego.py` manually to spawn the vehicle in different poses.
- Use `ped_randomizer.py` to respawn or randomize pedestrian positions between runs.

## Notes & Tips

- Camera/LiDAR sensor plugins can be added to `ego_vehicle.urdf.xacro` for perception experiments.
- If you modify URDF/XACRO files, restart Gazebo or respawn models to pick up changes.
- Use `advanced_evaluator.py` to produce consistent benchmarking CSV output for the `results/` folder.

## Where to look next

- More detailed instructions, examples, and background are in the package documentation: [doc/av_gazebo_bringup/README.md](../../doc/av_gazebo_bringup/README.md)

---
Maintainer: DvaitAI
