# av_sensor_package

Perception and fusion package for the Pedestrian Navigation AI System. This package contains Python nodes and utilities for object detection (YOLO), pedestrian tracking, BEV projection, fusion and visualization used in the perception pipeline.

See related documentation: [doc/ros_yolo_setup.md](../doc/ros_yolo_setup.md)

## Top-level files

- `package.xml` — ROS/packaging metadata and dependencies.
- `setup.py`, `setup.cfg` — Packaging and installation configuration for the Python package.
- `resource/av_sensor_package` — package resource marker used by ROS tooling.
- `test/` — repository tests for style and packaging.

## Python module folder

The main Python code lives in the `av_sensor_package/` directory. See `av_sensor_package/README.md` for details on the available modules and how to run them.

## Installation & usage

Install the package into your workspace (from project root):

```bash
pip install -e src/av_sensor_package
```

After installation you can run the modules directly (or run them as ROS nodes if wired up in your launch files). For example:

```bash
python -m av_sensor_package.yolo_node
python -m av_sensor_package.pedestrian_tracker_node
```

In a ROS-based setup, these modules are typically launched from your ROS launch files so they run as nodes and communicate over topics.

## Contact

Maintainer: DvaitAI
