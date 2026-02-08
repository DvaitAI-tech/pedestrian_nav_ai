# av_sensor_package (module)

This file documents the Python modules in `av_sensor_package/` and explains what each script does and how to run them.

## Files and purpose

- `__init__.py` — Package initializer.
- `yolo_node.py` — Runs YOLO-based object detection (camera image input). Produces detection messages (bounding boxes) for downstream nodes.
- `pedestrian_world_projection.py` — Utilities to project camera detections into world or bird's-eye-view (BEV) coordinates.
- `pedestrian_bev_fusion.py` — Fusion logic to combine detections (e.g., from YOLO) with BEV-projected information and possibly other sensors.
- `pedestrian_tracker_node.py` — Tracks pedestrians over time, assigns persistent IDs, and publishes tracked state (position, velocity, bounding box).
- `obstacle_node.py` — Node for handling static/dynamic obstacles and publishing obstacle lists for planning or visualization.
- `pedestrian_ttc_risk_world.py` — Computes Time-To-Contact (TTC) / risk metrics in world coordinates for tracked pedestrians.
- `pedestrian_ttc_risk_plot.py` — Visualization utilities for TTC/risk plotting (e.g., producing risk-time plots or heatmaps).
- `pedestrian_smoothing_prediction_plot.py` — Smoothing and prediction plot helpers for visualizing predicted pedestrian trajectories.
- `tracked_pedestrian_plot.py` — Plotting utilities to render tracked pedestrian histories and diagnostics.
- `tracked_pedestrian_viz.py` — Visualization node (or helper) that renders tracked pedestrians in RViz, Matplotlib or a web UI.
- `fusion_visualizer.py` — Visual helper combining detection, fusion, and tracking outputs into a single view for debugging.

## How to run

1. Ensure the package is installed (see parent README). 2. Run the detection node and then the tracker/fusion nodes. Example using direct module execution:

```bash
python -m av_sensor_package.yolo_node
python -m av_sensor_package.pedestrian_tracker_node
python -m av_sensor_package.pedestrian_bev_fusion
```

In a ROS workflow, these modules are typically launched by your ROS launch files so they run as nodes and exchange messages over topics. Configure topic names and parameters in your launch files or via parameter files.

## Config and parameters

- Each node usually accepts configuration via ROS parameters or environment variables (model paths, camera topics, thresholds). Check each node's top for default parameter names.

## Testing

- Unit/style tests are located in `test/` at the package root. Use the repository's test harness or `pytest` to run them.

## Notes

- This README is a high-level summary. For setup of YOLO and model files, consult [doc/ros_yolo_setup.md](../../doc/ros_yolo_setup.md).
