# risk_assesment

Safety and benchmarking utilities for the Pedestrian Navigation AI System. This package contains tools that compute risk metrics, compare platform performance, and decide safe actions in simulated or recorded scenarios.

## Overview

The package implements real-time and offline risk assessment, visualization, and decision logic. It can run as standalone Python modules or be launched as ROS nodes depending on your workspace configuration.

## Directory structure

```
risk_assesment/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── risk_assesment
├── risk_assesment/
│   ├── __init__.py
   │   ├── bev_comparison.py
   │   ├── control_arbiter.py
   │   ├── fusion_visualizer.py
   │   ├── plat_comparison_graph.py
   │   ├── processed_pedestrian_data.npz
   │   ├── realtime_comparison_fixed.csv
   │   ├── safety_decision_node.py
   │   └── safety_decision_node_ema_lstm.py
└── test/
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

## File summary

- `risk_assesment/bev_comparison.py` — Utilities to compare bird's-eye-view (BEV) representations across different methods or runs (used to produce comparison plots and CSV summaries).
- `risk_assesment/control_arbiter.py` — Arbitration logic that selects a safe control command when multiple decision modules provide competing suggestions.
- `risk_assesment/fusion_visualizer.py` — Visual debugging helpers that overlay fused pedestrian data and risk scores for inspection (Matplotlib/RViz helpers).
- `risk_assesment/plat_comparison_graph.py` — Produces platform/performance comparison graphs (latency, risk metrics across methods).
- `risk_assesment/safety_decision_node.py` — Primary safety decision node: consumes tracked pedestrian states and risk signals, outputs alerts or safe-actions.
- `risk_assesment/safety_decision_node_ema_lstm.py` — Variant that combines EMA smoothing and an LSTM-based predictor for more robust short-term predictions.
- `processed_pedestrian_data.npz` — Example dataset used by offline analysis scripts.
- `realtime_comparison_fixed.csv` — Example results or reference CSV for benchmarking.

## How to run

Install in editable mode for development:

```bash
pip install -e src/risk_assesment
```

Run a module directly for quick checks (examples):

```bash
python -m risk_assesment.plat_comparison_graph
python -m risk_assesment.bev_comparison
```

In a ROS workspace, build and source then run the modules as nodes via your launch files (or adapt the modules to ROS node entrypoints).

## Runtime flow (high-level)

Data sources (camera/LiDAR/trackers or recorded datasets) feed into fusion and prediction modules. The safety decision node consumes fused tracks and risk estimates to output alerts or safe control actions. `control_arbiter.py` reconciles multiple action proposals. Visualizers and comparison scripts generate plots and CSVs for analysis.

See the module README for a detailed flow diagram and per-script usage: [risk_assesment/risk_assesment/README.md](risk_assesment/risk_assesment/README.md)

---
Maintainer: DvaitAI
