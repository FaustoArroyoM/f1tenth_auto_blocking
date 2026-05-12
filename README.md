# F1TENTH Autonomous Blocking via Trajectory Switching

A real-time defensive racing framework for the F1TENTH 1/10-scale platform. The car detects a trailing opponent with a camera + YOLOv11 model and switches between three pre-computed racelines (inner, middle, outer) to block overtaking attempts — all while maintaining near-optimal lap speed when no opponent is present.

📄 [Read the paper](media/F1tenth_final_report.pdf)

---

## Demo

https://github.com/user-attachments/assets/070a1aa9-03e6-427b-abc4-89ecd98df3d9

---

## System Architecture

Four layers communicate over ROS 2:

```text
┌─────────────────┐     ┌──────────────────────┐     ┌──────────────────────┐     ┌─────────────────┐
│  Localization   │     │     Perception        │     │    Tactical Manager  │     │    Controller   │
│  (Particle      │────▶│  YOLOv11 + ZED2       │────▶│  FSM raceline        │────▶│  Multi-Raceline │
│   Filter / MCL) │     │  distance estimator   │     │  selector (Python)   │     │  Pure Pursuit   │
│                 │     │                       │     │                      │     │  (C++)          │
└─────────────────┘     └──────────────────────┘     └──────────────────────┘     └─────────────────┘
         │                       │                              │                           │
    /ego_racecar/odom    /race_tracker/bbox_center    /active_raceline (0/1/2)         /drive
                         /race_tracker/distance_car
```

---

## Key Features

- **Multi-Raceline Pure Pursuit (C++)** — Tracks all three racelines in parallel at every odometry tick; lane switches are computationally instantaneous with a 0.15 s slew-rate ramp to prevent impulsive steering.
- **Hybrid Distance Estimation** — Stereo depth (ZED2) for ranges > 1 m, pinhole bounding-box formula for close range. Seamless handoff between the two.
- **Finite-State Tactical Manager** — Reacts to the opponent's normalised horizontal position in the camera frame. Biased center line (0.55 vs. 0.50) accounts for the track's persistent left-hand geometry. Debounce + timeout safeguards prevent oscillation and recover to the middle raceline when the opponent disappears.
- **Semi-Supervised YOLOv11 Training** — Initial model trained on a small hand-labelled dataset, then expanded with pseudo-labels from track-driving video. TensorRT `.engine` export for real-time inference on the Jetson Orin Nano.
- **MCL Localization** — Particle filter over a pre-built occupancy map using Hokuyo LiDAR, providing robust pose estimates at racing speeds.

---

## Hardware

| Component | Spec |
| --- | --- |
| Platform | F1TENTH (1/10 RC car) |
| Compute | NVIDIA Jetson Orin Nano |
| Camera | ZED2 RGB-D stereo |
| LiDAR | Hokuyo UST-10LX |
| Motor controller | VESC |

---

## ROS 2 Package Overview

| Package | Language | Role |
| --- | --- | --- |
| `pure_pursuit` | C++ | Multi-raceline Pure Pursuit controller |
| `car_distance_estimator` | Python | YOLOv11 inference + hybrid distance estimation |
| `auto_block_bringup` | Python | Launch files for the full system |
| `particle_filter` | Python | MCL localization (adapted from f1tenth upstream) |
| `waypoint_visualizer` | C++ | RViz waypoint debug overlay |

---

## Raceline Switching Logic

```text
Normalised bbox center  →  0 ──────────── 0.45 ── 0.55 ── 0.65 ──────────── 1
                                  ↑                  ↑                 ↑
                           Switch INNER          Keep MIDDLE      Switch OUTER
                           (block right)                          (block left)
```

Lane changes are gated by:

- **Proximity threshold**: opponent must be within 1.0 m
- **Debounce**: minimum 45 iterations (≈1.5 s at 30 Hz) between changes
- **Timeout**: auto-return to middle after 60 iterations without detection

---

## Quickstart

```bash
# Build
cd f1tenth_auto_blocking
colcon build --symlink-install
source install/setup.bash

# Run on physical car
ros2 launch auto_block_bringup bringup_block.py

# Pure pursuit only (simulation)
ros2 launch pure_pursuit sim_pure_pursuit_launch.py

# Tune parameters live (no rebuild needed)
ros2 param set pure_pursuit_multi_node middle.K_p 0.25
ros2 param set raceline_manager_node blocking_distance_threshold 1.5
```

---

## Results

In closed-track testing against a human-controlled opponent:

- The tactical manager correctly identified approaching vehicles and transitioned to a defensive raceline before the opponent reached a passing position in the majority of trials.
- Stable raceline tracking was maintained throughout switches, with no boundary violations in nominal conditions.
- Two failure modes identified: (1) camera occlusion during side-by-side passing, (2) transient particle filter divergence after aggressive multi-lane transitions at corner exit.

---

## Third-Party Code & Licenses

| Component | Source | License |
| --- | --- | --- |
| `Raceline-Optimization/` | [TUM — global trajectory optimisation](https://github.com/TUMFTM/global_racetrajectory_optimization) | LGPL-3.0 |
| `f1tenth_ws/src/f1tenth_system/` | [F1TENTH driver stack](https://github.com/f1tenth/f1tenth_system) | MIT |
| `lab_ws25/src/particle_filter/` | [F1TENTH particle filter](https://github.com/f1tenth/particle_filter) | MIT |

Our modifications to `Raceline-Optimization` consist of adjusted track boundary inputs (GIMP-edited occupancy maps) to bias the optimiser toward inner, middle, and outer corridors. The optimisation code itself is unchanged and remains under LGPL-3.0.

---

## References

1. R. C. Coulter, *Implementation of the Pure Pursuit Path Tracking Algorithm*, CMU RI, 1992.
2. S. Garlick & A. Bradley, *Real-Time Optimal Trajectory Planning for Autonomous Vehicles*, arXiv:2102.02315, 2021.
3. Ultralytics YOLOv11 — [docs.ultralytics.com](https://docs.ultralytics.com/)
4. Stereolabs ZED2 — [stereolabs.com](https://www.stereolabs.com/products/zed-2/)
