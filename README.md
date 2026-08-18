# EGH446 Autonomous Systems — Major Project

Group project for **EGH446: Autonomous Systems** (Semester 2, 2026), Queensland University of Technology.

> Fill in the italicised placeholders below with your group's actual details before submitting/pushing.

## Overview

This repository contains our group's implementation of the guidance, control, and estimation subsystems for an autonomous *[ground robot / quadrotor]* — see [Section: Vehicle](#vehicle) — built in MATLAB/Simulink using the simulation environment provided for the course.

The goal is to navigate the vehicle through an optimally ordered sequence of 10 randomly generated waypoints (range `x, y ∈ [-100, 100] m`), while compensating for noisy position and heading measurements (`σ²ₓ = σ²ᵧ = 0.1 m²`, `σ²θ = 2 deg²`).

**Part A** (30% — 10% individual, 20% group) covers:
- Waypoint generation & optimisation (shortest path ordering)
- Guidance & control (heading-to-waypoint tracking)
- State estimation / filtering (optimal filtering to obtain vehicle states)

**Part B** (due Week 12) adds:
- Path planning to a goal location
- Collision avoidance (static obstacles / walls)

### Assessment format

- Report and code, assessed both individually and as a group.
- **Individual (10%):** one PDF report per student, focused on your own contribution.
- **Group (20%):** one zipped submission per group containing all MATLAB/Simulink code and data (report is **not** included in this zip — submitted separately).
- **Peer & self-evaluation:** each student rates themselves and every teammate against four teamwork criteria; scores are averaged across peers. Not submitting a peer evaluation results in zero for that component.

## Team

| Name | Student # | Subsystem(s) owned |
|---|---|---|
| *Name 1* | *n#######* | *e.g. Guidance & Control* |
| *Name 2* | *n#######* | *e.g. State Estimation* |
| *Name 3* | *n#######* | *e.g. Waypoint Optimisation* |

## Vehicle

*State which vehicle type your group chose and why.*

- Vehicle type: *ground robot / quadrotor*
- State variables used: *x, y, θ (ground) or x, y, ψ (quad)*

## Repository Structure

*Update this to match your actual folder/file layout.*

```
.
├── models/                 # Simulink .slx model(s)
│   └── main_sim.slx
├── functions/               # MATLAB function blocks / helper scripts
│   ├── waypoint_generation.m
│   ├── waypoint_optimisation.m
│   ├── guidance_control.m
│   └── state_estimation.m
├── data/                    # Saved waypoint sets, logged results
├── results/                 # Plots, RMS cross-track error, mission time logs
├── report/                  # PDF report(s)
└── README.md
```

## Subsystems

### 1. Waypoint Generation & Optimisation
Generates 10 random waypoints within the boundary and orders them to minimise path length (shortest path / TSP-style optimisation).

- **Inputs:** *e.g. number of waypoints, boundary limits*
- **Outputs:** *ordered waypoint list `{x, y}`*
- **Approach used:** *describe your optimisation method, e.g. nearest-neighbour, 2-opt, brute force*

### 2. Guidance & Control
Drives the vehicle toward the current target waypoint using a heading-error control law, switching to the next waypoint once within capture radius.

- **Controller type:** *e.g. PID on heading rate*
- **Guidance logic:** *heading-to-waypoint, line-of-sight, etc.*
- **Key parameters:** *gains, waypoint capture radius*

### 3. State Estimation / Filtering
Filters noisy position and heading measurements to produce a clean vehicle state estimate.

- **Filter type:** *e.g. Kalman filter, complementary filter*
- **Performance metric(s):** *e.g. estimation error vs. ground truth*

### 4. (Part B) Path Planning & Collision Avoidance
*To be completed in Part B.*

## Performance Metrics

As required by the assignment, we report:

- **Total mission time** — time to visit all waypoints
- **Average waypoint capture error**
- **RMS cross-track error**
- Trade-off analysis between waypoint accuracy and mission duration

Results and plots are stored in `results/`.

## Getting Started

### Prerequisites

- MATLAB Release **2025b**
- Simulink
- *Any additional toolboxes, e.g. Control System Toolbox, Robotics System Toolbox*
- Course simulation environment package: `egh446_simEnv2026_25b.zip` (provided via Canvas — extract into this repo, or reference its location if kept outside)

### Running the Simulation

1. Clone this repository.
2. Extract/set up the provided simulation environment (`egh446_simEnv2026_25b.zip`) so it's on the MATLAB path.
3. Open MATLAB and set the repository root as your working directory (or add it to the path).
4. Open `models/main_sim.slx` in Simulink.
5. Run `functions/waypoint_generation.m` (or the relevant script) to populate the `waypoints` struct in the workspace.
6. Run the Simulink model.
7. The simulation will stop automatically once the final waypoint is captured.

```matlab
% Example: generate and load waypoints into the workspace
waypoints.x = randi([-100 100], 10, 1);
waypoints.y = randi([-100 100], 10, 1);
```

## Report

The full report addressing the Part A/B requirements (subsystem block diagrams, architecture diagram, design justification, and performance results) is in `report/`.

## Submission Checklist (Part A)

- [ ] Individual PDF report — one per student, own contribution — via *EGH446 Group Assignment: Submissions Part A (Individual Component)*
- [ ] Group code zip — all MATLAB/Simulink code + data, **no report inside** — via *EGH446 Group Assignment: Submissions Part A (Group Component)*
- [ ] Peer & Self-Evaluation — completed individually by each student

## Notes

- Do **not** commit code screenshots into the report per the assignment rules.
- See the [assignment brief](EGH446MajorProject_1.pdf), Part A FAQs, and Canvas for full requirements and rubrics.
- Report any errors in the assignment spec to the unit coordinator (see brief).

## License

*Course project — for academic submission purposes (QUT EGH446). Not for external distribution unless otherwise agreed by the group.*
