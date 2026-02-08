# End-to-End RL UAV Flight Controller

Welcome to the documentation for the **End-to-End Reinforcement Learning UAV Flight Controller** project.

## Project Goal

Develop a complete autonomous quadrotor flight controller powered by a neural network policy trained via reinforcement learning, progressing from simulation to custom hardware.

## Development Stages

| Stage | Phase | Description |
|-------|-------|-------------|
| SITL Simulation | Phase 2 | Gazebo-based simulation with PID baseline |
| RL Training | Phase 3 | Train RL policy to replace PID controller |
| Hardware-in-Loop | Phase 4 | Deploy on dev board with real-time inference |
| Custom Hardware | Phase 5 | Design and fabricate custom flight controller PCB |
| Integration | Phase 6 | System integration and flight testing |

## Quick Links

- [Architecture Overview](architecture/system-overview.md)
- [Requirements Database](requirements/index.md)
- [Work Breakdown Structure](WBS/00_WBS_Master_Overview.md)
- [Stage Gate Criteria](gates/index.md)
- [Contributing Guide](contributing.md)

## Tech Stack

| Component | Technology |
|-----------|-----------|
| Simulation | ROS 2 Jazzy + Gazebo Harmonic |
| RL Training | Stable-Baselines3 + Gymnasium |
| Firmware | C/C++ on ARM Cortex-M with FreeRTOS |
| Hardware | Custom PCB (KiCad) |
| CI/CD | GitHub Actions |
| Requirements | Doorstop |
| Documentation | MkDocs Material |
