# End-to-End Reinforcement Learning UAV Flight Controller

An end-to-end reinforcement learning agent for autonomous UAV flight control, progressing from software-in-the-loop (SITL) simulation through hardware-in-the-loop (HIL) testing to a custom flight controller board.

## Project Overview

This project develops a complete autonomous quadrotor flight controller powered by a neural network policy trained via reinforcement learning. The system is built through a rigorous 6-phase methodology:

| Phase | Description | Key Output |
|-------|-------------|------------|
| **1. Infrastructure** | Project setup, CI/CD, requirements management | Repo, pipelines, docs framework |
| **2. SITL Baseline** | Gazebo simulation with PID baseline controller | Validated simulation environment |
| **3. RL Control** | Train RL policy to replace PID in simulation | Converged neural network policy |
| **4. HIL Flight Control** | Deploy policy on dev board with hardware-in-loop | Real-time firmware + RL inference |
| **5. Custom Hardware** | Design & fabricate custom flight controller PCB | Flight-ready custom board |
| **6. Integration & Validation** | System integration, flight testing, documentation | Validated autonomous UAV system |

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Ground Station                        │
│              (Monitoring & Telemetry)                    │
└───────────────────────┬─────────────────────────────────┘
                        │ MAVLink
┌───────────────────────▼─────────────────────────────────┐
│                 Flight Controller                        │
│  ┌─────────────┐  ┌──────────────┐  ┌───────────────┐  │
│  │ Sensor       │  │ RL Policy    │  │ Motor Control │  │
│  │ Fusion       │──│ (Neural Net) │──│ (PWM Output)  │  │
│  │ (IMU, Baro,  │  │ SAC/PPO      │  │ ESC Interface │  │
│  │  GPS, OF)    │  │              │  │               │  │
│  └─────────────┘  └──────────────┘  └───────────────┘  │
│                 ARM Cortex-M + RTOS                      │
└─────────────────────────────────────────────────────────┘
```

## Tech Stack

| Component | Technology |
|-----------|-----------|
| Simulation | ROS 2 Jazzy + Gazebo Harmonic |
| RL Training | Stable-Baselines3 + Gymnasium |
| Firmware | C/C++ on ARM Cortex-M with RTOS |
| Hardware | Custom PCB (KiCad) |
| CI/CD | GitHub Actions (sim + firmware pipelines) |
| Requirements | Doorstop (FOSS traceability) |
| Documentation | MkDocs + Material theme |
| Python | 3.12 |

## Repository Structure

```
├── simulation/      # ROS 2 + Gazebo SITL environment
├── firmware/        # Embedded C/C++ flight controller
├── training/        # RL training pipeline (SB3)
├── hardware/        # PCB design files (KiCad)
├── tests/           # Integration & system-level tests
├── tools/           # Docker, scripts, CI helpers
├── docs/            # Documentation & WBS
│   ├── WBS/         # Work breakdown structure
│   ├── templates/   # Document templates
│   └── gates/       # Stage gate review criteria
└── .github/         # CI/CD workflows, PR/issue templates
```

## Getting Started

### Prerequisites

- Python 3.12+ (conda environment provided)
- Docker (for CI-reproducible builds)
- Git

### Setup

```bash
# Clone the repository
git clone https://github.com/<owner>/End-to-End-RL-agent-UAV-flight-controller.git
cd End-to-End-RL-agent-UAV-flight-controller

# Install Python dependencies
pip install -e ".[dev]"

# Install pre-commit hooks
pre-commit install

# Build documentation locally
mkdocs serve
```

### Development Workflow

1. Create a feature branch: `git checkout -b feature/<description>`
2. Make changes and commit (pre-commit hooks run automatically)
3. Push and open a Pull Request against `develop`
4. CI pipelines run automatically; self-review using the PR checklist
5. Merge after CI passes and review is complete

## Documentation

Full project documentation is available at the MkDocs site (built via CI):

- **Requirements:** Doorstop-managed requirements with full traceability
- **Architecture:** System diagrams (Mermaid source in `docs/WBS/diagrams/source/`)
- **WBS:** Detailed work breakdown in `docs/WBS/`
- **API Reference:** Auto-generated from docstrings

## License

*TBD*

## Author

Ratan Lal Bunkar
