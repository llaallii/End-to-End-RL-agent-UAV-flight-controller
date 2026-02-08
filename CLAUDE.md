# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Common Commands

All development commands are available via `make <target>` (run `make help` for the full list).

### Setup
```bash
pip install -e ".[dev]"       # Dev tools: ruff, mypy, pytest, pre-commit, doorstop, mkdocs
pip install -e ".[sim]"       # Simulation: gymnasium, stable-baselines3, numpy, scipy, matplotlib
pip install -e ".[train]"     # Training: torch, tensorboard, wandb, stable-baselines3
pre-commit install            # Install git hooks
```

### Lint, Format, Typecheck
```bash
make lint                     # ruff check simulation/ training/ tests/
make format                   # ruff format simulation/ training/ tests/
make typecheck                # mypy simulation/ training/ --ignore-missing-imports
make lint-all                 # lint + typecheck combined
```

### Test
```bash
make test                     # pytest tests/ simulation/ training/ -v --tb=short
make test-cov                 # Same with --cov and HTML coverage report
make test-sim                 # Simulation tests only (excludes hardware marker)
pytest tests/path/test_foo.py::TestClass::test_method  # Single test
pytest -m "not slow"          # Skip slow tests
pytest -m integration         # Only integration tests
```

Markers: `slow`, `integration`, `hardware`, `safety`

### Docs & Requirements
```bash
make docs-serve               # Local MkDocs dev server
make docs-build               # Build static site (strict mode)
make reqs-validate            # Doorstop requirements traceability check
make reqs-publish             # Generate HTML from Doorstop to docs/requirements/output/
```

### Docker
```bash
make docker-sim               # Build simulation image (ROS 2 Jazzy + Gazebo Harmonic)
make docker-fw                # Build firmware image (ARM GCC + Unity + QEMU)
make docker-up / docker-down  # Start/stop docker-compose services
```

## Architecture

This is a multi-domain project building an end-to-end RL-based autonomous UAV flight controller across six sequential phases, each gated by a stage-gate review.

### Four Domains

| Domain | Directory | Language | Stack |
|--------|-----------|----------|-------|
| **Simulation** | `simulation/` | Python | ROS 2 Jazzy, Gazebo Harmonic, Gymnasium |
| **RL Training** | `training/` | Python | Stable-Baselines3 (SAC/PPO/TD3), PyTorch, W&B |
| **Firmware** | `firmware/` | C/C++ | ARM Cortex-M, FreeRTOS, CMake |
| **Hardware** | `hardware/` | -- | KiCad PCB design |

Supporting directories: `tests/` (integration/system/performance/safety), `tools/` (Docker, CI, scripts), `docs/` (MkDocs + Mermaid diagrams), `reqs/` (Doorstop requirements).

### How Domains Connect

1. **Simulation** (`simulation/core/`) provides 6-DOF quadrotor dynamics via `QuadrotorDynamics` using RK4 integration, with sensor models (IMU, barometer, GPS, optical flow) and a `CascadedPIDController` baseline (outer loop 50 Hz, inner loop 250 Hz).
2. **Training** wraps the simulation in `QuadrotorHoverEnv` (a Gymnasium `gym.Env`), trains an RL policy (2-layer MLP, 128 neurons/layer) using SB3 algorithms, and exports a quantized INT8 model.
3. **Firmware** (future) runs the quantized policy on an ARM MCU under FreeRTOS with a PID fallback and 100 Hz safety monitor.
4. **Hardware** (future) is a custom PCB flight controller hosting the firmware.

### Key Classes and Data Flow

**Simulation core** (`simulation/core/`):
- `QuadrotorDynamics` -- 6-DOF rigid body physics with RK4 integration; state is 13-element vector (position, velocity, quaternion [w,x,y,z], angular_velocity)
- `QuadrotorParams` / `QuadrotorState` -- dataclasses loaded from `simulation/config/quadrotor_params.yaml`; conventions: World=ENU, Body=FLU, quaternion Hamilton [w,x,y,z]
- `MotorModel` -- rotor dynamics with spin-up/down delays
- `AerodynamicsModel` -- drag forces and torques
- `quaternion_utils` -- Hamilton quaternion operations

**Controllers** (`simulation/controllers/`):
- `CascadedPIDController` -- Position(P) -> Velocity(PI) -> Attitude(P) -> Rate(PID) -> `QuadXMixer`
- Gains loaded from `simulation/config/pid_gains.yaml`

**Sensors** (`simulation/sensors/`): `IMUSensor`, `BarometerSensor`, `GPSSensor`, `OpticalFlowSensor` with configurable noise models from `simulation/config/sensor_params.yaml`

**Gymnasium environment** (`training/envs/quadrotor_hover_env.py`):
- `QuadrotorHoverEnv` -- observation (15-dim normalized): position error, velocity, Euler angles, angular velocity, goal position; action (4-dim [-1,1]): mapped to motor speeds; shaped reward with position/velocity/attitude/rate/action-smoothness penalties; safety termination on tilt/altitude/velocity/geofence violations
- `HoverEnvConfig` dataclass (`training/envs/config.py`) loaded from `training/configs/hover_env.yaml`

**Training pipeline** (`training/scripts/train_hover.py`):
- `train()` function selects SAC/PPO/TD3, wraps env with `Monitor` + `make_vec_env()`, saves checkpoints to `training/models/`
- `TrainConfig` / `WandbConfig` dataclasses (`training/configs/train_config.py`) loaded from `training/configs/train_hover.yaml`

### Key Design Constraints

- RL policy: **<=19,076 parameters** (~24 KB INT8) to fit MCU memory
- Inference latency: **<5 ms** on target MCU
- Control loop: **>=100 Hz** with **<1 ms jitter**
- INT8 quantization must degrade performance **<5%** vs FP32
- Safety monitor runs at highest RTOS priority; PID fallback always available

### Configuration Pattern

All major components use a **dataclass + YAML** pattern: a Python dataclass with `from_yaml()` and `default()` class methods, backed by a YAML config file. Key config files:
- `simulation/config/quadrotor_params.yaml` -- physical parameters (mass, inertia, motors, aero)
- `simulation/config/pid_gains.yaml` -- PID controller gains
- `simulation/config/sensor_params.yaml` -- sensor noise specs
- `training/configs/hover_env.yaml` -- environment physics, observation, reward, termination, domain randomization
- `training/configs/train_hover.yaml` -- algorithm, policy network, hyperparameters, logging

## Conventions

### Branching (Simplified Git Flow)
- `main` -- stable, gate-passed milestones only (protected)
- `develop` -- integration branch, default PR target
- `feature/*`, `bugfix/*`, `docs/*`, `infra/*` -- topic branches from `develop`
- Squash merge feature branches into `develop`; merge `develop` -> `main` at stage gates

### Commits
```
<type>(<scope>): <short description>
```
**Types:** `feat`, `fix`, `docs`, `ci`, `refactor`, `test`, `chore`, `perf`
**Scopes:** `sim`, `fw`, `rl`, `hw`, `safety`, `infra`, `docs`

### Code Style
- **Python:** Ruff formatter/linter, 100-char line length, target py312. Rules: E, W, F, I, N, UP, B, SIM, RUF (E501 ignored). First-party packages: `simulation`, `training`.
- **C/C++:** Google C++ Style Guide (adapted). `snake_case` functions/variables, `UPPER_CASE` constants. Linted with cppcheck + clang-tidy.
- **Docs:** Markdown with Mermaid diagrams (`.mmd` files).

### CI Pipelines (GitHub Actions)
- **sim-ci.yml** -- ruff check/format, mypy, pytest with coverage (triggers on simulation/, training/, tests/ changes)
- **fw-ci.yml** -- cppcheck, ARM GCC build (triggers on firmware/ changes)
- **docs-ci.yml** -- mkdocs build --strict, deploy to GitHub Pages on main (triggers on docs/, reqs/ changes)

## Requirements Traceability

Requirements are managed with Doorstop in `reqs/` across six categories:

| Prefix | Category | Count |
|--------|----------|-------|
| SYS | System-level | 9 |
| SIM | Simulation & PID baseline | 21 |
| RL | RL training & policy | 18 |
| FW | Firmware & real-time | 12 |
| HW | Hardware & PCB | 7 |
| SAF | Safety & fault handling | 29 |

Validate with `make reqs-validate`. Each requirement is a YAML file (e.g., `reqs/RL/RL001.yml`) with parent-child traceability links. Code components reference requirements in comments/docstrings (e.g., SIM003, RL007).

## Test Organization

- **Unit tests:** `simulation/tests/` -- physics, controllers, sensors, quaternion math, types
- **Integration tests:** `training/envs/tests/` -- Gymnasium environment behavior
- **Training smoke tests:** `training/scripts/tests/` -- end-to-end training pipeline
- **Cross-domain tests:** `tests/` -- shared fixtures in `tests/conftest.py`
