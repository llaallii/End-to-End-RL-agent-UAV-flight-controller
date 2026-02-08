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
| **Hardware** | `hardware/` | — | KiCad PCB design |

Supporting directories: `tests/` (integration/system/performance/safety), `tools/` (Docker, CI, scripts), `docs/` (MkDocs + Mermaid diagrams), `reqs/` (Doorstop requirements).

### How Domains Connect

1. **Simulation** provides a Gazebo physics environment with sensor models (IMU, barometer, GPS, optical flow) and a PID baseline controller validated for stable hover.
2. **Training** wraps the simulation in a Gymnasium environment, trains an RL policy (2-layer MLP, 128 neurons/layer), and exports a quantized INT8 model.
3. **Firmware** runs the quantized policy on an ARM MCU under FreeRTOS with a PID fallback and 100 Hz safety monitor.
4. **Hardware** is a custom PCB flight controller hosting the firmware.

### Key Design Constraints

- RL policy: **≤19,076 parameters** (~24 KB INT8) to fit MCU memory
- Inference latency: **<5 ms** on target MCU
- Control loop: **≥100 Hz** with **<1 ms jitter**
- INT8 quantization must degrade performance **<5%** vs FP32
- Safety monitor runs at highest RTOS priority; PID fallback always available

## Conventions

### Branching (Simplified Git Flow)
- `main` — stable, gate-passed milestones only (protected)
- `develop` — integration branch, default PR target
- `feature/*`, `bugfix/*`, `docs/*`, `infra/*` — topic branches from `develop`
- Squash merge feature branches into `develop`; merge `develop` → `main` at stage gates

### Commits
```
<type>(<scope>): <short description>
```
**Types:** `feat`, `fix`, `docs`, `ci`, `refactor`, `test`, `chore`, `perf`
**Scopes:** `sim`, `fw`, `rl`, `hw`, `safety`, `infra`, `docs`

### Code Style
- **Python:** Ruff formatter/linter, 100-char line length, target py312. Rules: E, W, F, I, N, UP, B, SIM, RUF (E501 ignored — handled by formatter). First-party packages: `simulation`, `training`.
- **C/C++:** Google C++ Style Guide (adapted). `snake_case` functions/variables, `UPPER_CASE` constants. Linted with cppcheck + clang-tidy.
- **Docs:** Markdown with Mermaid diagrams (`.mmd` files).

### CI Pipelines (GitHub Actions)
- **sim-ci.yml** — ruff check/format, mypy, pytest with coverage (triggers on simulation/, training/, tests/ changes)
- **fw-ci.yml** — cppcheck, ARM GCC build (triggers on firmware/ changes)
- **docs-ci.yml** — mkdocs build --strict, deploy to GitHub Pages on main (triggers on docs/, reqs/ changes)

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

Validate with `make reqs-validate`. Each requirement is a YAML file (e.g., `reqs/RL/RL001.yml`) with parent-child traceability links.
