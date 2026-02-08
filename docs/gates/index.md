# Stage Gate Criteria

This document defines the go/no-go criteria for each phase transition in the project.

## Gate Review Process (Solo Developer Adaptation)

Since this is a solo project, gate reviews are structured self-assessments:

1. Complete the gate checklist below
2. Score each criterion: **PASS** / **PARTIAL** / **FAIL**
3. Document evidence for each criterion
4. Make go/no-go decision: All "Must" criteria must PASS; ≥80% of "Should" criteria must PASS
5. Record the decision in a gate review document using the [Gate Review Template](#gate-review-template)

---

## Gate 1: Infrastructure → SITL Baseline (Phase 1 → Phase 2)

### Exit Criteria (Phase 1 Complete)

| # | Criterion | Priority | Evidence | Status |
|---|-----------|----------|----------|--------|
| 1 | Git repository with monorepo structure created | Must | Repo URL, directory listing | |
| 2 | ≥40 requirements documented in Doorstop with traceability links | Must | `doorstop` validation output | |
| 3 | Simulation CI/CD pipeline triggers and passes on PR | Must | GitHub Actions run screenshot | |
| 4 | Firmware CI/CD pipeline triggers and passes on PR | Must | GitHub Actions run screenshot | |
| 5 | MkDocs documentation site builds successfully | Must | `mkdocs build` output | |
| 6 | Branching strategy documented in CONTRIBUTING.md | Must | File exists, reviewed | |
| 7 | PR and issue templates configured | Should | `.github/` directory listing | |
| 8 | Pre-commit hooks configured and passing | Should | `pre-commit run --all-files` output | |
| 9 | 6 document templates created | Should | Template files listed | |
| 10 | Stage gate criteria defined for all 4 gates | Should | This document complete | |
| 11 | Docker environments for sim and firmware defined | Should | Dockerfiles exist, build tested | |
| 12 | `pyproject.toml` with dependency groups configured | Should | File exists, `pip install -e .` works | |

### Entry Criteria (Phase 2 Ready)

| # | Criterion | Priority |
|---|-----------|----------|
| 1 | Development workstation meets minimum specs (8+ cores, 16+ GB RAM) | Must |
| 2 | Ubuntu 24.04 or Docker available for ROS 2 Jazzy | Must |
| 3 | Internet access for package installation | Must |
| 4 | Gate 1 exit criteria met | Must |

---

## Gate 2: SITL Baseline → RL Control (Phase 2 → Phase 3)

### Exit Criteria (Phase 2 Complete)

| # | Criterion | Priority | Evidence | Status |
|---|-----------|----------|----------|--------|
| 1 | Gazebo simulation runs at ≥1× real-time | Must | Benchmark log | |
| 2 | UAV model with 6-DOF dynamics validated | Must | Step response plots | |
| 3 | IMU sensor model publishing at ≥100 Hz with realistic noise | Must | `ros2 topic hz` output | |
| 4 | PID controller achieves stable hover for ≥60 seconds | Must | Flight log, video recording | |
| 5 | PID position accuracy within ±0.5 m during hover | Must | Position error time series | |
| 6 | PID attitude accuracy within ±5° during hover | Must | Attitude error time series | |
| 7 | ≥2 distinct Gazebo world files created | Should | World file listing | |
| 8 | ≥10 automated SITL test scenarios | Should | `pytest` output | |
| 9 | ≥90% requirement coverage (SIM-xxx requirements) | Should | Verification matrix | |
| 10 | Disturbance rejection test passed | Should | Recovery plots | |
| 11 | SITL documentation complete (design doc + test report) | Should | Doc review checklist | |
| 12 | All Phase 2 CI pipelines green | Must | GitHub Actions dashboard | |

### Entry Criteria (Phase 3 Ready)

| # | Criterion | Priority |
|---|-----------|----------|
| 1 | Stable SITL environment with PID baseline | Must |
| 2 | Gymnasium-compatible environment wrapper ready | Must |
| 3 | Python 3.12 with Stable-Baselines3 installed | Must |
| 4 | GPU available for training (local or cloud) | Should |
| 5 | Gate 2 exit criteria met | Must |

---

## Gate 3: RL Control → HIL Flight Control (Phase 3 → Phase 4)

### Exit Criteria (Phase 3 Complete)

| # | Criterion | Priority | Evidence | Status |
|---|-----------|----------|----------|--------|
| 1 | RL policy achieves stable hover for ≥60 seconds in sim | Must | Flight log, reward curve | |
| 2 | RL policy meets or exceeds PID on tracking error | Must | Comparison table | |
| 3 | RL policy meets or exceeds PID on settling time | Must | Step response comparison | |
| 4 | Training reward curve plateaued across ≥3 seeds | Must | TensorBoard screenshots | |
| 5 | Policy network ≤19,076 parameters | Must | Model summary output | |
| 6 | INT8 quantized model performance within 5% of FP32 | Must | Quantization comparison | |
| 7 | Model exported in embedded-friendly format | Must | Export file, size report | |
| 8 | ≥3 training iterations with documented analysis | Should | Training log entries | |
| 9 | Domain randomization tested (mass, noise, motor gains) | Should | Generalization test results | |
| 10 | RL framework trade study documented | Should | Trade study document | |
| 11 | Sim-to-real gap analysis document complete | Should | Analysis report | |
| 12 | All Phase 3 CI pipelines green | Must | GitHub Actions dashboard | |

### Entry Criteria (Phase 4 Ready)

| # | Criterion | Priority |
|---|-----------|----------|
| 1 | Quantized RL model ready for embedded deployment | Must |
| 2 | Development board (STM32 Nucleo or similar) available | Must |
| 3 | ARM GCC toolchain installed and tested | Must |
| 4 | UART/USB cable for MCU-PC communication | Must |
| 5 | Gate 3 exit criteria met | Must |

---

## Gate 4: HIL Flight Control → Custom Hardware (Phase 4 → Phase 5)

### Exit Criteria (Phase 4 Complete)

| # | Criterion | Priority | Evidence | Status |
|---|-----------|----------|----------|--------|
| 1 | RL inference executes in <5 ms on target MCU | Must | Timing measurement log | |
| 2 | Control loop runs at ≥100 Hz with <1 ms jitter | Must | Timing histogram | |
| 3 | HIL loop: Gazebo ↔ MCU bidirectional communication working | Must | HIL test log | |
| 4 | Embedded inference matches PC inference within tolerance | Must | Numerical comparison | |
| 5 | FreeRTOS task architecture validated | Must | Task timing report | |
| 6 | Safety monitor running at 100 Hz as highest-priority task | Must | RTOS trace | |
| 7 | PID fallback controller tested and verified | Must | Fallback test log | |
| 8 | Communication protocol with CRC error detection | Should | Protocol test report | |
| 9 | Firmware unit tests passing via QEMU | Should | Test output | |
| 10 | MCU resource usage documented (CPU, RAM, Flash) | Should | Resource report | |
| 11 | HIL design document and test report complete | Should | Document review | |
| 12 | All Phase 4 CI pipelines green | Must | GitHub Actions dashboard | |

### Entry Criteria (Phase 5 Ready)

| # | Criterion | Priority |
|---|-----------|----------|
| 1 | Firmware validated on dev board | Must |
| 2 | KiCad installed and configured | Must |
| 3 | Component sourcing research complete | Should |
| 4 | PCB fabrication vendor identified | Should |
| 5 | Gate 4 exit criteria met | Must |

---

## Gate Review Template

```markdown
# Gate Review: Gate [N] — [Phase X] → [Phase Y]

**Date:** YYYY-MM-DD
**Reviewer:** [Name]
**Decision:** GO / NO-GO / CONDITIONAL GO

## Checklist Results

| # | Criterion | Priority | Status | Evidence |
|---|-----------|----------|--------|----------|
| 1 | ... | Must | PASS/FAIL | [link to evidence] |

## Summary

- **Must criteria:** X/Y passed
- **Should criteria:** X/Y passed
- **Overall pass rate:** X%

## Issues / Risks

| Issue | Severity | Mitigation |
|-------|----------|------------|
| | | |

## Decision Rationale

[Why go / no-go / conditional. For conditional: list conditions that must be met.]

## Action Items

| Action | Owner | Due Date |
|--------|-------|----------|
| | | |
```
