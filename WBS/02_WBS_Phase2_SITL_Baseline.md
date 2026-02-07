# Phase 2: SITL Baseline System
## Work Breakdown Structure - Detailed Execution Guide

---

**Document Information**

| Field | Value |
|-------|-------|
| **Document Title** | WBS Phase 2: SITL Baseline System |
| **Version** | 2.0 |
| **Date** | February 8, 2026 |
| **Author** | Ratan Lal Bunkar |
| **Status** | Active |
| **Phase** | 2.0 - SITL Simulation Development |
| **WBS IDs Covered** | 2.0-2.6 (all subsections) |

---

### Document Navigation

- **Master Overview**: [00_WBS_Master_Overview.md](./00_WBS_Master_Overview.md)
- **Previous Phase**: [01_WBS_Phase1_Infrastructure.md](./01_WBS_Phase1_Infrastructure.md)
- **Next Phase**: [03_WBS_Phase3_RL_Control.md](./03_WBS_Phase3_RL_Control.md)
- **All Phases**: [See Master Overview Navigation Guide](./00_WBS_Master_Overview.md#15-navigation-to-detail-documents)

---

## Phase 2 Overview

### Scope and Objectives

Phase 2 (Stage 1) establishes the software-in-the-loop simulation foundation for the entire project. This phase develops a high-fidelity UAV dynamics model within Gazebo simulation environment, implements realistic sensor models, and creates a classical PID-based flight controller as baseline. The SITL system runs entirely on development workstation without any embedded hardware. Success provides validated simulation environment, proven UAV dynamics, and baseline controller performance metrics for RL comparison.

### Total Duration and Effort

- **Duration**: 8-10 weeks
- **Total Effort**: 280 person-hours
- **Work Packages**: 24 Level 3 deliverables across 6 Level 2 subsystems

### Key Deliverables

1. Functioning Gazebo simulation with UAV model
2. Documented UAV dynamics and sensor models
3. Classical P ID flight controller achieving stable flight
4. Baseline performance test results and verification report
5. Stage 1 documentation package

### Prerequisites and Dependencies

- **Prerequisites**: Phase 1 (Project Management & Infrastructure) must be substantially complete
- **Dependencies**: Phase 3 (RL Control) requires successful completion of this phase

### Required Resources

**Personnel**: Software developer with robotics/simulation experience (120h), Controls engineer (100h), Test engineer (60h)

**Hardware**: Linux workstation (16GB+ RAM, GPU helpful but not required)

**Software/Tools**: Ubuntu 20.04/22.04, ROS Noetic or ROS2, Gazebo 11+, Python 3.8+

**Budget**: Minimal ($0-$200, all open-source tools)

### Success Criteria

✅ UAV achieves stable hover for 60+ seconds in simulation
✅ Position control accurate to within 0.5m
✅ Attitude control accurate to within 5 degrees
✅ Simulation runs in real-time or faster
✅ All Stage 1→2 gate criteria satisfied

### Risk Summary

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| Simulation instability | High | High | Small timesteps, validated equations |
| PID tuning difficulty | Medium | Medium | Systematic methods, accept "good enough" |
| Model complexity | High | High | Start simple, add incrementally |
| Parameter uncertainty | High | Medium | Sensitivity analysis, literature values |

---

## Detailed WBS Dictionary

### 2.0 STAGE 1: SITL BASELINE SYSTEM (Level 1)

**Duration**: 8-10 weeks | **Effort**: 280 person-hours

**Description**: Establishes software-in-the-loop simulation foundation including Gazebo environment, UAV dynamics model, sensor simulation, and PID baseline controller. Provides validated simulation and performance baseline for RL comparison.

**Key Deliverables**: Functioning Gazebo simulation, documented UAV dynamics, PID controller, verification report

**Success Criteria**: Stable 60s hover, 0.5m position accuracy, 5° attitude accuracy, real-time simulation

**Major Risks**: Simulation instability (High/High), PID tuning difficulty (Med/Med), model complexity (High/High)

---

### 2.1 Simulation Environment (Level 2)

**Duration**: 2 weeks | **Effort**: 50 person-hours

**Description**: Set up complete Gazebo simulation environment including installation, ROS integration, world creation, and physics configuration. Provides virtual testbed for UAV throughout SITL and HIL stages.

**Deliverables**: Gazebo installed, ROS workspace configured, simulation worlds, tuned physics parameters

**Prerequisites**: Phase 1 infrastructure operational

**Success Criteria**: Gazebo launches without errors, ROS-Gazebo communication functional, ≥1x real-time speed

#### Level 3 Work Packages Summary

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 2.1.1 | Gazebo Installation & Configuration | 2 days | 8h | Gazebo installed, docs, test simulation | GUI launches, 30+ FPS, documented setup |
| 2.1.2 | ROS/ROS2 Setup | 3 days | 12h | ROS workspace, Gazebo integration | ROS commands work, workspace builds, topics visible |
| 2.1.3 | Simulation World Design | 4 days | 16h | World files (2+ worlds), docs, screenshots | Worlds load, UAV spawns, real-time performance |
| 2.1.4 | Physics Parameter Tuning | 3 days | 14h | Tuned parameters, docs, benchmarks | 10min stable simulation, timestep ≤0.004s, real-time |

---

### 2.2 UAV Dynamics Model (Level 2)

**Duration**: 3 weeks | **Effort**: 80 person-hours

**Description**: Develop high-fidelity mathematical and computational model of quadrotor UAV dynamics including equations of motion, motor-propeller system, aerodynamics, and mass-inertia properties. Critical for RL training effectiveness and sim-to-real transfer.

**Deliverables**: Dynamics model (C++/Python), documentation with equations, URDF/SDF model, validation results, parameter files

**Prerequisites**: 2.1 Simulation Environment operational

**Success Criteria**: Accurate motion prediction, stable hover at mg/4 per motor, passes validation tests, real-time execution

#### Level 3 Work Packages Summary

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 2.2.1 | Quadrotor Equations of Motion | 5 days | 24h | Documented equations, implementation, unit tests | 6-DOF correct, free-fall matches theory, hover stable |
| 2.2.2 | Motor and Propeller Modeling | 5 days | 20h | Motor model, parameter database, validation | Commands→thrust accurate, 50-100ms time constant |
| 2.2.3 | Aerodynamic Effects | 4 days | 16h | Aero model, drag coefficients, tests | Drag opposes motion, realistic terminal velocity |
| 2.2.4 | Mass and Inertia Properties | 3 days | 12h | Mass/inertia document, URDF/SDF, validation | Gazebo loads correctly, realistic dynamics (~1kg mass) |

---

### 2.3 Sensor Simulation (Level 2)

**Duration**: 2 weeks | **Effort**: 50 person-hours

**Description**: Implement realistic sensor models providing state feedback to flight controller. Primary sensor is IMU (accelerometer + gyroscope) with realistic noise. Optional sensors include GPS, barometer, magnetometer.

**Deliverables**: IMU model with noise, Gazebo plugins configured, sensor placement defined, topic interfaces documented

**Prerequisites**: 2.1 Simulation Environment, 2.2 UAV Dynamics Model

**Success Criteria**: IMU publishes at ≥100 Hz, noise matches MEMS IMU specs, measurements reflect vehicle motion

#### Level 3 Work Packages Summary

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 2.3.1 | IMU Sensor Model | 5 days | 20h | IMU implementation, parameters, tests, frame docs | Accel measures specific force, gyro ≥100Hz, realistic noise |
| 2.3.2 | Sensor Noise Characteristics | 3 days | 12h | Noise parameters, generation code, validation | Statistics match specs, white noise characteristics |
| 2.3.3 | Sensor Placement Configuration | 2 days | 8h | Placement specs, URDF/SDF, TF visualization | Frames in URDF, TF tree correct, IMU near CoM |
| 2.3.4 | Optional Sensors Integration | 4 days | 16h | Selected sensors (GPS/baro/mag), parameters | Sensors publish at correct rates, realistic noise |

---

### 2.4 Baseline Controller (Level 2)

**Duration**: 2 weeks | **Effort**: 60 person-hours

**Description**: Develop classical PID-based flight controller with cascaded structure (attitude → velocity → position). Provides baseline performance for RL comparison. Emphasis on functionality rather than optimal performance—"good enough" is acceptable.

**Deliverables**: PID attitude controller, altitude controller, position controller, tuned parameters, documentation with block diagrams

**Prerequisites**: 2.2 UAV Dynamics Model, 2.3 Sensor Simulation (IMU at minimum)

**Success Criteria**: Stable 60s hover, position error <0.5m, attitude error <5°, adequately damped response

#### Level 3 Work Packages Summary

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 2.4.1 | PID Attitude Controller | 5 days | 20h | Attitude controller, initial gains, block diagram | Stabilizes from perturbations, ≥100Hz control loop |
| 2.4.2 | Altitude Controller | 3 days | 12h | Altitude controller, tuned gains, integration | Altitude within ±0.3m, settles in 3-5s |
| 2.4.3 | Position Controller | 4 days | 16h | Position controller, full cascade, test results | Position within ±0.5m, smooth trajectories |
| 2.4.4 | Controller Parameter Tuning | 4 days | 16h | Final parameters, methodology docs, performance tests | Stable over range, overshoot <30%, documented gains |

---

### 2.5 SITL Verification (Level 2)

**Duration**: 1.5 weeks | **Effort**: 40 person-hours

**Description**: Comprehensive verification testing of complete SITL baseline system to demonstrate Stage 1 requirements satisfaction. Includes test scenarios, performance testing, disturbance rejection, and formal verification report.

**Deliverables**: Test scenarios/scripts, baseline performance results, disturbance test results, verification report, simulation videos

**Prerequisites**: 2.4 Baseline Controller (complete system functional)

**Success Criteria**: All Stage 1 requirements verified, test coverage ≥90%, verification report complete and approved, no critical defects

#### Level 3 Work Packages Summary

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 2.5.1 | Test Scenario Creation | 3 days | 12h | Scenario specs, automated scripts, logging | 10+ scenarios defined, automated, repeatable |
| 2.5.2 | Baseline Performance Testing | 4 days | 16h | Performance metrics, plots, comparison to requirements | All requirements met or deviations documented |
| 2.5.3 | Disturbance Rejection Tests | 3 days | 10h | Disturbance results, recovery metrics, summary | Controller recovers, no instability, quantified rejection |
|  2.5.4 | Verification Report | 2 days | 8h | Formal report, verification matrix, test archives | Report complete, all requirements addressed, approved |

---

### 2.6 Stage 1→2 Gate (Level 2)

**Duration**: 1 week | **Effort**: 16 person-hours

**Description**: Formal stage gate review to evaluate Stage 1 completion and authorize Stage 2 transition. Evaluates exit criteria (SITL functional, requirements verified), entry criteria (resources for Stage 2), and overall project health. Conducted by stakeholders with decision authority.

**Deliverables**: Gate review package, exit criteria checklist, entry criteria checklist, gate review report with decision

**Prerequisites**: 2.5 SITL Verification complete

**Success Criteria**: Review conducted per procedures, decision made (go/no-go/conditional), formal authorization if go

#### Level 3 Work Packages Summary

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 2.6.1 | Stage 1 Exit Criteria Verification | 2 days | 6h | Exit checklist with evidence, deviations | All criteria evaluated, evidence linked, status clear |
| 2.6.2 | Stage 2 Entry Criteria Validation | 2 days | 6h | Entry checklist, gap analysis, readiness summary | All criteria evaluated, gaps identified, readiness clear |
| 2.6.3 | Documentation Review | 2 days | 4h | Review checklist, corrected docs, review package | All docs present/approved, package distributed 1 week prior |
| 2.6.4 | Go/No-Go Decision | 1 day | 4h | Presentation, minutes, gate report, action items | Review conducted, stakeholders participate, decision documented |

---

## Phase 2 Resource Requirements

### Personnel Skills Matrix

| Work Package | Software Dev | Controls Eng | Test Engineer | Systems Eng |
|--------------|--------------|--------------|---------------|-------------|
| 2.1 Simulation Environment | **Lead** (50h) | Support (10h) | - | Review (5h) |
| 2.2 UAV Dynamics Model | Support (30h) | **Lead** (50h) | - | Review (5h) |
| 2.3 Sensor Simulation | **Lead** (40h) | Support (10h) | - | - |
| 2.4 Baseline Controller | Support (20h) | **Lead** (60h) | - | Review (5h) |
| 2.5 SITL Verification | Support (20h) | Support (20h) | **Lead** (40h) | Review (10h) |
| 2.6 Stage Gate | - | Support (10h) | Support (10h) | **Lead** (16h) |

### Tools and Software Stack

**Core Platform**: Ubuntu 20.04/22.04, ROS Noetic (ROS1) or ROS2 Humble

**Simulation**: Gazebo 11 (ROS1) or Gazebo Fortress/Harmonic (ROS2)

**Languages**: Python 3.8+, C++14+

**Libraries**: Eigen (linear algebra), NumPy, Matplotlib, ROS packages (tf, sensor_msgs, geometry_msgs)

**Build Tools**: CMake, Colcon (ROS2) or Catkin (ROS1)

---

## Phase 2 Risks and Mitigations

### High-Priority Risks

| Risk | Probability | Impact | Mitigation Strategy |
|------|------------|--------|-------------------|
| Simulation instability | High | High | Use validated physics parameters; small timesteps (≤0.004s); test incrementally |
| PID tuning takes longer than expected | High | High | Systematic tuning (Ziegler-Nichols); accept "good enough"; allocate buffer time |
| UAV dynamics model too complex | High | High | Start simple rigid body; add complexity incrementally; validate each addition |
| Parameter uncertainty (motors/inertias) | High | Medium | Use literature values for similar vehicles; sensitivity analysis; document assumptions |

### Medium-Priority Risks

| Risk | Probability | Impact | Mitigation Strategy |
|------|------------|--------|-------------------|
| Gazebo installation/compatibility issues | Medium | Medium | Use tested version combinations; follow official docs; use Docker if conflicts |
| Controller-simulator coupling issues | Medium | High | Clean interfaces; standard ROS messages; test interface early |
| Sensor noise model unrealistic | Medium | High | Reference actual datasheets (MPU-6050); validate noise statistics |
| Performance issues (slow simulation) | Medium | Medium | Reduce visual complexity; disable shadows; optimize physics solver |

---

## Cross-References

### Prerequisites
- **Phase 1 (Infrastructure)**: Core infrastructure (version control, documentation, requirements) operational

### Outputs Used By
- **Phase 3 (RL Control)**: Validated SITL environment, baseline performance metrics, functional sensor models
- **Phase 4 (HIL)**: Dynamics model and control architecture as reference

### Related Documents
- **Master Overview**: [00_WBS_Master_Overview.md](./00_WBS_Master_Overview.md)
- **Previous**: [01_WBS_Phase1_Infrastructure.md](./01_WBS_Phase1_Infrastructure.md)
- **Next**: [03_WBS_Phase3_RL_Control.md](./03_WBS_Phase3_RL_Control.md)

---

## Document Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | February 7, 2026 | Ratan Lal Bunkar | Initial WBS (monolithic document) |
| 2.0 | February 8, 2026 | Ratan Lal Bunkar | Extracted Phase 2 standalone document with comprehensive tables |

---

**End of Phase 2 Document**

**Navigate to**: [← Phase 1](./01_WBS_Phase1_Infrastructure.md) | [Master Overview](./00_WBS_Master_Overview.md) | [Phase 3 →](./03_WBS_Phase3_RL_Control.md)
