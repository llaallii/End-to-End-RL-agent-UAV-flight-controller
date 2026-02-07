# Phase 3: RL Control System
## Work Breakdown Structure - Detailed Execution Guide

---

**Document Information**

| Field | Value |
|-------|-------|
| **Document Title** | WBS Phase 3: RL Control System |
| **Version** | 2.0 |
| **Date** | February 8, 2026 |
| **Author** | Ratan Lal Bunkar |
| **Status** | Active |
| **Phase** | 3.0 - RL Training and Development |
| **WBS IDs Covered** | 3.0-3.7 (all subsections) |

---

### Document Navigation

- **Master Overview**: [00_WBS_Master_Overview.md](./00_WBS_Master_Overview.md)
- **Previous Phase**: [02_WBS_Phase2_SITL_Baseline.md](./02_WBS_Phase2_SITL_Baseline.md)
- **Next Phase**: [04_WBS_Phase4_HIL_Flight_Control.md](./04_WBS_Phase4_HIL_Flight_Control.md)
- **All Phases**: [See Master Overview Navigation Guide](./00_WBS_Master_Overview.md#15-navigation-to-detail-documents)

---

## Phase 3 Overview

### Scope and Objectives

Phase 3 (Stage 2) develops the reinforcement learning-based flight controller to replace or augment the classical PID baseline. This phase integrates an RL framework, designs the training environment with observation/action spaces and reward function, architects the neural network policy, conducts iterative training with hyperparameter tuning, evaluates performance against baseline, and prepares the policy for embedded deployment through quantization and compression. This stage represents the core machine learning innovation of the project.

#### Figure 3.1: RL Training Pipeline

![RL Training Pipeline](diagrams/rendered/process/D2.2_rl_training_pipeline.png)

*Figure 3.1 shows the complete RL training workflow from framework integration through policy deployment, including iterative training process and hyperparameter optimization.*

### Total Duration and Effort

- **Duration**: 10-12 weeks
- **Total Effort**: 320 person-hours
- **Work Packages**: 32 Level 3 deliverables across 7 Level 2 subsystems

### Key Deliverables

1. Integrated RL training framework
2. Designed and implemented training environment (Gym wrapper)
3. Neural network policy architecture
4. Trained RL policy with documented training process
5. Performance evaluation comparing RL to baseline
6. Quantized/compressed policy ready for embedded deployment
7. Stage 2 documentation package

### Prerequisites and Dependencies

- **Prerequisites**: Phase 2 (Stage 1 SITL Baseline System) complete, Stage 1→2 gate approved
- **Dependencies**: Phase 4 (HIL) requires compressed RL policy from this phase

### Required Resources

**Personnel**: ML engineer or researcher (200h), Controls engineer for domain knowledge (80h), Systems engineer (40h)

**Hardware**: Training workstation with GPU (NVIDIA GPU with CUDA, 8GB+ VRAM), development workstation for simulation

**Software/Tools**: PyTorch or TensorFlow, Stable-Baselines3/RLlib, Gym/Gymnasium, ROS, Gazebo, TensorBoard

**Budget**: $500-$2000 for cloud GPU compute if local GPU unavailable

### Success Criteria

✅ RL policy achieves stable flight (hover 60+ seconds)
✅ RL performance meets or exceeds baseline on key metrics
✅ Policy size <10MB (preferably <1MB for embedded)
✅ Training process documented and reproducible
✅ Policy validated in multiple test scenarios
✅ All Stage 2→3 gate criteria satisfied

#### Figure 3.2: RL Training State Machine

![RL Training State Machine](diagrams/rendered/behavioral/D6.2_rl_training_state_machine.png)

*Figure 3.2 illustrates the state machine controlling the iterative RL training process with states for initialization, training, evaluation, hyperparameter tuning, and convergence validation.*

### Risk Summary

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| Training instability | High | High | Careful tuning, multiple seeds, patience |
| Reward shaping difficulty | High | High | Iterate on reward design, monitor behaviors |
| Long training times | High | Medium | Parallelize with GPUs/cloud, efficient algorithms |
| Sim-to-real gap | High | High | Domain randomization, robust training |

---

## Detailed WBS Dictionary

### 3.0 STAGE 2: RL CONTROL SYSTEM (Level 1)

**Duration**: 10-12 weeks | **Effort**: 320 person-hours

**Description**: Develops reinforcement learning-based flight controller through framework integration, training environment design, policy architecture, iterative training with hyperparameter tuning, performance evaluation, and compression for embedded deployment. Operates entirely in simulation building on validated SITL system.

**Key Deliverables**: RL framework integrated, training environment, trained policy, evaluation vs baseline, compressed policy

**Success Criteria**: Stable 60s hover, performance ≥ baseline, policy <10MB (ideally <1MB), documented and reproducible

**Major Risks**: Training instability (High/High), reward shaping (High/High), long training times (High/Med), sim-to-real gap (High/High)

---

### 3.1 RL Framework Integration (Level 2)

**Duration**: 1.5 weeks | **Effort**: 40 person-hours

**Description**: Select and integrate RL framework (Stable-Baselines3 recommended) to manage training loop, policy updates, and algorithms. Create Gym environment wrapper for Gazebo simulation. Provides foundation for all RL development.

**Deliverables**: Selected framework, installation, Gym wrapper, training script, documentation

**Prerequisites**: Stage 1 SITL system provides simulation

**Success Criteria**: Framework trains on test environment, Gym wrapper interfaces Gazebo, training loop functional end-to-end

#### Level 3 Work Packages Summary

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 3.1.1 | Framework Selection | 3 days | 12h | Trade study, selected framework, justification | 3+ frameworks evaluated, supports PPO/SAC, documented selection |
| 3.1.2 | Installation and Setup | 2 days | 8h | Installed framework, requirements.txt, GPU config | Framework imports, GPU detected, reproducible install |
| 3.1.3 | Gym Environment Wrapper | 5 days | 20h | Gym class, registration, unit tests, docs | reset/step/render implemented, ROS-Gazebo communication works |
| 3.1.4 | Framework Validation | 2 days | 8h | Test training results, validation report | Simple environment trains successfully, metrics logged |

---

### 3.2 Training Environment Design (Level 2)

**Duration**: 2 weeks | **Effort**: 50 person-hours

**Description**: Design complete training environment including observation space (state representation), action space (control commands), reward function (guides learning), termination conditions, and domain randomization (sim-to-real transfer). Critical design choices that determine learning success.

**Deliverables**: Observation space design, action space design, reward function, termination conditions, domain randomization parameters

**Prerequisites**: 3.1 RL Framework Integration

**Success Criteria**: Spaces properly bounded, reward aligns with objectives, termination prevents runaway, domain randomization covers key variations

#### Level 3 Work Packages Summary

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 3.2.1 | Observation Space Design | 3 days | 12h | Observation specification, normalization scheme | Includes position, velocity, attitude, rates; properly normalized |
| 3.2.2 | Action Space Design | 2 days | 8h | Action specification, saturation limits | Motor commands or body-frame forces; bounded appropriately |
| 3.2.3 | Reward Function Design | 4 days | 16h | Reward equation, tunable parameters | Encourages goal, penalizes crashes, tunable weights |
| 3.2.4 | Episode Termination Conditions | 2 days | 6h | Termination criteria, timeout specification | Terminates on crash/success/timeout; prevents divergence |
| 3.2.5 | Domain Randomization | 3 days | 12h | Randomization parameters, ranges | Mass, inertia, motor gains, sensor noise randomized |

---

### 3.3 Policy Architecture Development (Level 2)

**Duration**: 1.5 weeks | **Effort**: 40 person-hours

**Description**: Design neural network architecture for policy including layer structure, activation functions, input/output normalization, and policy type (stochastic vs deterministic). Architecture affects learning speed, final performance, and embedded deployment feasibility.

#### Figure 3.3: Neural Network Policy Architecture

![Neural Network Architecture](diagrams/rendered/components/D5.5_neural_network_architecture.png)

*Figure 3.3 details the multi-layer neural network architecture with actor-critic structure, shared/separate layers, activation functions, and dimensions.*

**Deliverables**: Network architecture definition, normalization strategy, policy structure, architecture validation

**Prerequisites**: 3.2 Training Environment Design (spaces define network I/O)

**Success Criteria**: Architecture matches observation/action dimensions, forward pass executes correctly, parameter count reasonable (<1M for embedded)

#### Level 3 Work Packages Summary

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 3.3.1 | Neural Network Architecture Selection | 3 days | 12h | Architecture specification, layer sizes | 2-3 hidden layers, 64-256 units/layer, appropriate for task |
| 3.3.2 | Input/Output Normalization | 2 days | 8h | Normalization scheme, statistics collection | Observations normalized to ~[-1,1], actions properly scaled |
| 3.3.3 | Policy Structure Definition | 3 days | 12h | Policy type (stochastic/deterministic), implementation | Matches algorithm (PPO→stochastic), value function if needed |
| 3.3.4 | Architecture Validation | 2 days | 8h | Forward pass tests, parameter count | Executes without errors, <1M parameters for embedded target |

---

### 3.4 Training & Hyperparameter Tuning (Level 2)

**Duration**: 4-6 weeks | **Effort**: 80 person-hours

**Description**: Conduct iterative training of RL policy with systematic hyperparameter tuning. Includes 3+ training iterations with performance analysis between iterations. Long calendar duration but lower human effort (mostly compute time). Most uncertain phase due to RL training variability.

#### Figure 3.4: Hyperparameter Tuning Workflow

![Hyperparameter Tuning](diagrams/rendered/process/D2.3_rl_hyperparameter_tuning.png)

*Figure 3.4 shows the systematic hyperparameter optimization workflow with grid search, random search, and Bayesian optimization strategies across multiple training iterations.*

**Deliverables**: Trained policies (multiple iterations), hyperparameter configurations, training logs, convergence analysis

**Prerequisites**: 3.2 Training Environment, 3.3 Policy Architecture

**Success Criteria**: Policy converges to stable behavior, performance improves across iterations, final policy meets success criteria

#### Level 3 Work Packages Summary

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 3.4.1 | Training Iteration 1 (Baseline) | 1-2 weeks | 16h | Trained policy v1, training logs, hyperparams | Completes without crashes, some learning observed |
| 3.4.2 | Performance Analysis & Adjustment Round 1 | 3 days | 12h | Analysis report, identified issues, adjustment plan | Root causes identified, actionable improvements defined |
| 3.4.3 | Training Iteration 2 | 1-2 weeks | 16h | Trained policy v2, training logs, updated hyperparams | Performance improves vs v1, better stability |
| 3.4.4 | Performance Analysis & Adjustment Round 2 | 3 days | 12h | Analysis report, refinements | Fine-tuning identified, diminishing returns assessed |
| 3.4.5 | Training Iteration 3 (Final) | 1-2 weeks | 16h | Final trained policy, full training logs | Converges to acceptable performance, reproducible |
| 3.4.6 | Final Convergence Validation | 2 days | 8h | Convergence analysis, training completion report | Learning curve plateaus, performance stable across seeds |

---

### 3.5 Performance Evaluation (Level 2)

**Duration**: 2 weeks | **Effort**: 50 person-hours

**Description**: Comprehensive evaluation of trained RL policy vs PID baseline across multiple metrics and test scenarios. Includes quantitative metrics (tracking error, control effort, robustness) and generalization testing (unseen scenarios).

#### Figure 3.5: RL Policy Data Flow

![RL Policy Dataflow](diagrams/rendered/dataflow/D3.3_rl_policy_dataflow.png)

*Figure 3.5 illustrates the complete data flow through the trained RL policy from sensor observations through neural network inference to motor commands.*

**Deliverables**: Evaluation environment, performance metrics, baseline comparison, generalization tests, evaluation report

**Prerequisites**: 3.4 Training complete with acceptable policy

**Success Criteria**: RL meets or exceeds baseline on key metrics, generalizes to unseen scenarios, evaluation documented

#### Level 3 Work Packages Summary

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 3.5.1 | Evaluation Environment Creation | 3 days | 12h | Test scenarios, evaluation scripts, data logging | Scenarios cover key behaviors, automated evaluation |
| 3.5.2 | Performance Metrics Definition | 2 days | 8h | Metrics specification, calculation methods | Includes tracking error, settling time, control effort, robustness |
| 3.5.3 | Baseline Comparison Testing | 4 days | 16h | RL vs PID results, comparison plots, analysis | Fair comparison (same scenarios), statistical significance |
| 3.5.4 | Generalization Testing | 3 days | 10h | Unseen scenario results, generalization assessment | Tests mass variations, wind, sensor noise, degradation analysis |
| 3.5.5 | Evaluation Report | 2 days | 8h | Comprehensive report, conclusions, recommendations | Documents all tests, clear conclusions, recommendations for Stage 3 |

---

### 3.6 Policy Quantization/Compression (Level 2)

**Duration**: 1.5 weeks | **Effort**: 40 person-hours

**Description**: Compress and quantize trained policy for embedded deployment. Reduce model size and inference time while maintaining performance. Prepares policy for resource-constrained MCU in Stage 3.

**Deliverables**: Compressed policy, quantization analysis, performance validation, embedded deployment preparation

**Prerequisites**: 3.4 Training complete, 3.5 Evaluation shows acceptable performance

**Success Criteria**: Policy <1MB, inference time reduces significantly, performance degradation <5% vs full-precision

#### Level 3 Work Packages Summary

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 3.6.1 | Model Size Analysis | 2 days | 8h | Size breakdown, bottleneck identification | Parameter count, memory footprint, layer-wise analysis |
| 3.6.2 | Quantization Techniques Application | 4 days | 16h | Quantized models (INT8, INT16), conversion tools | Post-training quantization or quantization-aware training |
| 3.6.3 | Compression Validation | 3 days | 12h | Performance comparison, size/accuracy tradeoff | Performance degradation quantified, acceptable tradeoff |
| 3.6.4 | Embedded Deployment Preparation | 2 days | 8h | Model export formats, inference library selection | ONNX/TFLite export, TensorFlow Lite Micro or custom inference |

---

### 3.7 Stage 2→3 Gate (Level 2)

**Duration**: 1 week | **Effort**: 16 person-hours

**Description**: Formal stage gate review to evaluate Stage 2 completion and authorize Stage 3 (HIL) transition. Evaluates RL policy performance, compression readiness, and Stage 3 resource availability.

**Deliverables**: Gate review package, exit/entry criteria checklists, gate review report with decision

**Prerequisites**: All Stage 2 work complete (3.1-3.6)

**Success Criteria**: Gate review conducted, decision made (go/no-go/conditional), formal authorization if go

#### Level 3 Work Packages Summary

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 3.7.1 | Stage 2 Exit Criteria Verification | 2 days | 6h | Exit checklist with evidence | RL policy meets performance criteria, compressed <1MB, documented |
| 3.7.2 | Stage 3 Entry Criteria Validation | 2 days | 6h | Entry checklist, gap analysis | MCU selected, resources available, firmware plan ready |
| 3.7.3 | Documentation Review | 2 days | 4h | Review checklist, documentation package | Training process documented, evaluation report complete |
| 3.7.4 | Go/No-Go Decision | 1 day | 4h | Presentation, gate report, decision | Stakeholders agree RL policy ready for HIL deployment |

---

## Phase 3 Resource Requirements

### Personnel Skills Matrix

| Work Package | ML Engineer | Controls Eng | Software Dev | Systems Eng |
|--------------|-------------|--------------|--------------|-------------|
| 3.1 RL Framework Integration | **Lead** (40h) | - | Support (10h) | Review (5h) |
| 3.2 Training Environment Design | **Lead** (30h) | **Co-lead** (20h) | - | - |
| 3.3 Policy Architecture | **Lead** (35h) | Review (5h) | - | - |
| 3.4 Training & Tuning | **Lead** (70h) | Support (10h) | - | - |
| 3.5 Performance Evaluation | Lead (30h) | **Co-lead** (20h) | Support (10h) | Review (10h) |
| 3.6 Policy Compression | **Lead** (35h) | - | Support (5h) | - |
| 3.7 Stage Gate | Support  (10h) | Support (10h) | - | **Lead** (16h) |

### Tools and Software Stack

**ML Framework**: Stable-Baselines3 (recommended) or RLlib

**Deep Learning**: PyTorch (preferred) or TensorFlow

**Environment**: Gym/Gymnasium

**Simulation**: ROS + Gazebo (from Phase 2)

**Monitoring**: TensorBoard, Weights & Biases (optional)

**Compression**: TensorFlow Lite, ONNX, PyTorch Mobile

---

## Phase 3 Risks and Mitigations

### High-Priority Risks

| Risk | Probability | Impact | Mitigation Strategy |
|------|------------|--------|-------------------|
| Training instability / non-convergence | High | High | Use stable algorithms (PPO), multiple seeds, careful hyperparameter tuning, expected multiple attempts |
| Reward shaping difficulty | High | High | Start simple reward, iterate based on observed behaviors, monitor for unintended optimization |
| Long training times (days to weeks) | High | Medium | Use cloud GPUs, parallelize training across seeds, efficient algorithms, accept longer schedule |
| Sim-to-real gap | High | High | Domain randomization (mass, inertia, sensors), robust training, conservative deployment |

### Medium-Priority Risks

| Risk | Probability | Impact | Mitigation Strategy |
|------|------------|--------|-------------------|
| RL policy worse than baseline | Medium | High | Carefully designed reward, adequate training time, accept PID baseline if RL doesn't improve |
| Compression degrades performance | Medium | High | Validate compressed policy thoroughly, use quantization-aware training if needed |
| Framework incompatibilities | Medium | Medium | Pin all package versions, use virtual environments, test early |
| GPU resource constraints | Medium | Medium | Use cloud GPU (AWS/GCP), optimize batch size, train overnight/weekends |

---

## Cross-References

### Prerequisites
- **Phase 2 (SITL Baseline)**: Validated simulation environment, baseline PID performance metrics, sensor models

### Outputs Used By
- **Phase 4 (HIL Flight Control)**: Compressed RL policy for embedded deployment, performance requirements from evaluation

### Related Documents
- **Master Overview**: [00_WBS_Master_Overview.md](./00_WBS_Master_Overview.md)
- **Previous**: [02_WBS_Phase2_SITL_Baseline.md](./02_WBS_Phase2_SITL_Baseline.md)
- **Next**: [04_WBS_Phase4_HIL_Flight_Control.md](./04_WBS_Phase4_HIL_Flight_Control.md)

---

## Document Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | February 7, 2026 | Ratan Lal Bunkar | Initial WBS (monolithic document) |
| 2.0 | February 8, 2026 | Ratan Lal Bunkar | Extracted Phase 3 standalone document with comprehensive tables |

---

**End of Phase 3 Document**

**Navigate to**: [← Phase 2](./02_WBS_Phase2_SITL_Baseline.md) | [Master Overview](./00_WBS_Master_Overview.md) | [Phase 4 →](./04_WBS_Phase4_HIL_Flight_Control.md)
