# Phase 4: HIL Flight Control
## Work Breakdown Structure - Detailed Execution Guide

---

**Document Information**

| Field | Value |
|-------|-------|
| **Document Title** | WBS Phase 4: HIL Flight Control |
| **Version** | 2.0 |
| **Date** | February 8, 2026 |
| **Author** | Ratan Lal Bunkar |
| **Status** | Active |
| **Phase** | 4.0 - Hardware-in-the-Loop Integration |
| **WBS IDs Covered** | 4.0-4.7 (all subsections) |

---

### Document Navigation

- **Master Overview**: [00_WBS_Master_Overview.md](./00_WBS_Master_Overview.md)
- **Previous Phase**: [03_WBS_Phase3_RL_Control.md](./03_WBS_Phase3_RL_Control.md)
- **Next Phase**: [05_WBS_Phase5_Custom_Hardware.md](./05_WBS_Phase5_Custom_Hardware.md)
- **All Phases**: [See Master Overview Navigation Guide](./00_WBS_Master_Overview.md#15-navigation-to-detail-documents)

---

## Phase 4 Overview

### Scope and Objectives

Phase 4 (Stage 3) integrates the trained RL policy with embedded hardware through hardware-in-the-loop simulation. This phase selects and procures MCU, develops firmware architecture with RTOS, ports RL inference to embedded platform, creates HIL interface between Gazebo and firmware, validates real-time performance, and implements safety and fault handling. Success produces validated embedded flight control firmware ready for custom hardware integration.

#### Figure 4.1: HIL System Architecture

![HIL Architecture](diagrams/rendered/architecture/D1.4_hil_architecture.png)

*Figure 4.1 shows the complete HIL architecture with Gazebo simulation connected to STM32 MCU firmware via UART/USB interface, including sensor data flow and control commands.*

### Total Duration and Effort

- **Duration**: 6-8 weeks
- **Total Effort**: 240 person-hours
- **Work Packages**: 28 Level 3 deliverables across 7 Level 2 subsystems

### Key Deliverables

1. Selected and procured MCU development board
2. Firmware architecture with RTOS
3. RL inference engine ported to embedded platform
4. HIL interface connecting Gazebo to firmware
5. Real-time performance validation results
6. Safety and fault handling implementation
7. Stage 3 documentation package

### Prerequisites and Dependencies

- **Prerequisites**: Phase 3 (RL Control System) complete with compressed policy, Stage 2→3 gate approved
- **Dependencies**: Phase 5 (Custom Hardware) uses firmware architecture from this phase

### Required Resources

**Personnel**: Embedded engineer (140h), ML engineer (40h), Software developer (40h), Systems engineer (20h)

**Hardware**: MCU development board (ARM Cortex-M4/M7), debugger/programmer (ST-Link/J-Link), development workstation

**Software/Tools**: ARM GCC toolchain, RTOS (FreeRTOS/Zephyr), debugger (GDB, Ozone), ROS, Gazebo, TensorFlow Lite Micro or custom RL inference library

**Budget**: $200-500 for MCU dev boards, debugger, components

### Success Criteria

✅ RL policy executes on MCU in <10ms per inference
✅ HIL simulation runs stably with firmware in the loop
✅ Real-time constraints met (deterministic timing, low jitter)
✅ Safety systems functional (watchdog, failsafe)
✅ All Stage 3→4 gate criteria satisfied

#### Figure 4.2: HIL Communication Protocol

![HIL Communication](diagrams/rendered/dataflow/D3.2_hil_communication.png)

*Figure 4.2 illustrates the bidirectional communication protocol between Gazebo and MCU firmware with message formats, timing, and error handling.*

#### Figure 4.3: MAVLink Message Flow

![MAVLink Message Flow](diagrams/rendered/dataflow/D3.4_mavlink_message_flow.png)

*Figure 4.3 details the MAVLink messaging protocol used for telemetry, commands, and status updates between flight controller and ground control station.*

### Risk Summary

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| Real-time deadline misses | High | High | Profile code, optimize inference, use faster MCU if needed |
| Memory constraints | High | High | Compress policy further, optimize memory usage, consider external memory |
| HIL interface complexity | Medium | High | Design simple protocol, test incrementally |
| Numerical precision issues | Medium | Medium | Validate fixed-point vs floating-point carefully |

---

## Detailed WBS Dictionary

### 4.0 STAGE 3: HIL FLIGHT CONTROL (Level 1)

**Duration**: 6-8 weeks | **Effort**: 240 person-hours

**Description**: Integrates trained RL policy with embedded hardware through HIL simulation. Includes MCU selection, firmware development with RTOS, RL inference porting, HIL interface creation, real-time validation, and safety implementation.

**Key Deliverables**: MCU with firmware, RL inference on embedded, HIL interface, real-time validation, safety systems

**Success Criteria**: Policy runs <10ms on MCU, HIL stable, real-time constraints met, safety functional

**Major Risks**: Real-time deadlines (High/High), memory constraints (High/High), HIL complexity (Med/High)

---

### Level 2 Subsystems and Level 3 Work Packages

#### 4.1 MCU Selection & Procurement (2 - Duration: 1 week | Effort: 32h)

**Description**: Select appropriate microcontroller based on computation requirements, memory needs, I/O interfaces, and cost. Procure development boards and tools.

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 4.1.1 | Requirements Analysis | 2 days | 8h | Requirements doc, computation/memory estimates | CPU speed ≥168MHz, RAM ≥256KB, Flash ≥512KB, FPU desirable |
| 4.1.2 | MCU Comparison | 2 days | 8h | Comparison matrix, trade study | Compare STM32, NXP, ESP32; evaluate against requirements |
| 4.1.3 | Development Board Selection | 1 day | 4h | Selected board, justification | Development board available, debugger support, community |
| 4.1.4 | Component Ordering | 1 day | 4h | Purchase orders, lead time tracking | Boards ordered, delivery schedule, backup options identified |

---

#### 4.2 Firmware Architecture (Duration: 1.5 weeks | Effort: 48h)

**Description**: Design firmware architecture including RTOS selection, module structure, control loop design, and memory management.

#### Figure 4.4: Firmware Architecture

![Firmware Architecture](diagrams/rendered/components/D5.2_firmware_architecture.png)

*Figure 4.4 shows the layered firmware architecture with RTOS, drivers, sensor interfaces, RL inference engine, and motor control modules.*

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 4.2.1 | RTOS Selection | 2 days | 8h | Selected RTOS, justification | FreeRTOS recommended; lightweight, well-documented, MCU support |
| 4.2.2 | Firmware Module Design | 3 days | 16h | Module architecture, interfaces | Sensor, control, communication, safety modules defined |
| 4.2.3 | Control Loop Architecture | 3 days | 12h | Control loop design, timing diagram | ≥100Hz control rate, deterministic execution |
| 4.2.4 | Memory Management Strategy | 2 days | 12h | Memory layout, allocation strategy | Static allocation for real-time, heap minimized |

#### Figure 4.5: Sensor Interface Architecture

![Sensor Interface](diagrams/rendered/components/D5.3_sensor_interface_diagram.png)

*Figure 4.5 details the sensor interface layer with I2C/SPI drivers, IMU data acquisition, filtering, and state estimation integration.*

#### Figure 4.6: Motor Control Architecture

![Motor Control](diagrams/rendered/components/D5.4_motor_control_architecture.png)

*Figure 4.6 illustrates the motor control subsystem with PWM generation, ESC interface, and thrust command processing.*

---

#### 4.3 RL Inference Engine Porting (Duration: 1.5 weeks | Effort: 48h)

**Description**: Port trained RL policy to embedded platform using TensorFlow Lite Micro or custom inference engine. Validate numerical accuracy.

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 4.3.1 | Inference Library Selection | 2 days | 8h | Selected library, trade study | TFLite Micro, CMSIS-NN, or custom; memory/speed tradeoffs |
| 4.3.2 | Model Conversion | 3 days | 16h | Converted model, metadata | Policy converted to target format, parameters preserved |
| 4.3.3 | Inference Implementation | 4 days | 16h | Inference code, integration with firmware | Forward pass executes, inputs/outputs connected |
| 4.3.4 | Numerical Validation | 2 days | 8h | Validation results, error analysis | Output matches PC version within acceptable tolerance |

---

#### 4.4 HIL Interface Development (Duration: 1.5 weeks | Effort: 48h)

**Description**: Create HIL interface connecting Gazebo simulation to firmware running on MCU. Bidirectional communication for sensor data and control commands.

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 4.4.1 | Communication Protocol Design | 2 days | 12h | Protocol specification, message formats | Simple binary protocol, minimal overhead, error detection |
| 4.4.2 | Gazebo HIL Plugin | 3 days | 16h | Gazebo plugin, ROS interface | Sends sensor data to MCU, receives control commands |
| 4.4.3 | Firmware Communication Interface | 3 days | 12h | UART/USB driver, message parser | Reliable communication, buffering, timeout handling |
| 4.4.4 | Data Serialization | 2 days | 8h | Serialization/deserialization, endianness handling | Efficient encoding, cross-platform compatibility |

---

#### 4.5 Real-Time Validation (Duration: 1.5 weeks | Effort: 40h)

**Description**: Validate that firmware meets real-time constraints including inference timing, control loop determinism, and latency bounds.

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 4.5.1 | Timing Analysis | 3 days | 12h | Timing measurements, profiling results | Inference <10ms, control loop <10ms total |
| 4.5.2 | Latency Measurement | 2 days | 8h | End-to-end latency, communication delays | Total latency quantified, acceptable for control |
| 4.5.3 | Jitter Characterization | 3 days | 12h | Jitter measurements, histograms | Control loop jitter <1ms, deterministic execution |
| 4.5.4 | Determinism Verification | 2 days | 8h | Repeatability tests, timing guarantees | Execution time consistent, worst-case bounds established |

---

#### 4.6 Safety & Fault Handling (Duration: 1 week | Effort: 32h)

**Description**: Implement safety systems including watchdog, failsafe conditions, error detection, and recovery procedures.

#### Figure 4.7: Flight Controller State Machine

![Flight Controller FSM](diagrams/rendered/behavioral/D6.1_flight_controller_state_machine.png)

*Figure 4.7 shows the main flight controller state machine with states for initialization, armed, flying, emergency, and fault handling.*

#### Figure 4.8: Initialization Sequence

![Initialization Sequence](diagrams/rendered/behavioral/D6.3_initialization_sequence.png)

*Figure 4.8 details the power-on initialization sequence with sensor calibration, system checks, and arm/disarm procedures.*

#### Figure 4.9: Emergency Shutdown Procedure

![Emergency Shutdown](diagrams/rendered/behavioral/D6.4_emergency_shutdown_sequence.png)

*Figure 4.9 illustrates the emergency shutdown sequence triggered by critical faults, including motor kill and safe state entry.*

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 4.6.1 | Watchdog Implementation | 2 days | 8h | Watchdog timer config, reset handling | Watchdog triggers on hang, recovers safely |
| 4.6.2 | Failsafe Conditions | 2 days | 8h | Failsafe logic, safe state definition | Triggers on sensor loss, enters safe mode |
| 4.6.3 | Error Detection | 2 days | 8h | Error checking, anomaly detection | Detects comm failures, invalid states, sensor errors |
| 4.6.4 | Recovery Procedures | 2 days | 8h | Recovery logic, graceful degradation | Attempts recovery, logs errors, alerts operator |

#### Figure 4.10: Phase 3-4 Detailed Timeline (Gantt)

![Phase 3 Gantt](diagrams/rendered/timeline/D4.3_phase3_detailed_gantt.png)

*Figure 4.10 provides the detailed 6-8 week timeline for Phases 3-4 with HIL work packages, dependencies, and milestone tracking.*

---

#### 4.7 Stage 3→4 Gate (Duration: 1 week | Effort: 16h)

| WBS ID | Work Package | Duration | Effort | Key Deliverables | Critical Success Factors |
|--------|--------------|----------|--------|------------------|-------------------------|
| 4.7.1 | Stage 3 Exit Criteria Verification | 2 days | 6h | Exit checklist | HIL functional, real-time met, safety implemented |
| 4.7.2 | Stage 4 Entry Criteria Validation | 2 days | 6h | Entry checklist | Hardware design ready, resources available |
| 4.7.3 | Documentation Review | 2 days | 4h | Documentation package | Firmware architecture, HIL interface, validation documented |
| 4.7.4 | Go/No-Go Decision | 1 day | 4h | Gate report, decision | Firmware ready for custom hardware integration |

---

## Phase 4 Resource Requirements

### Personnel Skills Matrix

| Work Package | Embedded Eng | ML Engineer | Software Dev | Systems Eng |
|--------------|--------------|-------------|--------------|-------------|
| 4.1 MCU Selection | **Lead** (32h) | Review (4h) | - | Review (4h) |
| 4.2 Firmware Architecture | **Lead** (48h) | - | Support (8h) | Review (8h) |
| 4.3 RL Inference Porting | Lead (24h) | **Co-lead** (24h) | - | - |
| 4.4 HIL Interface | Lead (32h) | - | **Co-lead** (16h) | - |
| 4.5 Real-Time Validation | **Lead** (32h) | - | Support (8h) | Review (8h) |
| 4.6 Safety Systems | **Lead** (32h) | - | - | Review (8h) |
| 4.7 Stage Gate | Support (8h) | - | - | **Lead** (16h) |

---

## Cross-References

- **Previous**: [03_WBS_Phase3_RL_Control.md](./03_WBS_Phase3_RL_Control.md) - Requires compressed RL policy
- **Next**: [05_WBS_Phase5_Custom_Hardware.md](./05_WBS_Phase5_Custom_Hardware.md) - Firmware architecture feeds hardware design
- **Master**: [00_WBS_Master_Overview.md](./00_WBS_Master_Overview.md)

---

## Document Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 2.0 | February 8, 2026 | Ratan Lal Bunkar | Extracted Phase 4 standalone with comprehensive tables |

---

**End of Phase 4 Document**

**Navigate to**: [← Phase 3](./03_WBS_Phase3_RL_Control.md) | [Master Overview](./00_WBS_Master_Overview.md) | [Phase 5 →](./05_WBS_Phase5_Custom_Hardware.md)
