# System Architecture Overview

## System Context

The UAV flight controller system operates within the following context:

```mermaid
graph TB
    subgraph "Ground Station"
        GCS[Ground Control Software]
        RC[RC Transmitter]
    end

    subgraph "UAV System"
        subgraph "Flight Controller"
            SF[Sensor Fusion]
            RL[RL Policy / Neural Net]
            MC[Motor Control]
            SM[Safety Monitor]
            PID[PID Fallback]
        end
        IMU[IMU - Accel/Gyro]
        BARO[Barometer]
        GPS[GPS Module]
        MAG[Magnetometer]
        ESC[ESC x4]
        MOTORS[Motors x4]
        BATT[Battery]
    end

    RC -->|RC Signals| SF
    GCS <-->|MAVLink Telemetry| SF
    IMU --> SF
    BARO --> SF
    GPS --> SF
    MAG --> SF
    SF --> RL
    SF --> PID
    RL --> MC
    PID -.->|Fallback| MC
    SM -->|Override| MC
    MC --> ESC --> MOTORS
    BATT -->|Voltage/Current| SM
```

## Development Stages

```mermaid
graph LR
    S1[Phase 2: SITL<br/>Gazebo Sim + PID]
    S2[Phase 3: RL Training<br/>Policy Learning]
    S3[Phase 4: HIL<br/>MCU + Sim Loop]
    S4[Phase 5: Custom HW<br/>PCB Design]
    S5[Phase 6: Integration<br/>Flight Test]

    S1 -->|Gate 1| S2
    S2 -->|Gate 2| S3
    S3 -->|Gate 3| S4
    S4 -->|Gate 4| S5
```

## Key Design Decisions

1. **ROS 2 Jazzy** — LTS through May 2029, widest package ecosystem
2. **Gazebo Harmonic** — Tight ROS 2 integration, modern rendering
3. **Stable-Baselines3** — Well-documented PPO/SAC implementations
4. **FreeRTOS** — Industry-standard embedded RTOS, large community
5. **PID Fallback** — Safety-critical backup ensures flyable state at all times
