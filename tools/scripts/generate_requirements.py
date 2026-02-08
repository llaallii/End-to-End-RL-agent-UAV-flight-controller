"""Generate Doorstop requirements tree from extracted WBS requirements.

Run once to bootstrap the requirements database, then manage via `doorstop` CLI.
"""

import os
from pathlib import Path

ROOT = Path(r"c:\Users\ratan\Desktop\End-to-End-RL-agent-UAV-flight-controller")
REQS = ROOT / "reqs"

# ── Document definitions ────────────────────────────────────────────────────
DOCUMENTS = {
    "SYS": {"digits": 3, "parent": None,  "sep": ""},
    "SIM": {"digits": 3, "parent": "SYS", "sep": ""},
    "RL":  {"digits": 3, "parent": "SYS", "sep": ""},
    "FW":  {"digits": 3, "parent": "SYS", "sep": ""},
    "HW":  {"digits": 3, "parent": "SYS", "sep": ""},
    "SAF": {"digits": 3, "parent": "SYS", "sep": ""},
}

# ── Requirements data ───────────────────────────────────────────────────────
REQUIREMENTS: dict[str, list[dict]] = {
    "SYS": [
        {"text": "The project shall progress through 4 validated stages: SITL, RL Training, HIL, and Custom Hardware, with formal gate reviews between each.", "rationale": "Stage-gated development reduces integration risk."},
        {"text": "The end-to-end test suite shall include SITL regression, HIL regression, and custom hardware integration test campaigns with documented results.", "rationale": "Cross-stage testing confirms performance is maintained across transitions."},
        {"text": "A quantitative performance comparison (PID baseline vs. RL) shall be generated across all stages (SITL, HIL, hardware).", "rationale": "Cross-stage comparison is the primary evidence of RL benefit."},
        {"text": "All technical documentation, user manuals, operating procedures, and design rationale shall be completed and archived in version control before project closure.", "rationale": "Complete documentation enables future maintenance and knowledge transfer."},
        {"text": "The total system shall complete integration of 132 work packages across 6 phases within an estimated 42-53 weeks and ~1,304 person-hours.", "rationale": "Budget and schedule constraints scope overall project effort."},
        {"text": "Each stage gate shall produce a documented go/no-go decision with exit criteria checklist, entry criteria checklist, and formal authorization.", "rationale": "Formal gates prevent premature advancement."},
        {"text": "The system shall use ROS 2 Jazzy as the middleware for inter-process communication in SITL and HIL configurations.", "rationale": "ROS 2 standardizes message interfaces and enables community plugin reuse."},
        {"text": "CPU load shall be monitored in flight; load >80% shall generate a WARNING and >95% shall generate a CRITICAL alert.", "rationale": "CPU overload can cause missed control deadlines."},
        {"text": "RAM usage shall be monitored in flight; usage >90% shall generate a WARNING and >98% shall generate a CRITICAL alert.", "rationale": "Memory exhaustion causes unpredictable firmware behavior."},
    ],
    "SIM": [
        {"text": "The SITL simulation environment shall execute at >=1x real-time speed on the development workstation.", "rationale": "Real-time simulation is required for meaningful validation and RL training."},
        {"text": "The Gazebo simulator shall render the UAV environment at >=30 FPS with GUI enabled.", "rationale": "Interactive frame rates are needed for development debugging."},
        {"text": "The physics simulation shall use a timestep of <=0.004 s (>=250 Hz).", "rationale": "Small timesteps prevent numerical instability in rigid-body dynamics."},
        {"text": "The simulation shall sustain stable physics execution for >=10 minutes without drift or crash.", "rationale": "Extended runs are required for controller verification and RL episodes."},
        {"text": "The UAV dynamics model shall implement full 6-DOF rigid-body equations of motion (3 translational + 3 rotational).", "rationale": "Complete dynamics capture is necessary for sim-to-real transfer."},
        {"text": "The motor model shall simulate first-order thrust response with a time constant of 50-100 ms.", "rationale": "Realistic motor lag is critical for RL policy robustness."},
        {"text": "The UAV model shall have a total mass of approximately 1 kg with documented inertia tensor.", "rationale": "Representative mass/inertia is required for policy transfer to hardware."},
        {"text": "The aerodynamic model shall produce drag forces that oppose vehicle motion and produce a realistic terminal velocity.", "rationale": "Drag prevents unrealistic acceleration during RL training."},
        {"text": "The IMU sensor model shall publish accelerometer and gyroscope data at >=100 Hz.", "rationale": "100 Hz IMU data supports 100 Hz control loops."},
        {"text": "The simulated IMU shall inject white noise with statistics matching MEMS IMU specifications (e.g., MPU-6050).", "rationale": "Realistic sensor noise ensures RL policies develop noise-tolerant behaviors."},
        {"text": "The sensor coordinate frames shall be defined in the URDF/SDF model and published as a correct TF tree.", "rationale": "Consistent frames prevent sign/axis errors in state estimation."},
        {"text": "The simulation world shall include at least 2 distinct world files (e.g., indoor flat, outdoor with obstacles).", "rationale": "Multiple environments are needed for generalization testing."},
        {"text": "The PID baseline controller shall achieve stable hover for >=60 seconds in simulation.", "rationale": "60 s hover is the primary Phase 2 exit criterion."},
        {"text": "The PID controller shall maintain position accuracy within +/-0.5 m during hover.", "rationale": "Position accuracy establishes the bar for RL policy performance."},
        {"text": "The PID controller shall maintain attitude accuracy within +/-5 degrees during hover.", "rationale": "Attitude accuracy bounds ensure stable flight without excessive oscillation."},
        {"text": "The PID altitude controller shall hold altitude within +/-0.3 m and settle within 3-5 s of a setpoint change.", "rationale": "Tight altitude control is the foundation for safe position-hold."},
        {"text": "The PID controller shall exhibit overshoot of <30% in step response for all controlled axes.", "rationale": "Excessive overshoot indicates poor tuning and risks instability."},
        {"text": "The attitude control loop shall execute at >=100 Hz.", "rationale": "Fast inner-loop rate is required for adequate phase margin."},
        {"text": "The SITL verification suite shall include >=10 defined test scenarios that are automated and repeatable.", "rationale": "Comprehensive automated testing ensures regression detection."},
        {"text": "The SITL test campaign shall achieve >=90% requirement coverage as documented in the verification matrix.", "rationale": "High coverage ensures no critical requirements are untested."},
        {"text": "Disturbance rejection tests shall demonstrate controller recovery without instability following applied perturbations.", "rationale": "Robustness validates viability for real-world wind and turbulence."},
    ],
    "RL": [
        {"text": "The RL observation space shall include at minimum: position (3), velocity (3), attitude (3), angular rates (3), and goal position (3), totaling >=15 dimensions.", "rationale": "Complete state observability is necessary for stable control."},
        {"text": "All observation inputs shall be normalized to approximately [-1, 1] or standard-normal range using collected training statistics.", "rationale": "Input normalization is essential for stable training convergence."},
        {"text": "The action space shall output 4 motor commands bounded in the range [-1, 1] via Tanh activation.", "rationale": "Bounded outputs prevent actuator saturation."},
        {"text": "The reward function shall encourage goal-seeking behavior and penalize crashes, with tunable weighting parameters.", "rationale": "Proper reward shaping guides RL policy behavior."},
        {"text": "Episode termination shall trigger on crash, success condition, or a maximum-time timeout.", "rationale": "Termination conditions prevent runaway simulation."},
        {"text": "Domain randomization shall vary at minimum: mass, inertia, motor gains, and sensor noise levels during training.", "rationale": "Randomization narrows the sim-to-real gap."},
        {"text": "The actor neural network shall be a feed-forward MLP with 2 hidden layers of 128 neurons each using ReLU activation.", "rationale": "Architecture provides sufficient power within embedded memory constraints."},
        {"text": "The deployed actor network shall contain <=19,076 parameters (~19 K INT8 or ~76 KB FP32).", "rationale": "Parameter budget is set by MCU available RAM."},
        {"text": "The critic (value-function) head shall be used during training only and shall NOT be deployed on the MCU.", "rationale": "Excluding the critic saves memory on the embedded target."},
        {"text": "The RL framework selection process shall evaluate >=3 candidate frameworks with a documented trade study.", "rationale": "Rigorous selection ensures the chosen framework meets requirements."},
        {"text": "The RL training pipeline shall execute >=3 training iterations with performance analysis between each.", "rationale": "Iterative refinement tunes hyperparameters and diagnoses issues."},
        {"text": "The trained RL policy shall achieve stable hover for >=60 seconds in simulation.", "rationale": "Matching PID baseline is the minimum bar for acceptance."},
        {"text": "The RL policy shall meet or exceed the PID baseline on at least tracking error, settling time, and control effort metrics.", "rationale": "Demonstrating RL advantage justifies the core innovation."},
        {"text": "The RL policy evaluation shall test generalization across unseen mass variations, wind disturbances, and sensor noise levels.", "rationale": "Generalization validates robustness for real-world deployment."},
        {"text": "Final convergence validation shall confirm that the training reward curve has plateaued and performance is stable across >=3 random seeds.", "rationale": "Seed-level reproducibility confirms the result is not an artifact."},
        {"text": "The compressed RL policy shall occupy <1 MB of storage, targeting ~24 KB (INT8 weights + activation buffers).", "rationale": "MCU has limited Flash/RAM."},
        {"text": "Post-quantization (INT8) performance degradation shall be <5% relative to FP32 on all key metrics.", "rationale": "Excessive accuracy loss negates the trained policy's benefits."},
        {"text": "The compressed policy shall be exported in an embedded-friendly format (ONNX, TFLite, or custom C array).", "rationale": "Standard export formats enable optimized inference libraries on MCU."},
    ],
    "FW": [
        {"text": "The selected MCU shall have a CPU clock >=168 MHz, RAM >=256 KB, Flash >=512 KB, and a hardware FPU.", "rationale": "Minimum compute resources for real-time RL inference."},
        {"text": "The firmware shall run on a real-time operating system (RTOS), with FreeRTOS as the recommended choice.", "rationale": "RTOS provides deterministic scheduling for hard-real-time control."},
        {"text": "The firmware shall be organized into isolated modules: sensor, control, communication, and safety.", "rationale": "Modular architecture enables independent testing."},
        {"text": "The firmware shall use static memory allocation for all real-time tasks; dynamic heap usage shall be minimized.", "rationale": "Static allocation eliminates non-deterministic latency and fragmentation."},
        {"text": "RL policy inference (forward pass) shall complete in <5 ms on the target MCU using INT8 quantization, targeting ~3 ms.", "rationale": "5 ms inference fits within a 10 ms control period for 100 Hz."},
        {"text": "The complete control loop (sensor read, inference, motor output) shall execute in <10 ms (>=100 Hz).", "rationale": "100 Hz is the minimum rate for adequate attitude stabilization."},
        {"text": "Control loop timing jitter shall be <1 ms.", "rationale": "Low jitter is critical for deterministic control performance."},
        {"text": "End-to-end latency from sensor-to-actuator shall be quantified, documented, and acceptable for stable flight.", "rationale": "Latency directly affects control phase margin."},
        {"text": "The embedded RL inference output shall match the PC-based FP32 inference output within a documented acceptable tolerance.", "rationale": "Numerical divergence could cause unsafe behavior."},
        {"text": "The HIL communication protocol shall use a simple binary encoding with error detection (e.g., CRC) over UART or USB.", "rationale": "Minimal overhead and error detection ensure reliable real-time exchange."},
        {"text": "The HIL interface shall support bidirectional exchange of sensor data (Gazebo to MCU) and control commands (MCU to Gazebo).", "rationale": "Bidirectional communication is fundamental for HIL simulation."},
        {"text": "The firmware communication driver shall implement buffering, timeout handling, and error recovery.", "rationale": "Robustness to transient failures prevents HIL test interruptions."},
    ],
    "HW": [
        {"text": "The custom flight controller PCB shall provide regulated 5 V and 3.3 V power rails with voltage ripple <50 mV.", "rationale": "Clean power rails are required for accurate sensor readings."},
        {"text": "The PCB shall use a >=4-layer stackup with dedicated power and ground planes.", "rationale": "Proper layer stackup reduces EMI and improves signal integrity."},
        {"text": "The PCB shall provide I2C and SPI interfaces for sensor connectivity, UART/USB for communication, and >=4 PWM outputs for motor ESCs.", "rationale": "Covers all required sensor, communication, and actuator connections."},
        {"text": "The hardware requirements shall define operating temperature range, vibration, and moisture tolerance appropriate for UAV flight.", "rationale": "Environmental requirements ensure reliability during outdoor flight."},
        {"text": "The PCB design shall pass all Design Rule Checks (DRC) with zero violations prior to fabrication release.", "rationale": "DRC compliance prevents manufacturing defects."},
        {"text": "All power rails shall measure within specification and the MCU shall boot and communicate before the board is accepted.", "rationale": "Hardware bring-up validation confirms PCB functionality."},
        {"text": "The Bill of Materials shall identify all components with manufacturer part numbers, and all long-lead items shall have backup alternates sourced.", "rationale": "Supply chain readiness prevents schedule delays."},
    ],
    "SAF": [
        {"text": "The Safety Monitor task shall execute at 100 Hz as the highest-priority RTOS task.", "rationale": "High-frequency monitoring ensures rapid fault detection."},
        {"text": "A hardware watchdog timer with a 50 ms timeout shall be implemented; failure to reset shall trigger the EMERGENCY state.", "rationale": "Hardware watchdog protects against firmware hang or deadlock."},
        {"text": "IMU data staleness shall be detected if no valid reading is received within 1 ms; the fault shall be reported to the Safety Monitor.", "rationale": "IMU is the primary stability sensor."},
        {"text": "Barometer data timeout shall be detected at >20 ms; GPS no-fix timeout shall be detected at >30 s.", "rationale": "Tiered timeouts match characteristic update rates."},
        {"text": "GPS quality shall be flagged as degraded when HDOP >5.0 or satellite count <4.", "rationale": "Poor GPS geometry makes position estimates unreliable."},
        {"text": "Magnetometer data timeout shall be detected at >15 ms, and magnetic interference shall be flagged.", "rationale": "Magnetometer faults corrupt heading estimation."},
        {"text": "The system shall enforce a hard altitude limit of 100 m AGL.", "rationale": "Margin below FAA 400 ft (122 m) regulatory ceiling."},
        {"text": "The system shall enforce a maximum tilt angle of +/-45 degrees (roll and pitch).", "rationale": "Excessive tilt risks unrecoverable loss of altitude."},
        {"text": "The system shall enforce a maximum horizontal velocity of 15 m/s and a maximum vertical velocity of 5 m/s.", "rationale": "Velocity limits bound kinetic energy."},
        {"text": "A geofence of +/-500 m radius from the home point shall be enforced; breach shall trigger safety state escalation.", "rationale": "Geofencing prevents flyaway."},
        {"text": "An RL inference watchdog with a 5 ms timeout shall monitor each inference cycle; timeout shall trigger automatic fallback to the PID controller.", "rationale": "Guarantees stable flight if RL inference stalls."},
        {"text": "A PID fallback controller shall be loaded and ready for immediate activation at all times during RL-mode flight.", "rationale": "PID provides a proven-safe control path if RL fails."},
        {"text": "Motor mixer saturation exceeding 90% shall generate a WARNING-level alert.", "rationale": "Near-saturation indicates loss of control authority."},
        {"text": "ESC heartbeat loss shall be detected and reported to the Safety Monitor.", "rationale": "Non-responsive ESC means loss of thrust."},
        {"text": "Battery voltage shall be monitored at 10 Hz; voltage <3.5 V/cell shall trigger RTL, <3.3 V/cell shall trigger emergency landing, and <3.0 V/cell shall trigger emergency shutdown.", "rationale": "Tiered voltage thresholds provide progressive response."},
        {"text": "Total motor current shall be limited to a maximum of 60 A.", "rationale": "Current limiting prevents motor burnout and over-discharge."},
        {"text": "Board temperature shall be monitored at 1 Hz; temperature >70C shall trigger power reduction, and >85C shall trigger emergency shutdown.", "rationale": "Thermal protection prevents component damage."},
        {"text": "The flight safety system shall implement a 5-state FSM: NORMAL, WARNING, CRITICAL, EMERGENCY, SHUTDOWN.", "rationale": "Structured state machine ensures deterministic fault response."},
        {"text": "Transition from EMERGENCY to SHUTDOWN (motor kill) shall complete in <1 ms.", "rationale": "Sub-millisecond kill latency prevents hazardous motor operation."},
        {"text": "Emergency landing shall execute a vertical descent at 1 m/s with touchdown detection (accelerometer) and automatic disarm on ground contact.", "rationale": "Controlled descent minimizes impact energy."},
        {"text": "The following events shall trigger immediate EMERGENCY state: manual kill switch, attitude error >45 degrees, IMU failure, battery <3.0 V/cell, thermal >85C.", "rationale": "These represent immediately dangerous conditions."},
        {"text": "RC signal loss >5 s shall escalate to CRITICAL state; telemetry loss >30 s shall escalate to WARNING state.", "rationale": "Different timeouts reflect criticality of each link."},
        {"text": "RL inference timeout >10 ms shall trigger a WARNING and automatic switch to the PID fallback controller.", "rationale": "System-level timeout allows for occasional latency spikes."},
        {"text": "The system shall block arming if sensor calibration is invalid, GPS fix is not acquired (in outdoor mode), or battery is <3.7 V/cell.", "rationale": "Pre-flight checks prevent launch in unsafe configuration."},
        {"text": "If RL neural network weights are not loaded, the system shall warn the operator and allow arming in PID-only mode.", "rationale": "Graceful degradation allows flight without a trained RL policy."},
        {"text": "A black-box flight data logger shall record sensor data, control signals, fault events, and battery telemetry to SD card at 100 Hz.", "rationale": "Comprehensive logging enables post-flight analysis."},
        {"text": "Crash detection shall trigger on accelerometer readings >4 g or attitude exceeding +/-90 degrees (flip detection), followed by immediate disarm and log dump.", "rationale": "Rapid crash response and data preservation for root-cause analysis."},
        {"text": "Fault codes shall be indicated via LED blink patterns (1-9 blinks) and buzzer audio alerts.", "rationale": "Visual and audible indicators provide operator awareness."},
    ],
}


def write_doorstop_yml(path: Path, prefix: str, parent: str | None, digits: int, sep: str) -> None:
    """Write a .doorstop.yml configuration file."""
    lines = [
        "settings:",
        f"  digits: {digits}",
        f"  prefix: {prefix}",
        f"  sep: '{sep}'",
    ]
    if parent:
        lines.append(f"  parent: {parent}")
    path.mkdir(parents=True, exist_ok=True)
    (path / ".doorstop.yml").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_requirement(path: Path, prefix: str, number: int, text: str, rationale: str, parent_links: list[str] | None = None) -> None:
    """Write a single Doorstop requirement YAML file."""
    uid = f"{prefix}{number:03d}"
    links_str = ""
    if parent_links:
        links_str = "\n".join(f"- {link}: null" for link in parent_links)
        links_str = f"\n{links_str}"
    else:
        links_str = " []"

    content = f"""active: true
derived: false
header: ''
level: 1.{number}
links:{links_str}
normative: true
ref: ''
reviewed: null
text: |
  {text}
"""
    (path / f"{uid}.yml").write_text(content, encoding="utf-8")
    return uid


def main():
    print(f"Creating Doorstop requirements tree in {REQS}")

    # Create document configs
    for prefix, cfg in DOCUMENTS.items():
        doc_path = REQS / prefix
        write_doorstop_yml(doc_path, prefix, cfg["parent"], cfg["digits"], cfg["sep"])
        print(f"  Created document: {prefix} (parent={cfg['parent']})")

    # Create requirements
    total = 0
    for prefix, reqs in REQUIREMENTS.items():
        doc_path = REQS / prefix
        for i, req in enumerate(reqs, start=1):
            # Link child reqs to parent SYS-001 (stage-gate architecture)
            links = []
            if prefix != "SYS" and i == 1:
                links = ["SYS001"]
            write_requirement(doc_path, prefix, i, req["text"], req["rationale"], links or None)
            total += 1

    print(f"\nCreated {total} requirements across {len(DOCUMENTS)} documents.")
    print("Run 'doorstop publish all docs/requirements/' to generate HTML output.")


if __name__ == "__main__":
    main()
