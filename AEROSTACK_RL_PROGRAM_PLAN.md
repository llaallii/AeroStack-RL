# AeroStack-RL Program Plan

## 1. Requirements Review & Analysis

### 1.1 Acceptance Criteria Refinement
Review of `URS.md` identified the need for more specific, quantifiable acceptance criteria for "Must" requirements.

| ID | Original Requirement | Refined Acceptance Criteria / Metric |
| :--- | :--- | :--- |
| **REQ-1.1** | ROS ready state after power-on | System boots to `ros2 node system_check` ok status in **< 45 seconds** from power application. |
| **REQ-2.1** | Connect embedded nodes | FCU (via UART) @ >50Hz, Lidar/Camera (via USB/CSI) @ Target FPS with **< 10ms transport latency**. |
| **REQ-3.3** | UART debugging | Dedicated debug port exposed; `minicom` session captures boot log + crash dumps without dropping frames @ 115200 baud. |
| **REQ-7.1** | Measure loop latency | End-to-End (Sensor -> ROS -> Actuator) latency is **deterministic and < 20ms** (or compatible with flight control loop). |

### 1.2 Missing / Implicit Requirements
| ID | Requirement | Priority | Rationale |
| :--- | :--- | :--- | :--- |
| **REQ-SAFE-1** | System Watchdog | **M** | Hardware watchdog must reset the STM32MP2 if the Linux kernel hangs. |
| **REQ-DATA-1** | Log Storage Management | **S** | System must auto-rotate or stop logging when disk usage > 90% to prevent corruption. |
| **REQ-PWR-1** | Safe Shutdown | **M** | Soft shutdown command must sync disks before cutting power (prevent FS corruption). |

---

## 2. System Architecture Proposal

### 2.1 High-Level Architecture
The system follows a Companion Computer paradigm where the **STM32MP257F-DK** acts as the high-level brain, connected to a dedicated Flight Control Unit (FCU).

*   **Compute Module**: STM32MP257F-DK
    *   **Cortex-A35 x2 (Linux)**: Runs ROS 2 (Jazzy/Humble), RL Inference, Path Planning, Logging.
    *   **Cortex-M33 (Real-Time)**: *Optional* - Can be used for hard real-time IO expansion or Safety Watchdog monitoring if FCU IO is insufficient.
*   **Flight Controller (FCU)**: Standard Pixhawk-class or STM32-based FCU running PX4/ArduPilot. Handles attitude control, motor mixing, and failsafe.
*   **Interconnects**:
    *   **UART/Serial**: MAVLink bridge between STM32MP2 (Linux) and FCU.
    *   **OpenAMP (RPMsg)**: *Internal* IPC between Cortex-A and Cortex-M on the MP2 (if utilized).

### 2.2 Software Stack (Linux Side)
*   **OS**: Custom Yocto or Minimal Ubuntu image (optimized for boot time).
*   **Middleware**: ROS 2 (DDS FastRTPS/CycloneDDS configured for shm).
*   **Key Nodes**:
    *   `mavros` / `uxrce-dds`: Bridge to FCU.
    *   `aerostack_supervisor`: State machine, health monitoring.
    *   `rl_agent`: PyTorch/ONNX runtime for policy inference.
    *   `sensor_drivers`: Nodes for attached peripherals (Lidar, Cameras).

---

## 3. Detailed Implementation Plan (WBS)

### Phase 0: Platform Foundation (BSP & OS)
**Goal**: Stable Linux environment with basic connectivity and real-time tweaks.

| ID | Task | Deliverable | Priority |
| :--- | :--- | :--- | :--- |
| **T-0.1** | **Setup STM32MP2 Dev Environment**<br>Install SDK, Flash tools, Serial console access. | Working Build Host | M |
| **T-0.2** | **Build Minimal Linux Image**<br>Yocto/Buildroot config. Strip GUI. Enable PREEMPT_RT if possible. | Bootable Image | M |
| **T-0.3** | **Configure Connectivity**<br>Static IP on eth0, WiFi setup (if applicable), SSH keys. | Network Config Script | M |
| **T-0.4** | **Enable Hardware Interfaces**<br>Device Tree overlay for UARTs, I2C, SPI. Verify /dev/tty* presence. | DTB / Overlays | M |
| **T-0.5** | **Boot Time Optimization (Sub-phase)**<br>Disable unused services (systemd-analyze). | Boot Trace Log (<45s target) | S |

### Phase 1: ROS 2 Architecture & FCU Link
**Goal**: ROS 2 running and talking to the Flight Controller.

| ID | Task | Deliverable | Priority |
| :--- | :--- | :--- | :--- |
| **T-1.1** | **Install ROS 2 Base**<br>Cross-compile or install arm64 binaries. Verify `ros2 run demo_nodes_cpp talker`. | ROS 2 Environment | M |
| **T-1.2** | **Deploy MAVROS/MicroXRCE-DDS**<br>Bridge node setup. | Communication with FCU | M |
| **T-1.3** | **Establish Hardware Link (UART)**<br>Wiring STM32MP2 UART to FCU TELEM port. Baud rate tuning. | `mavlink_inspector` output | M |
| **T-1.4** | **Create AeroStack Workspace**<br>Colcon workspace structure, core package skeletal. | Git Repo Structure | M |

### Phase 2: Sensor Integration & Logging
**Goal**: Trusted data flowing into ROS topics.

| ID | Task | Deliverable | Priority |
| :--- | :--- | :--- | :--- |
| **T-2.1** | **Integrate Primary Sensors**<br>Driver nodes for specific sensors (Cam, Lidar). Check timestamps. | Validated Topics | M |
| **T-2.2** | **Implement Time Sync**<br>Chrony/NTP or PTP if networked. MAVLink timesync for FCU. | Time Offset Graphs | M |
| **T-2.3** | **Health Monitor Node**<br>Node that subscribes to critical topics and publishes `/system/status`. | `aerostack_health` pkg | M |
| **T-2.4** | **Logging Configuration**<br>Rosbag2 config (compression, split size, storage path). | `launch_record.py` | S |

### Phase 3: Simulation-in-the-Loop (SITL)
**Goal**: Develop and test without risking hardware.

| ID | Task | Deliverable | Priority |
| :--- | :--- | :--- | :--- |
| **T-3.1** | **Setup Gazebo/Sim Container**<br>SITL environment (e.g., PX4 SITL + Gazebo Garden). | Dockerfile / Compose | M |
| **T-3.2** | **Bridge Sim to Hardware Interface**<br>Ensure `rl_agent` sees same topics in Sim as Real. | Interface Abstraction Layer | M |
| **T-3.3** | **Disturbance Injection Module**<br>Add wind/noise models to Gazebo plugins. | Benchmarking Scenarios | S |

### Phase 4: RL Infrastructure & Autonomy
**Goal**: The core "AeroStack-RL" capability.

| ID | Task | Deliverable | Priority |
| :--- | :--- | :--- | :--- |
| **T-4.1** | **Gym-ROS Interface**<br>OpenAI Gym wrapper converting Observations <-> ROS Topics. | `aerostack_gym` pkg | M |
| **T-4.2** | **Policy Loader Node**<br>Python/C++ node to load ONNX/TorchScript models dynamically. | `policy_runner` node | M |
| **T-4.3** | **Safety Failsafe Implementation**<br>Geofence + Manual Override triggers in software. | Safety Check Logic | M |
| **T-4.4** | **Latency Profiling**<br>Instrument code to measure Observation -> Action delay. | Performance Report | M |

### Phase 5: Verification & Full System Test
**Goal**: Flight readiness.

| ID | Task | Deliverable | Priority |
| :--- | :--- | :--- | :--- |
| **T-5.1** | **Bench Test (Hardware)**<br>Long-duration run (2h) logging stability test on desk. | Stability Log | M |
| **T-5.2** | **Sim-to-Real Validation**<br>Run simple policy in Sim, then Real (tethered/safe). Compare logs. | Validation Report | M |
| **T-5.3** | **Documentation Finalization**<br>User manual, setup guide, reproducible scripts. | Final Docs | M |

---

## 4. Verification & Validation Plan (V&V)

### 4.1 Test Cases Mapping

| Req ID | Test Case ID | Procedure Summary | Pass Criteria |
| :--- | :--- | :--- | :--- |
| **REQ-1.1** | **TC-BOOT-01** | Cold boot device, measure time to `ros2 node list` populated. | < 45s, all nodes active. |
| **REQ-2.1** | **TC-COM-01** | Send MAVLink Heartbeat from FCU. Check ROS `/mavros/state`. | Update rate > 0.5Hz, `connected: true`. |
| **REQ-4.1** | **TC-LOG-01** | Record 10min flight bag. Run `ros2 bag info`. | No dropped messages > 1%, defined size. |
| **REQ-8.1** | **TC-SAFE-01** | Trigger Kill Switch (Phys/Soft). Monitor PWM output scope. | Motor signal = Disarmed (1000us/0) in < 200ms. |
| **REQ-5.2** | **TC-RL-01** | Swap Policy A (Hover) to B (Circle) at runtime via Service. | Switch distinct capability < 2s, no crash. |

---

## 5. Phase MVPs & Definitions

*   **P0-MVP (Board Alive)**: Device boots Linux, accessible via SSH, correct Device Tree loaded.
*   **P1-MVP (ROS Connected)**: ROS 2 installed, can echo `/mavros/imu/data` from connected FCU.
*   **P2-MVP (Data Valid)**: All sensors publishing valid data; system logs data correctly to disk.
*   **P3-MVP (Sim Twin)**: User can fly the drone in Simulator using the exact same code launch file as hardware.
*   **P4-MVP (Smart Pilot)**: System can load a neural network policy and output velocity commands based on inputs.
*   **P5-MVP (Flight Ready)**: System passes all reliability stress tests and safety checks; ready for field trials.

---

## 6. Risk Register

| Risk | Likelihood | Impact | Mitigation Strategy |
| :--- | :--- | :--- | :--- |
| **R1: Boot Time Exceeds Specs** | Medium | Low (Operational annoyance) | Use `systemd-analyze` to prune services; Initramfs optimization; Static IP. |
| **R2: UART Jitter/Data Loss** | Medium | High (Control Instability) | Use DMA-enabled UART drivers; Increase FIFO buffers; Move to SPI if UART bottlenecks. |
| **R3: Compute Overload (Heat/Lag)** | High | High (System Freeze) | Passive heatsink upgrade; RL Model Optimization (Quantization INT8). |
| **R4: Sim-to-Real Gap** | High | High (Crash) | System identification to tune Sim physics; Conservative initial policies. |
| **R5: SD Card Corruption** | Medium | High (Data Loss/No Boot) | Use Read-Only RootFS overlay; separate partition for Logs; Industrial grade SD card. |
