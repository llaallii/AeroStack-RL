# AeroStack-RL Program Plan

## 1. System Architecture

### 1.1 High-Level Architecture
The system follows a Companion Computer paradigm where the **STM32MP257F-DK** acts as the high-level brain, connected to a dedicated Flight Control Unit (FCU).

*   **Compute Module**: STM32MP257F-DK
    *   **Cortex-A35 x2 (Linux)**: Runs ROS 2 (Jazzy/Humble), RL Inference, Path Planning, Logging.
    *   **Cortex-M33 (Real-Time)**: *Optional* - Can be used for hard real-time IO expansion or Safety Watchdog monitoring if FCU IO is insufficient.
*   **Flight Controller (FCU)**: Standard Pixhawk-class or STM32-based FCU running PX4/ArduPilot. Handles attitude control, motor mixing, and failsafe.
*   **Interconnects**:
    *   **UART/Serial**: MAVLink bridge between STM32MP2 (Linux) and FCU.
    *   **OpenAMP (RPMsg)**: *Internal* IPC between Cortex-A and Cortex-M on the MP2 (if utilized).

### 1.2 Software Stack (Linux Side)
*   **OS**: Custom Yocto or Minimal Ubuntu image (optimized for boot time).
*   **Middleware**: ROS 2 (DDS FastRTPS/CycloneDDS configured for shm).
*   **Key Nodes**:
    *   `mavros` / `uxrce-dds`: Bridge to FCU.
    *   `aerostack_supervisor`: State machine, health monitoring.
    *   `rl_agent`: PyTorch/ONNX runtime for policy inference.
    *   `sensor_drivers`: Nodes for attached peripherals (Lidar, Cameras).

---

## 2. Detailed Implementation Plan (WBS)

### Phase 0: Platform Foundation (BSP & OS)
**Goal**: Stable Linux environment with basic connectivity and real-time tweaks.

| ID | Task | Deliverable | Priority |
| :--- | :--- | :--- | :--- |
| **T-0.1** | **Setup STM32MP2 Dev Environment**<br>Install SDK, Flash tools, Serial console access. | Working Build Host | M |
| **T-0.2** | **Build Minimal Linux Image**<br>Yocto/Buildroot config. Strip GUI. Enable PREEMPT_RT. | Bootable Image | M |
| **T-0.3** | **Configure Connectivity**<br>Static IP, network config. | Network Config Script | M |
| **T-0.4** | **Hardware Interfaces & Watchdog**<br>Device Tree for UARTs/I2C/SPI. Enable HW Watchdog (REQ-SAFE-1). | DTB / Overlays | M |
| **T-0.5** | **Boot & Shutdown Optimization**<br>Optimize boot time (REQ-1.1), configure safe shutdown (REQ-PWR-1). | Boot Trace Log | M |

### Phase 1: ROS 2 Architecture & FCU Link
**Goal**: ROS 2 running and talking to the Flight Controller with sync.

| ID | Task | Deliverable | Priority |
| :--- | :--- | :--- | :--- |
| **T-1.1** | **Install ROS 2 Base**<br>Cross-compile arm64 binaries. | ROS 2 Environment | M |
| **T-1.2** | **Deploy Bridge & Time Sync**<br>MAVROS/MicroXRCE-DDS + Timesync Setup (REQ-TIME-1). | Communication + Sync | M |
| **T-1.3** | **Hardware Link Tuning (UART)**<br>Tune for 50Hz+, low drop (REQ-2.1). Debug port setup (REQ-3.3). | `mavlink_inspector` output | M |
| **T-1.4** | **Create Workspace**<br>Colcon structure. | Git Repo | M |

### Phase 2: Sensor Integration & Logging
**Goal**: Trusted data flow and managed storage.

| ID | Task | Deliverable | Priority |
| :--- | :--- | :--- | :--- |
| **T-2.1** | **Integrate Sensors**<br>Drivers for Cam/Lidar. | Validated Topics | M |
| **T-2.2** | **Health & Failsafe Node**<br>Monitor system status. Implement Loss of ROS > 1s logic (REQ-SAFE-3). | `aerostack_health` pkg | M |
| **T-2.3** | **Logging & Storage Mgmt**<br>Rosbag config + Auto-rotate script (REQ-DATA-1). | recording script | M |

### Phase 3: Simulation-in-the-Loop (SITL)
**Goal**: Develop and test without risking hardware.

| ID | Task | Deliverable | Priority |
| :--- | :--- | :--- | :--- |
| **T-3.1** | **Setup Gazebo SITL**<br>PX4 SITL + Gazebo. | Sim Container | M |
| **T-3.2** | **Sim-Real Parity**<br>Ensure interfaces match. | Parity Report | M |
| **T-3.3** | **Disturbance Injection**<br>Wind/Noise models. | Benchmarking Scenarios | S |

### Phase 4: RL Infrastructure & Autonomy
**Goal**: The core "AeroStack-RL" capability.

| ID | Task | Deliverable | Priority |
| :--- | :--- | :--- | :--- |
| **T-4.1** | **Gym-ROS Interface**<br>OpenAI Gym wrapper. | `aerostack_gym` pkg | M |
| **T-4.2** | **Policy Loader & Perf**<br>Runtime loader. Optmize for Memory < 512MB (REQ-PERF-2). | `policy_runner` node | M |
| **T-4.3** | **Safety & Override**<br>Manual Override < 100ms logic (REQ-SAFE-2). | Safety Check Logic | M |
| **T-4.4** | **Latency & CPU Profiling**<br>Measure End-to-End < 20ms (REQ-7.1), CPU budget (REQ-PERF-1). | Performance Report | M |

### Phase 5: Verification & Full System Test
**Goal**: Flight readiness.

| ID | Task | Deliverable | Priority |
| :--- | :--- | :--- | :--- |
| **T-5.1** | **Bench & Stability Test**<br>Long-duration run. | Reliability Report | M |
| **T-5.2** | **Sim-to-Real Validation**<br>Confirm policy transfer. | Validation Report | M |
| **T-5.3** | **Documentation**<br>Manuals and Configs. | Final Docs | M |

---

## 3. Verification & Validation Plan (V&V)

### 3.1 Test Cases Mapping

| Req ID | Test Case ID | Procedure Summary | Pass Criteria |
| :--- | :--- | :--- | :--- |
| **REQ-1.1** | **TC-BOOT-01** | Cold boot 10x with script. | 95th percentile < 45s. |
| **REQ-2.1** | **TC-COM-01** | Profiler script on UART topic. | Rate ≥ 50Hz, Loss < 0.1%. |
| **REQ-3.3** | **TC-UART-01** | 30-min stress test @ 921600 baud. | 0 dropped frames. |
| **REQ-7.1** | **TC-LAT-01** | `ros2_tracing` end-to-end trace. | Latency ≤ 20ms, Jitter ≤ 5ms. |
| **REQ-SAFE-1** | **TC-WD-01** | Inject Kernel Panic (`echo c > /proc/sysrq-trigger`). | Reboot < 2s. |
| **REQ-PWR-1** | **TC-PWR-01** | Trigger shutdown. Check FS clean bit. | No corruption. |
| **REQ-TIME-1** | **TC-SYNC-01** | Check offset between MAVLink & System time. | Drift ≤ 2ms/min. |
| **REQ-SAFE-2** | **TC-MAN-01** | Toggle RC Switch, measure PWM change. | Delay < 100ms. |
| **REQ-SAFE-3** | **TC-FAIL-01** | Kill ROS Process. Monitor FCU mode. | FCU -> Stabilized in < 1s. |
| **REQ-PERF-1** | **TC-CPU-01** | Run full stack + stress. Monitor `htop`. | Load ≤ 70%. |
| **REQ-PERF-2** | **TC-MEM-01** | Load largest RL policy. Check process RAM. | RSS < 512MB. |

---

## 4. Phase MVPs & Definitions

*   **P0-MVP (Board Alive)**: Linux boots <45s, Watchdog active, SSH ready.
*   **P1-MVP (ROS Connected)**: ROS 2 talks to FCU @ 50Hz+, Time synced.
*   **P2-MVP (Data Valid)**: Sensors valid, Logging handles storage limits.
*   **P3-MVP (Sim Twin)**: Sim matches Real interfaces; Disturbance tested.
*   **P4-MVP (Smart Pilot)**: RL Policy runs within CPU/RAM limits; Latency < 20ms.
*   **P5-MVP (Flight Ready)**: All Safety TCs pass; 2h stability verified.

---

## 5. Traceability Matrix

| Requirement ID | Implemented By (Task) | Verified By (Test Case) |
| :--- | :--- | :--- |
| REQ-1.1 (Boot Time) | T-0.5, T-0.2 | TC-BOOT-01 |
| REQ-1.2 (Boot Chain) | T-0.2, T-0.1 | Health Check Script |
| REQ-1.3 (Config) | T-0.3, T-2.4 | Config Audit |
| REQ-PWR-1 (Safe Shutdown) | T-0.5 | TC-PWR-01 |
| REQ-TIME-1 (Sync) | T-1.2, T-2.2 | TC-SYNC-01 |
| REQ-2.1 (Connectivity) | T-1.3 | TC-COM-01 |
| REQ-2.2 (Pub/Sub) | T-1.2, T-2.1 | Rosbag Validation |
| REQ-2.3 (Modularity) | T-4.2 | Service Test |
| REQ-3.1 (Sensors) | T-2.1 | Data Analysis |
| REQ-3.2 (Actuators) | T-1.3 | Actuator Log |
| REQ-3.3 (Debug UART) | T-1.3 | TC-UART-01 |
| REQ-4.1 (Telemetry) | T-2.4 | Log Review |
| REQ-4.2 (Replay) | T-2.4 | Replay Test |
| REQ-4.3 (Faults) | T-2.2 | Fault Injection |
| REQ-DATA-1 (Storage) | T-2.3 | Storage Stress Test |
| REQ-5.1 (Nav Interface) | T-4.1 | Interface Check |
| REQ-5.2 (Policy Swap) | T-4.2 | TC-RL-01 |
| REQ-5.3 (Metrics) | T-4.4 | Metric Log |
| REQ-6.1 (Sim Run) | T-3.1 | Sim Flight |
| REQ-6.2 (Sim Switch) | T-3.2 | Launch Diff |
| REQ-6.3 (Disturbances) | T-3.3 | Sim Benchmarking |
| REQ-7.1 (Latency) | T-4.4 | TC-LAT-01 |
| REQ-7.2 (Startup) | T-0.5 | Boot Trace |
| REQ-7.3 (Load) | T-5.1 | Stress Test |
| REQ-PERF-1 (CPU) | T-4.4 | TC-CPU-01 |
| REQ-PERF-2 (Memory) | T-4.2 | TC-MEM-01 |
| REQ-PERF-3 (A/B) | T-0.2 | Update Test |
| REQ-8.1 (Kill) | T-4.3 | TC-SAFE-01 |
| REQ-8.2 (Manual Fallback) | T-4.3 | TC-MAN-01 |
| REQ-8.3 (Geofence) | T-4.3 | Geofence Test |
| REQ-SAFE-1 (Watchdog) | T-0.4 | TC-WD-01 |
| REQ-SAFE-2 (Man Latency) | T-4.3 | TC-MAN-01 |
| REQ-SAFE-3 (Failsafe) | T-2.2 | TC-FAIL-01 |
| REQ-9.1 (Docs) | T-5.3 | Doc Review |
| REQ-9.2 (Dash) | T-2.4 | Dashboard Demo |
| REQ-9.3 (Configs) | T-1.4 | Repo Audit |
| REQ-10.1 (Sensors) | T-2.1 | Integration Test |
| REQ-10.2 (Portability) | T-0.1 | Build Test |
| REQ-10.3 (Missions) | T-4.2 | Mission Test |

---

## 6. Risk Register

| Risk | Likelihood | Impact | Mitigation Strategy |
| :--- | :--- | :--- | :--- |
| **R1: Boot Time > 45s** | Medium | Low | `systemd-analyze` to prune, static IP, preempt-rt. |
| **R2: UART Jitter** | Medium | High | DMA drivers, FIFO buffers, move to SPI. |
| **R3: CPU/Thermal Throttling** | High | High | Heatsink, Model Quantization (INT8), Fan. |
| **R4: Sim-to-Real Divergence** | High | High | System ID for physics tuning, conservative policies. |
| **R5: SD Card Corruption** | Medium | High | Read-Only RootFS, separate data partition, industrial SD. |
