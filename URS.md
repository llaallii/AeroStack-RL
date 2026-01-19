# AeroStack-RL

This project builds a ROS-centered UAV compute and autonomy experimentation platform that connects embedded control nodes (flight controller, sensor/actuator interfaces) with high-level onboard computing (Linux + ROS) to enable reliable autonomy and reinforcement-learning research.

Here are **high-level user requirements** for **AeroStack-RL** project.

---

# ✅ AeroStack-RL — High Level User Requirements

### Priority Legend

* **M – Must**: Core functionality required for system success
* **S – Should**: Important but not critical for first release
* **C – Could**: Desirable for future iterations

---

## 1) System Initialization & Configuration

| ID | Requirement | Priority | Acceptance Criteria / Metric | Verification Method |
| --- | --- | --- | --- | --- |
| REQ-1.1 | ROS ready state after power-on | **M** | From power application to successful `ros2 node system_check` = **≤ 45 s (95th percentile)** over 10 cold boots | Boot timestamp log + automated script |
| REQ-1.2 | Verify boot chain health | **M** | Diagnostic tool reports status of bootloader, Linux, and ROS | Output of `health_check.sh` |
| REQ-1.3 | Configurable networking/timing | **M** | Parameters editable via config, no code changes | Manual config test |
| REQ-PWR-1 | Safe Shutdown | **M** | `shutdown_service` syncs FS; power cut allowed only after ACK | Power analyzer trace & FS check |
| REQ-TIME-1 | Time Synchronization | **M** | Clock drift **≤ 2 ms/min** between FCU & ROS | Chrony/PTP log vs FCU time |

---

## 2) ROS Architecture & Connectivity

| ID | Requirement | Priority | Acceptance Criteria / Metric | Verification Method |
| --- | --- | --- | --- | --- |
| REQ-2.1 | Connect embedded nodes | **M** | FCU UART stream **≥ 50 Hz**, packet loss **< 0.1%**; camera/LiDAR transport latency **≤ 10 ms** | `ros2 topic hz` + latency profiler |
| REQ-2.2 | Publish/record key topics | **M** | rosbag captures validated streams | `ros2 bag info` |
| REQ-2.3 | Autonomy modules independent | **S** | Nodes can be launched without reboot | Service restart test |

---

## 3) Sensor & Actuator Integration

| ID | Requirement | Priority | Acceptance Criteria / Metric | Verification Method |
| --- | --- | --- | --- | --- |
| REQ-3.1 | Validate sensor interfaces | **M** | IMU/GPS/range data within expected ranges | Sensor noise analysis script |
| REQ-3.2 | Validate actuator outputs | **M** | Measured response matches command | Motor test bench |
| REQ-3.3 | UART debugging | **M** | Debug port captures continuous logs at 115200–921600 baud with **0 dropped frames** for 30-min stress | `minicom`/`pyserial` capture checksum |

---

## 4) Telemetry, Logging & Diagnostics

| ID | Requirement | Priority | Acceptance Criteria / Metric | Verification Method |
| --- | --- | --- | --- | --- |
| REQ-4.1 | Log telemetry + metrics | **M** | Logs contain state, CPU, latency | Log review |
| REQ-4.2 | Tag/replay experiments | **S** | Run comparison possible | Replay tool test |
| REQ-4.3 | Detect faults | **M** | Alerts generated on failure | Fault injection test |
| REQ-DATA-1 | Log Storage Management | **S** | Auto-rotate at 80%, emergency stop at 90%, **no FS corruption** after 20 cycles | Storage stress test script |

---

## 5) Autonomy & RL Experimentation

| ID | Requirement | Priority | Acceptance Criteria / Metric | Verification Method |
| --- | --- | --- | --- | --- |
| REQ-5.1 | Run classical or RL navigation | **M** | Same interface for both | Interface header check |
| REQ-5.2 | Swap policies runtime | **M** | Plugin-style policy loader | Runtime swap test |
| REQ-5.3 | Track metrics | **S** | Metrics exported per run | Metric file validation |

---

## 6) Simulation-in-the-Loop

| ID | Requirement | Priority | Acceptance Criteria / Metric | Verification Method |
| --- | --- | --- | --- | --- |
| REQ-6.1 | Full stack runs in simulation | **M** | Identical ROS interfaces | SITL run w/ real launch files |
| REQ-6.2 | Switch sim ↔ hardware | **M** | No code modification | Diff check of launch files |
| REQ-6.3 | Inject disturbances | **S** | Noise/delay models applied | Sim data analysis |

---

## 7) Timing & Workload Behavior

| ID | Requirement | Priority | Acceptance Criteria / Metric | Verification Method |
| --- | --- | --- | --- | --- |
| REQ-7.1 | Measure loop latency | **M** | Sensor→ROS→Actuator latency **≤ 20 ms**, jitter **≤ 5 ms** | Timestamp tracing (`ros2_tracing`) |
| REQ-7.2 | Analyze startup effects | **S** | Boot trace available | `systemd-analyze` plot |
| REQ-7.3 | Evaluate under load | **S** | CPU/GPU stress tests | Stress-ng run |
| REQ-PERF-1 | CPU Budget | **M** | Autonomy stack **≤ 70% CPU**, ≥1 core free | `htop` / monitoring log |
| REQ-PERF-2 | Memory Constraints | **M** | RL inference **≤ 512 MB RAM** | Valgrind / `free -m` |
| REQ-PERF-3 | Boot Partition | **S** | A/B update support | FW update test |

---

## 8) Safety Controls

| ID | Requirement | Priority | Acceptance Criteria / Metric | Verification Method |
| --- | --- | --- | --- | --- |
| REQ-8.1 | Arm/disarm & kill | **M** | Immediate motor cutoff | Bench test with scope |
| REQ-8.2 | Fallback to manual | **M** | RC override verified | Manual takeover test |
| REQ-8.3 | Geofencing limits | **S** | Breach triggers RTL | Field/Sim test |
| REQ-SAFE-1 | Hardware Watchdog | **M** | STM32MP2 watchdog triggers reboot within **< 2 s** of kernel hang | Kernel panic injection |
| REQ-SAFE-2 | Manual Override Latency | **M** | RC override within **< 100 ms** from trigger | Latency tester |
| REQ-SAFE-3 | Failsafe Mode | **M** | Loss of ROS for **> 1 s** → FCU takes control | Link severance test |

---

## 9) Usability & Documentation

| ID | Requirement | Priority | Acceptance Criteria / Metric | Verification Method |
| --- | --- | --- | --- | --- |
| REQ-9.1 | Workflow documentation | **M** | Step-by-step guide | Peer review |
| REQ-9.2 | Dashboard | **S** | rqt/foxglove view | UI check |
| REQ-9.3 | Reproducible configs | **M** | Versioned metadata | Config diff check |

---

## 10) Portability & Extensibility

| ID | Requirement | Priority | Acceptance Criteria / Metric | Verification Method |
| --- | --- | --- | --- | --- |
| REQ-10.1 | Extensible sensors | **S** | Standard ROS messages | New sensor bringup |
| REQ-10.2 | Deploy targets | **M** | Build scripts portable | CI cross-build |
| REQ-10.3 | New missions | **S** | Same framework reused | Multi-mission test |
