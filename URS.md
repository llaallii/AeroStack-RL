# AeroStack-RL
This project builds a ROS-centered UAV compute and autonomy experimentation platform that connects embedded control nodes (flight controller, sensor/actuator interfaces) with high-level onboard computing (Linux + ROS) to enable reliable autonomy and reinforcement-learning research. The platform emphasizes repeatable bring-up and timing-aware initialization from boot through ROS readiness, plus robust data logging and diagnostics to capture flight telemetry, compute load, and end-to-end latency for system-level evaluation. It supports simulation-in-the-loop workflows where navigation policies—especially RL-based approaches—can be trained, swapped, and benchmarked under controlled disturbances before deployment, while also providing structured interface validation (including UART-based debugging) to ensure the real hardware stack behaves predictably during experiments.

Here are **high-level user requirements** for **AeroStack-RL** project.

---

# ✅ AeroStack-RL — High Level User Requirements 

### Priority Legend

* **M – Must**: Core functionality required for system success
* **S – Should**: Important but not critical for first release
* **C – Could**: Desirable for future iterations

---

## 1) System Initialization & Configuration

| ID      | Requirement                                                                               | Priority | Acceptance Criteria                                          |
| ------- | ----------------------------------------------------------------------------------------- | -------- | ------------------------------------------------------------ |
| REQ-1.1 | User shall reach a ready-to-fly ROS state after power-on of STM32MP257F-DK based platform | **M**    | ROS master and core nodes active within defined startup time |
| REQ-1.2 | User shall verify boot chain (bootloader → Linux → ROS) via health checks                 | **M**    | Diagnostic tool reports status of each stage                 |
| REQ-1.3 | User shall configure networking, time sync, device mapping without code changes           | **M**    | Parameters editable via config files/UI                      |

---

## 2) ROS Architecture & Connectivity

| ID      | Requirement                                                           | Priority | Acceptance Criteria                       |
| ------- | --------------------------------------------------------------------- | -------- | ----------------------------------------- |
| REQ-2.1 | Connect embedded nodes (FCU, sensors, actuators) to onboard Linux/ROS | **M**    | Data flows through defined ROS interfaces |
| REQ-2.2 | Publish/subscribe/record key ROS topics                               | **M**    | rosbag captures validated streams         |
| REQ-2.3 | Autonomy modules start/stop independently                             | **S**    | Nodes can be launched without reboot      |

---

## 3) Sensor & Actuator Integration

| ID      | Requirement                                                  | Priority | Acceptance Criteria                         |
| ------- | ------------------------------------------------------------ | -------- | ------------------------------------------- |
| REQ-3.1 | Validate sensor interfaces with timestamps and sanity checks | **M**    | IMU/GPS/range data within expected ranges   |
| REQ-3.2 | Validate actuator outputs via test commands                  | **M**    | Measured response matches command           |
| REQ-3.3 | UART debugging for peripherals                               | **M**    | Error frames detectable on STM32MP257F UART |

---

## 4) Telemetry, Logging & Diagnostics

| ID      | Requirement                     | Priority | Acceptance Criteria              |
| ------- | ------------------------------- | -------- | -------------------------------- |
| REQ-4.1 | Log telemetry + compute metrics | **M**    | Logs contain state, CPU, latency |
| REQ-4.2 | Tag/replay experiments          | **S**    | Run comparison possible          |
| REQ-4.3 | Detect faults (dropout, drift)  | **M**    | Alerts generated on failure      |

---

## 5) Autonomy & RL Experimentation

| ID      | Requirement                               | Priority | Acceptance Criteria        |
| ------- | ----------------------------------------- | -------- | -------------------------- |
| REQ-5.1 | Run classical or RL navigation            | **M**    | Same interface for both    |
| REQ-5.2 | Swap policies without stack changes       | **M**    | Plugin-style policy loader |
| REQ-5.3 | Track metrics (success, collisions, time) | **S**    | Metrics exported per run   |

---

## 6) Simulation-in-the-Loop

| ID      | Requirement                      | Priority | Acceptance Criteria        |
| ------- | -------------------------------- | -------- | -------------------------- |
| REQ-6.1 | Full stack runs in simulation    | **M**    | Identical ROS interfaces   |
| REQ-6.2 | Switch sim ↔ hardware via config | **M**    | No code modification       |
| REQ-6.3 | Inject disturbances              | **S**    | Noise/delay models applied |

---

## 7) Timing & Workload Behavior

| ID      | Requirement             | Priority | Acceptance Criteria       |
| ------- | ----------------------- | -------- | ------------------------- |
| REQ-7.1 | Measure loop latency    | **M**    | End-to-end delay profiled |
| REQ-7.2 | Analyze startup effects | **S**    | Boot trace available      |
| REQ-7.3 | Evaluate under load     | **S**    | CPU/GPU stress tests      |

---

## 8) Safety Controls

| ID      | Requirement                   | Priority | Acceptance Criteria    |
| ------- | ----------------------------- | -------- | ---------------------- |
| REQ-8.1 | Arm/disarm & emergency kill   | **M**    | Immediate motor cutoff |
| REQ-8.2 | Fallback to manual/stabilized | **M**    | RC override verified   |
| REQ-8.3 | Geofencing limits             | **S**    | Breach triggers RTL    |

---

## 9) Usability & Documentation

| ID      | Requirement                    | Priority | Acceptance Criteria |
| ------- | ------------------------------ | -------- | ------------------- |
| REQ-9.1 | Documented experiment workflow | **M**    | Step-by-step guide  |
| REQ-9.2 | Dashboard for state            | **S**    | rqt/foxglove view   |
| REQ-9.3 | Reproducible configs           | **M**    | Versioned metadata  |

---

## 10) Portability & Extensibility

| ID       | Requirement                         | Priority | Acceptance Criteria    |
| -------- | ----------------------------------- | -------- | ---------------------- |
| REQ-10.1 | Add/replace sensors easily          | **S**    | Standard ROS messages  |
| REQ-10.2 | Deploy on STM32MP257F-DK and others | **M**    | Build scripts portable |
| REQ-10.3 | Extend to new missions              | **S**    | Same framework reused  |

---


