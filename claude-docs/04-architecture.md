# 04 — Software Architecture (ROS graph, topics, language split)

## Command path (the only path to the wheels)

```
racer_policy/policy_node ─┐
                          ├─> /drive_raw ──> racer_safety/safety_node ──> /drive ──> racer_drivers/vesc_node
racer_control/tracker_node┘        (TTC braking, covariance gate,              (current-mode cmd)
                                    envelope enforcement, watchdog)
                                                          │
                               hardware RC mux (layer 1) ─┴─ physically downstream of ALL software
```

Rules: no node other than `safety_node` publishes `/drive`. No node other than `vesc_node`
talks to the ESC. The mux is physically downstream of everything; software cannot bypass it.

## Topics (interface — do not improvise names)

| Topic | Type | Rate | Producer |
|---|---|---|---|
| `/drive_raw` | AckermannDriveStamped | 50 Hz | tracker_node or policy_node |
| `/drive` | AckermannDriveStamped | 50 Hz | safety_node only |
| `/odom` | Odometry | ≥100 Hz | EKF (racer_state) |
| `/pose` | PoseWithCovarianceStamped | scan rate | particle filter |
| `/scan` | LaserScan | LiDAR rate | lidar driver |
| `/ingest/imu` | Imu | ≥200 Hz | ingest driver |
| `/ingest/wheel_speeds` | racer_msgs/WheelSpeeds | ≥100 Hz | ingest driver |
| `/ingest/steering` | racer_msgs/SteeringState | ≥100 Hz | ingest driver |
| `/power/rail` | racer_msgs/RailState | 10 Hz | ingest or vesc driver |
| `/safety/events` | racer_msgs/SafetyEvent | on event | safety_node (every gate action, logged) |

All units SI, REP-103 frames (`map`, `odom`, `base_link`), REP-105 semantics. Steering angle:
radians, left positive. Current: amps, drive positive.

## Language split

| Component | Language | Why |
|---|---|---|
| State estimation (EKF), localization | C++ — **adopted** (`robot_localization`, F1TENTH particle filter) | latency; don't hand-write, configure |
| safety_node, tracker_node, vesc path | C++ | control path tail latency |
| policy_node (inference @50 Hz) | Python initially, **with jitter histogram published**; port to C++ only if tails are bad | pragmatism |
| Training, analysis, sysid, tools | Python | velocity |

**Build vs adopt:** adopt proven C++ nodes first; write bespoke C++ only when profiling or
accuracy data indicts the adopted one. Hand-writing an EKF is the canonical Phase 2 overrun.

## Latency budget (measure, don't assume)

Sensor → estimator → policy → safety → ESC command, per-hop timestamps in every message
path. Deliverable: latency + jitter histogram for the full control path (roadmap 2.7).
Command-to-torque delay is measured once on the bench and encoded in the sim as a fixed
transport lag (see `07-sim-and-sysid.md`).

## Degradation behavior

- Pose covariance above gate threshold → safety_node ramps speed cap down (does NOT stop
  abruptly at speed unless TTC demands it).
- Policy OOD / envelope violation → fall back to pure tracker output (base controller).
- Watchdog: missing `/drive_raw` for 3 cycles → brake command.
- Every one of these emits `/safety/events`; interventions are a reported metric (see 09).
