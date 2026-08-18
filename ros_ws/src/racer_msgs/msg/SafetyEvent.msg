# racer_safety/safety_node emits one of these on EVERY intervention
# (claude-docs/04-architecture.md: "/safety/events | racer_msgs/SafetyEvent | on event |
# safety_node (every gate action, logged)"; claude-docs/05-safety.md: "Emits /safety/events
# on every intervention. Interventions are evaluation data (09) -- an unlogged intervention
# is a bug."). Kept minimal per claude-docs/02-repo-layout.md's rule that racer_msgs exists
# only where std/ackermann msgs won't do: stamp, which gate raised the event, and a free-text
# detail string are the whole contract.

builtin_interfaces/Time stamp

# Severity: INFO is advisory-only (no command change, e.g. TTC warning-zone), WARNING is
# reserved for future advisory-with-attention-needed use, BRAKE means this event coincided
# with a hard brake command on /drive.
uint8 SEVERITY_INFO=0
uint8 SEVERITY_WARNING=1
uint8 SEVERITY_BRAKE=2
uint8 severity

# Which gate raised this event. One of: "watchdog", "command_sanity", "bounds_clamp",
# "rate_limit", "ttc", "covariance", "internal_fault" (racer_safety::GateSource, see
# ros_ws/src/racer_safety/include/racer_safety/gate_logic.hpp).
string source

# Human-readable specifics (values involved, thresholds crossed) for post-hoc analysis
# (claude-docs/09-evaluation.md) -- interventions are a reported metric.
string detail
