# racer_safety/safety_node emits one of these on every gate TRANSITION -- one record when a
# gate ENGAGES and one when that same engagement RELEASES -- never one per control cycle
# (GitHub issue #37: a gate that stayed engaged re-emitted at 50 Hz, so a count of these
# records measured how LONG a gate was engaged, not how many times it engaged).
# claude-docs/04-architecture.md ("/safety/events | racer_msgs/SafetyEvent | on event |
# safety_node (every gate action, logged)") and claude-docs/05-safety.md ("Emits
# /safety/events on every intervention. Interventions are evaluation data (09) -- an
# unlogged intervention is a bug") are both satisfied by the engage record: every
# intervention still produces exactly one, and the sustained state is the interval between
# the engage record and its release record.
#
# COUNTING RULE (claude-docs/09-evaluation.md reports interventions as a metric): count
# records with `phase == PHASE_ENGAGE`. A PHASE_RELEASE record closes an engagement already
# counted and must never be counted again.
#
# Kept minimal per claude-docs/02-repo-layout.md's rule that racer_msgs exists only where
# std/ackermann msgs won't do: stamp, which gate raised the event, the transition phase, how
# long the engagement lasted, and a free-text detail string are the whole contract.

builtin_interfaces/Time stamp

# Severity: INFO is advisory-only (no command change, e.g. TTC warning-zone), WARNING is
# reserved for future advisory-with-attention-needed use, BRAKE means this event coincided
# with a hard brake command on /drive. A PHASE_RELEASE record carries the SAME severity as
# the engagement it closes, so a consumer can filter on (phase, severity) together.
uint8 SEVERITY_INFO=0
uint8 SEVERITY_WARNING=1
uint8 SEVERITY_BRAKE=2
uint8 severity

# Transition phase. PHASE_ENGAGE: this gate/severity became engaged on this control cycle
# (one intervention = one of these). PHASE_RELEASE: the engagement that PHASE_ENGAGE opened
# ended on this cycle; `duration_s` says how long it lasted. Zero-valued default is
# PHASE_ENGAGE deliberately: a consumer reading an older bag with no `phase` field sees the
# record as an engagement, which is what those records meant.
uint8 PHASE_ENGAGE=0
uint8 PHASE_RELEASE=1
uint8 phase

# Seconds the engagement lasted, measured on safety_node's monotonic clock (see
# safety_node.cpp's CLOCK POLICY). Always 0.0 on a PHASE_ENGAGE record (the engagement has
# not ended yet); >= 0.0 on a PHASE_RELEASE record.
float64 duration_s

# Which gate raised this event. One of: "watchdog", "command_sanity", "bounds_clamp",
# "rate_limit", "ttc", "covariance", "internal_fault" (racer_safety::GateSource, see
# ros_ws/src/racer_safety/include/racer_safety/gate_logic.hpp).
string source

# Human-readable specifics (values involved, thresholds crossed) for post-hoc analysis
# (claude-docs/09-evaluation.md) -- interventions are a reported metric. On a PHASE_ENGAGE
# record this is the gate's own description of what tripped it; on a PHASE_RELEASE record it
# states how long the engagement lasted.
string detail
