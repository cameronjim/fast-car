# The timing chain of one publish tick of mppi_planner_node (claude-docs/13 17.4.5, brief 42).
# Published on /pnp/planner/timing once per publish tick (50 Hz), recorded by the PnP bag profile.
# PlannerStatus and every other message are unchanged.
#
# Every time is float64 seconds on the node clock (the clock of header.stamp). A source that has not
# arrived yet is 0.0 (stamp and receipt); consumers treat 0.0 as absent. Stamps are the sources'
# own header stamps (a zero header stamp falls back to the arrival time, as the node does);
# receipts are the node clock at callback entry.
#
# header.stamp is the publish time of this tick.
std_msgs/Header header
# The planner cycle of this tick (PlannerStatus.cycle).
uint32 cycle
# The inputs of the solve whose plan this tick published.
float64 scan_stamp_s
float64 scan_receipt_s
float64 track_stamp_s
float64 track_receipt_s
float64 odom_stamp_s
float64 odom_receipt_s
float64 solve_start_s
float64 solve_end_s
# The latest gated /drive the node had when this tick was published: its header stamp (safety_node
# stamps it with the cycle time of its 50 Hz timer, so this is the safety cycle) and the time the
# node received it.
float64 gated_stamp_s
float64 gated_receipt_s
# The planner cycle whose published /drive_raw command that gated message equals in BOTH fields
# (exact float32 equality), the newest such cycle of the last 16, 0 if none (the gate changed a
# field, or the message is older than 16 ticks). Equal consecutive commands make this the newest
# of several; tools/analysis resolve ambiguity from the bag (timing_chain.py).
uint32 gated_echo_cycle
# What the command sequence of this tick's solve is.
# The smoothed weighted mean, collision free.
uint8 FINAL_MEAN=0
# The mean collided or was not finite: the best collision free sample of the chosen class.
uint8 FINAL_FALLBACK=1
# Every sample collided: the least bad one, speed 0.
uint8 FINAL_ALL_COLLIDE=2
uint8 final_sequence
# The cost terms of the final sequence, float32, fixed order:
#   0 progress, 1 collision, 2 clearance, 3 boundary, 4 lateral, 5 smoothness, 6 steer_rate,
#   7 terminal, 8 gate, 9 lap_trust, 10 lap_terminal
# (the MppiSolver CostBreakdown terms, weights applied; progress is negative when moving forward).
# Empty when this tick had no solver run (no plan).
float32[] cost_terms
# The term with the largest marginal cost on speed in the final sequence. The rule: re-evaluate the
# final sequence with every speed command raised by 0.1 m/s; clearance, terminal, gate and lap_trust
# are the terms that can limit; if the sum of their positive changes is at least the fall of the
# progress terms the largest one is the limiter; otherwise the cap if the plan's mean speed over
# its first 0.5 s is within 0.05 m/s of the cap, else the sample choice. SPEED_LIMITER_NONE: the plan does not
# move. VALIDITY: the cap was lowered by the track confidence (a cap limiter whose cap is below
# the cap at full confidence).
uint8 SPEED_LIMITER_NONE=0
uint8 SPEED_LIMITER_CAP=1
uint8 SPEED_LIMITER_CLEARANCE=2
uint8 SPEED_LIMITER_TERMINAL=3
uint8 SPEED_LIMITER_GATE=4
uint8 SPEED_LIMITER_LAP_BOUND=5
uint8 SPEED_LIMITER_VALIDITY=6
uint8 SPEED_LIMITER_SAMPLE_CHOICE=7
uint8 speed_limiter
