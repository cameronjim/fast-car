# One per control tick of mppi_planner_node (50 Hz), for the log, part D and evaluation.
#
# header.stamp is the time of this control tick.
std_msgs/Header header
uint8 STATE_STARTING=0
uint8 STATE_DRIVING=1
# Not produced since planner v0.1: a missing or distrusted track lowers track_confidence and the
# planner drives on the scan alone. Kept for old bags.
uint8 STATE_NO_TRACK=2
uint8 STATE_NO_SCAN=3
uint8 STATE_NO_ODOM=4
# Nothing drivable and no recovery running, see block_reason.
uint8 STATE_BLOCKED=5
# Internal error, commands zero speed.
uint8 STATE_FAULT=6
# Backing up along a planned reverse arc to get out of a stuck pose (cmd_speed_mps is negative).
uint8 STATE_RECOVERING=7
uint8 state
uint32 cycle
float32 solve_time_s
# What was published on /drive_raw this tick. cmd_speed_mps is negative while STATE_RECOVERING and
# never otherwise.
float32 cmd_speed_mps
float32 cmd_steering_rad
# Wheel speed used as the initial state.
float32 est_speed_mps
# Modelled road wheel angle used as the initial state.
float32 est_steering_rad
# Effective speed cap in force this tick (v_max or the guidance cap, scaled down with the track
# confidence).
float32 v_cap_mps
float32 best_cost
# Centreline progress of the weighted mean plan.
float32 plan_progress_m
# Smallest body clearance along the weighted mean plan.
float32 plan_min_clearance_m
# Share of samples that hit the hard collision cost.
float32 collision_fraction
float32 track_age_s
float32 scan_age_s
# Samples used this tick.
uint16 samples
string dynamics_model_id
string perception_model_id
# Appended in planner v0.1. Weight of the track prior this tick, 0 to 1 (1 minus it weighs the
# free-space progress field).
float32 track_confidence
# Free-space goal in base_link now; NaN when there is no field.
float32 freespace_goal_x_m
float32 freespace_goal_y_m
# Cost-to-reach of the goal from the car, metres of weighted path.
float32 freespace_reach_m
uint8 BLOCK_NONE=0
uint8 BLOCK_ALL_COLLIDE=1
uint8 BLOCK_BELOW_SPEED_FLOOR=2
uint8 BLOCK_WHEEL_STALLED=3
uint8 BLOCK_GATE_REFUSING=4
uint8 BLOCK_RECOVERY_EXHAUSTED=5
# Why the planner is in STATE_BLOCKED (BLOCK_NONE otherwise).
uint8 block_reason
# Recovery attempts counted since the car last covered 0.5 m forward.
uint8 recovery_attempts
