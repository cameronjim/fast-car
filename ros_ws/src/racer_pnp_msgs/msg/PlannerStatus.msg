# One per control tick of mppi_planner_node (50 Hz), for the log, part D and evaluation.
#
# header.stamp is the time of this control tick.
std_msgs/Header header
uint8 STATE_STARTING=0
uint8 STATE_DRIVING=1
# Local track stale, invalid, or failing its self-check.
uint8 STATE_NO_TRACK=2
uint8 STATE_NO_SCAN=3
uint8 STATE_NO_ODOM=4
# Every sampled trajectory collides.
uint8 STATE_BLOCKED=5
# Internal error, commands zero speed.
uint8 STATE_FAULT=6
uint8 state
uint32 cycle
float32 solve_time_s
# What was published on /drive_raw this tick.
float32 cmd_speed_mps
float32 cmd_steering_rad
# Wheel speed used as the initial state.
float32 est_speed_mps
# Modelled road wheel angle used as the initial state.
float32 est_steering_rad
# Speed cap in force this tick.
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
