# One per completed lap (part D, phase P4).
#
# header.stamp is the stamp of the start line crossing, header.frame_id is "map".
std_msgs/Header header
# 1 for the first completed lap.
uint32 lap_index
float32 lap_time_s
float32 lap_length_m
# Speed cap the lap was driven with.
float32 v_cap_mps
# /safety/events PHASE_ENGAGE with SEVERITY_BRAKE count during the lap.
uint16 safety_brakes
# Min of PlannerStatus.plan_min_clearance_m over the lap.
float32 min_clearance_m
# True if there was no brake engagement and no BLOCKED or NO_TRACK state during the lap.
bool clean
