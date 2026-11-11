# Lap memory guidance to the planner (part D, phase P4), 2 Hz.
std_msgs/Header header
# The lap being driven now, 1 = first lap.
uint32 lap_index
# Speed cap for this lap.
float32 v_cap_mps
# Empty in D v0. From D v1: stations along the lap from the start line.
float32[] s_m
# Per station speed bound from the safe set.
float32[] v_ref_mps
# Per station time to finish the lap, from the safe set.
float32[] cost_to_go_s
