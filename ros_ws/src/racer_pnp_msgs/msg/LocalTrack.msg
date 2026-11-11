# Local track estimate from one /scan (Plug-and-Play part A to part C).
#
# header.stamp is copied from the /scan message the track was computed from.
# header.frame_id is "base_link": every coordinate is in the car frame at that scan stamp.
std_msgs/Header header
# Bundle model_id of the producer, or "geometric-v0" for the geometric backend.
string model_id
uint8 BACKEND_NET=0
uint8 BACKEND_GEOMETRIC=1
uint8 backend
# Arc length between stations, 0.20 m.
float32 station_spacing_m
# The five arrays below have exactly K = 16 entries.
# Station 0 is the centreline point nearest to the base_link origin. Station k lies k * 0.20 m
# further along the centreline in the travel direction (the centreline direction within 90
# degrees of the car's heading), so the track reaches 3.0 m ahead.
float32[] x_m
float32[] y_m
# Free distance from centreline point k to the boundary on the left / right of the travel
# direction, along the centreline normal, capped at 2.0 m.
float32[] half_width_left_m
float32[] half_width_right_m
# 1 sigma position uncertainty of point k, metres.
float32[] sigma_m
# Stations 0 .. valid_count-1 may be used. 0 means "no track".
uint8 valid_count
# Label-free self-checks, NaN if not computable.
# boundary_residual_m: over valid stations within 2.0 m, the median distance from each predicted
# boundary point (centreline point plus or minus half width along the normal) to the nearest
# scan point.
float32 boundary_residual_m
# intrusion_fraction: of the scan points within 2.5 m of the car and ahead of it (x > 0), the
# share lying inside the predicted corridor (between the two boundaries over the valid stations)
# by more than 0.05 m.
float32 intrusion_fraction
# A consumer treats a message as "no track" if any array is not length 16, the spacing is not
# 0.20 within 1e-6, the frame is not base_link, or any of the first valid_count entries is not
# finite.
