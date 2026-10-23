# One camera detection fused with the LiDAR scan (roadmap 5.1, racer_perception/fusion_node).
#
# SI units; the bearing is in the VEHICLE frame (base_link), LEFT positive, 0 straight ahead
# (REP-103), the same convention as racer_safety and racer_control. Range is metres from the
# LiDAR head, measured along the bearing. The box is in IMAGE pixels, origin top-left.
#
# header.stamp is the camera image's stamp, header.frame_id is "base_link" (the frame the
# bearing is expressed in).
std_msgs/Header header

string class_name
float32 score               # detector confidence, 0..1

float32 bearing_rad         # vehicle frame, left positive, wrapped to [-pi, pi]
float32 bearing_half_width_rad  # half the box's angular width, camera pinhole model

# Range along the bearing, metres. NaN when range_valid is false.
float32 range_m
bool range_valid

# Why range_m is or is not valid: "ok", "no_scan" (no LaserScan received yet), "stale_scan"
# (the newest scan is too far from the image stamp), "no_returns" (no valid return inside the
# angular window).
string range_status

# Number of valid LiDAR returns the range was taken from (0 unless range_valid).
uint16 range_samples

# Box in the source image, pixels: centre and size.
float32 box_cx_px
float32 box_cy_px
float32 box_width_px
float32 box_height_px
