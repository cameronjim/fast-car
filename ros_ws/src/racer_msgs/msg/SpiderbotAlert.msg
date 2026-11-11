# Raised once per spiderbot engagement, after the car has stopped and tried to face the person
# (roadmap 5.3, ros_ws/src/racer_apps). Published on /spiderbot/alert, reliable, KeepLast(10).
# SI units. header.stamp is the time the rule fired (the image stamp of the frame that completed
# the temporal rule); header.frame_id is "map".
std_msgs/Header header

string event_id             # unique per engagement within a run, e.g. "20261111T031500Z-0001"
string rule                 # name of the pose rule that fired, e.g. "arms_raised"
uint16 held_frames          # how many consecutive frames the rule held when it fired
float32 score               # detector score of the person that triggered it, 0..1

# The person as the camera and LiDAR saw them when the rule fired (the trigger frame).
float32 bearing_rad         # vehicle frame, left positive, 0 straight ahead
float32 range_m             # NaN when range_valid is false
bool range_valid

# Whether the car turned to face the person: the bearing was within the tolerance when it
# stopped. False when the person was lost, the range was unknown or too short to creep, or the
# facing timed out. The alert is raised either way.
bool facing_ok
float32 final_bearing_rad   # last bearing seen while facing; NaN if never seen after the stop

# Where the car was when it raised the alert, in the map frame. pose_valid is false when no
# localised pose had been received (patrol needs one, so this should be true in practice).
bool pose_valid
float32 pose_x_m
float32 pose_y_m
float32 pose_yaw_rad

# Snapshot JPEG written for this event (path as the node sees it), empty if no image was
# available or it could not be written. The same event is appended to the events log.
string snapshot_path
