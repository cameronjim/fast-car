# One body keypoint from the pose detector (roadmap 5.1 pose mode, 5.3 spiderbot).
#
# Position in IMAGE pixels, origin top-left, x to the right, y DOWN (so a raised wrist has a
# SMALLER y than its shoulder). confidence is the detector's per-keypoint score, 0..1. A keypoint
# the model could not place has confidence 0 and position (0, 0): ignore it by confidence, never
# by position.
float32 x_px
float32 y_px
float32 confidence
