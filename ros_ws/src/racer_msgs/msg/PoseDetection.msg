# One person with body keypoints, straight from the pose detector (roadmap 5.1 pose mode).
#
# Everything is in IMAGE pixels, origin top-left. No vehicle-frame bearing or LiDAR range yet:
# racer_perception/fusion_node adds those and republishes this as racer_msgs/ObjectDetection with
# the same keypoints. The box is centre and size, like ObjectDetection.
string class_name           # "person" for the COCO pose models
float32 score               # box confidence, 0..1

float32 box_cx_px
float32 box_cy_px
float32 box_width_px
float32 box_height_px

# Names the order of `keypoints`. "coco17": nose, left_eye, right_eye, left_ear, right_ear,
# left_shoulder, right_shoulder, left_elbow, right_elbow, left_wrist, right_wrist, left_hip,
# right_hip, left_knee, right_knee, left_ankle, right_ankle (the person's own left and right).
string keypoint_layout
racer_msgs/Keypoint2D[] keypoints
