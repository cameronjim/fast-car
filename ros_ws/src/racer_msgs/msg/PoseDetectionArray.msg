# All pose detections for one camera frame (roadmap 5.1 pose mode). header is the image's header,
# unchanged. One array per processed frame, EMPTY when nobody was found, so a consumer can tell
# "nobody there" from "detector dead".
std_msgs/Header header

# Size of the image the boxes and keypoints are in, pixels (0 if unknown). fusion_node uses the
# width for the pinhole bearing instead of a parameter when it is set.
uint32 image_width_px
uint32 image_height_px

racer_msgs/PoseDetection[] detections
