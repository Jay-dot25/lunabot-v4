# Semantic terrain inference metadata and aligned image outputs.
std_msgs/Header header
uint8 STATE_UNKNOWN=0
uint8 STATE_READY=1
uint8 STATE_DEGRADED=2
uint8 STATE_ERROR=3
uint8 state
string model_name
string model_version
sensor_msgs/Image class_image
sensor_msgs/Image confidence_image
float32 inference_time_ms
float32 frames_per_second
uint32 dropped_frames
string detail
