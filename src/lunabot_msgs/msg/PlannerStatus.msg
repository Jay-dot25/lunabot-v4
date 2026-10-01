# Structured global/incremental planner status.
std_msgs/Header header
uint8 STATE_IDLE=0
uint8 STATE_PLANNING=1
uint8 STATE_PATH_READY=2
uint8 STATE_GOAL_REACHED=3
uint8 STATE_NO_PATH=4
uint8 STATE_ERROR=5
uint8 state
string planner_id
bool success
bool incremental
float32 planning_time_ms
uint32 expanded_nodes
uint32 path_cells
float32 path_length_m
float32 accumulated_cost
string detail
