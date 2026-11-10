# A route is tied to a stable goal revision and the inflated grid used to plan it.
# /local_path remains visualization only. Motion consumes this contract.
std_msgs/Header header
geometry_msgs/PoseStamped goal
nav_msgs/Path path
nav_msgs/OccupancyGrid clearance_grid
# True when the path stops at the edge of observed ground before the goal. The path
# still belongs to this goal revision; only observed, clear ground is included.
bool partial
