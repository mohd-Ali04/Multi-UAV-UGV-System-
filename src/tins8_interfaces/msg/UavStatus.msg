builtin_interfaces/Time stamp
string uav_id
bool connected                 # PX4 telemetry received within the freshness limit
float32 telemetry_age_s        # age of the newest vehicle_local_position sample (ROS time)
bool armed
string nav_state               # PX4 navigation state name
string phase                   # DOCKED | TAKEOFF | TRANSIT | HOLD | PURSUIT | LANDING | LANDED
string control_owner           # agent | pursuit | none
geometry_msgs/Point position_enu   # world ENU from PX4 estimate (not simulator ground truth)
float32 yaw_enu_rad
float32 speed_mps
geometry_msgs/Point hold_target_enu
float32 hold_error_m
string target_name             # op1, wp_bus_stop ...
string last_command
string last_command_state
