builtin_interfaces/Time stamp
string[] joint_names            # tray_a_x_joint, tray_a_z_joint, tray_b_x_joint, tray_b_z_joint
float64[] measured_position_m   # from Gazebo JointStatePublisher (measured)
float64[] commanded_position_m
bool measured_fresh
bool moving
int32 waypoint_index            # last guarded-exchange waypoint reached (measured), -1 between
int32 target_index
string station                  # A_UPPER | B_UPPER | IN_TRANSIT | UNKNOWN
string[] accessible_docks
string[] latched                # aircraft still held by their dock latch
string activity                 # IDLE | INDEXING | RELEASING | STOPPED | FAULT
