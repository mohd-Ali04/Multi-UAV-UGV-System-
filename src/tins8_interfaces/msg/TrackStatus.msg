builtin_interfaces/Time stamp
string camera_uav
string state                  # IDLE | ACQUIRING | TRACKING | LOST
int32 selected_id             # YOLOX tracker ID chosen by the operator, -1 none
float32 confidence
float32[] bbox_xyxy           # selected detection in full-frame pixels
float32 center_error_px_x     # inset window centre vs detection centre
float32 center_error_px_y
float32 seconds_since_seen
float32 selection_to_centered_s   # -1 until the first centred inset after a selection
int32 visible_person_ids_count
int32[] visible_person_ids
