# One line of the shared command log (/tins8/command_log). Every command, from the
# terminal or the GUI, moves through REQUESTED -> ACCEPTED -> COMPLETED, or ends in
# REJECTED, CANCELED or FAILED.  GUI and terminal both display this same stream.
builtin_interfaces/Time stamp
string request_id
string target        # uav01, uav02, carousel, tracker, mission, scenario, ugv
string command
string argument
string state         # REQUESTED | ACCEPTED | COMPLETED | REJECTED | CANCELED | FAILED
string detail
