#!/usr/bin/env bash
# Operator shell: environment sourced, prints the command cheat-sheet, then gives you a prompt.
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cat <<'TXT'
tins8 operator terminal - every assessed operation is a tins8ctl command:
  tins8ctl status                      tins8ctl watch
  tins8ctl mission start --track 3     tins8ctl mission cancel
  tins8ctl carousel release uav01      tins8ctl carousel index-b      tins8ctl carousel stop
  tins8ctl uav uav01 obs op1           tins8ctl uav uav02 goto wp_bus_stop     tins8ctl uav uav01 hold
  tins8ctl select 3                    tins8ctl select clear
  tins8ctl actor person_12 leave       tins8ctl pursuit uav01 start 3          tins8ctl pursuit uav01 stop
  tins8ctl teleop                      tins8ctl shutdown
TXT
exec bash --norc -i
