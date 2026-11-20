#!/usr/bin/env bash
# Start one PX4 SITL instance attached to an aircraft that already exists in the Gazebo world.
#   scripts/start_px4.sh uav01 [world] [--daemon]
# Ready when the log prints "INFO  [uxrce_dds_client] successfully created" and
# "INFO  [commander] Ready for takeoff!".  Type commands at the pxh> prompt (e.g. 'listener vehicle_status').
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"   # ROS setup files are not set -u safe
set -eo pipefail
uav="${1:-uav01}"; world="${2:-fieldlab}"; mode="${3:-}"
mkdir -p "$TINS8_LOG_DIR/pids"; echo $$ > "$TINS8_LOG_DIR/pids/px4_$uav.pid"
idx=$((10#${uav#uav} - 1))
inst=$((TINS8_PX4_INSTANCE_BASE + idx))
build="$TINS8_PX4_DIR/build/px4_sitl_default"
[ -x "$build/bin/px4" ] || { echo "PX4 SITL is not built at $build - run scripts/setup_deps.sh"; exit 1; }
work="$TINS8_LOG_DIR/px4/$uav"
rm -rf "$work"; mkdir -p "$work"; cd "$work"     # fresh parameters every start
export PX4_SIMULATOR=gz PX4_SYS_AUTOSTART=4001 PX4_GZ_STANDALONE=1 PX4_GZ_WORLD="$world"
export PX4_GZ_MODEL_NAME="$uav" PX4_UXRCE_DDS_NS="$uav" PX4_UXRCE_DDS_PORT="$TINS8_XRCE_PORT"
unset PX4_SIM_MODEL
params="$(ros2 pkg prefix tins8_bringup)/share/tins8_bringup/config/px4_qav250.params"
while read -r name value; do
  [[ -z "${name}" || "${name}" == \#* ]] && continue
  export "PX4_PARAM_${name}=${value}"
done < "$params"
# rcS applies PX4_PARAM_* before the airframe file; a value equal to the firmware default is not
# stored, so the airframe's 'param set-default' (e.g. NAV_DLL_ACT 2) would win.  Re-apply the
# overrides through the PX4 client once the instance answers.
(
  for _ in $(seq 1 60); do
    "$build/bin/px4-param" --instance "$inst" show SYS_AUTOSTART >/dev/null 2>&1 && break; sleep 1
  done
  sleep 3
  while read -r name value; do
    [[ -z "${name}" || "${name}" == \#* ]] && continue
    "$build/bin/px4-param" --instance "$inst" set "$name" "$value" >/dev/null 2>&1 || true
  done < "$params"
  echo "INFO  [tins8] re-applied $(grep -cv '^\s*\(#\|$\)' "$params") parameter overrides" >&2
) &
echo "PX4 instance $inst (MAV_SYS_ID $((inst + 1))) for $uav in world $world, DDS namespace /$uav, agent port $TINS8_XRCE_PORT"
if [ "$mode" = "--daemon" ]; then
  exec "$build/bin/px4" -d -i "$inst" "$build/etc"
else
  exec "$build/bin/px4" -i "$inst" "$build/etc"
fi
