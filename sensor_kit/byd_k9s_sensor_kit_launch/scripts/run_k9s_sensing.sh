#!/usr/bin/env bash
# Bench test of the BYD K9S sensing: vehicle description + sensing only, a fake stopped-vehicle status
# (the bus has no vehicle interface yet; distortion correction and the concat filter need a twist),
# and RViz in base_link. No map, localization, perception, planning or control.
#
#   run_k9s_sensing.sh            # then check: ros2 topic hz /sensing/lidar/concatenated/pointcloud
set -eo pipefail
source "$HOME/utils/autoware_env_setup/aw.sh"
DIR=$(cd "$(dirname "$0")" && pwd)

python3 "$HOME/autoware.mach_e/fake_vehicle_status.py" &
PIDS=($!)
trap 'kill -INT "${PIDS[@]}" 2>/dev/null; wait' EXIT

# map_path is a required argument even with launch_map:=false
ros2 launch autoware_launch autoware.launch.xml \
  map_path:="$HOME/autoware_map/PD2LAB" vehicle_model:=byd_k9s_vehicle sensor_model:=byd_k9s_sensor_kit \
  launch_vehicle_interface:=false launch_map:=false launch_system:=false launch_localization:=false \
  launch_perception:=false launch_planning:=false launch_control:=false launch_api:=false rviz:=false &
PIDS+=($!)
sleep 12
rviz2 -d "$DIR/k9s_sensing.rviz"
