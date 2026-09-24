#!/usr/bin/env bash
# sitl_swarm.sh - five ArduCopter SITL instances wired exactly like the real
# system (same sysids, same script ports 1457N, same .parm files), pads 6 m apart.
# SITL feeds the script directly, standing in for the WiFi module + MAVProxy hop.
#
# Prereq: an ArduPilot checkout with Copter SITL built once:
#   cd ~/ardupilot && ./waf configure --board sitl && ./waf copter
# Usage:
#   ARDUPILOT=~/ardupilot ./sitl_swarm.sh
#   then: python3 swarm_gcs.py        (in another terminal)
#   and : QGroundControl on UDP 14550 to fly the master (e.g. a mission in AUTO)
# If an option is rejected, check `sim_vehicle.py --help` for your version.
set -euo pipefail
ARDUPILOT=${ARDUPILOT:-$HOME/ardupilot}
HERE="$(cd "$(dirname "$0")" && pwd)"
LAT=-35.3632621; LON0=149.1652374; ALT=584   # ArduPilot's default SITL field
TMP=$(mktemp -d)

cat > "$TMP/sitl_override.parm" << 'P'
# SITL simulates a 3S pack - disable the 6S voltage thresholds
BATT_LOW_VOLT,0
BATT_CRT_VOLT,0
P

trap 'kill 0' EXIT INT TERM
cd "$ARDUPILOT/ArduCopter"
for id in 1 2 3 4 5; do
  i=$((id - 1))
  lon=$(python3 -c "print(f'{$LON0 + $i * 6.6e-5:.7f}')")   # ~6 m east per pad
  if [ "$id" -eq 1 ]; then base="$HERE/params/master.parm"; else base="$HERE/params/slave$id.parm"; fi
  cat "$base" "$TMP/sitl_override.parm" > "$TMP/d$id.parm"
  sim_vehicle.py -v ArduCopter -I "$i" --sysid "$id" --no-rebuild \
      --custom-location="$LAT,$lon,$ALT,0" \
      --add-param-file="$TMP/d$id.parm" \
      --out="udp:127.0.0.1:1457$id" \
      --mavproxy-args="--daemon" > "$TMP/sitl_$id.log" 2>&1 &
  echo "SITL drone $id started (log $TMP/sitl_$id.log)"
  sleep 3
done
echo "Five vehicles running. Ctrl-C stops all."
wait
