#!/usr/bin/env bash
# start_links.sh - WiFi version: one MAVProxy router per drone.
#
#   drone N WiFi module --UDP--> <GCS_IP>:1456N --> MAVProxy #N --> 127.0.0.1:1457N (swarm_gcs.py)
#                                                              \--> 127.0.0.1:14550 (QGC / MP, monitor)
#
# Configure module N to send UNICAST to the GCS PC's static IP, port 1456N.
# MAVProxy learns the module's address from its first packet and replies there.
# A module pointed at the wrong port is harmless: the script filters by SYSID
# and will simply report that drone as LOST.
# --streamrate=-1 stops MAVProxy overwriting the SRn_ rates on connect.
# Each instance writes its own .tlog into $LOGDIR/dN.
set -euo pipefail
LOGDIR=${LOGDIR:-$HOME/swarm_logs/$(date +%Y%m%d_%H%M%S)}

trap 'kill 0' EXIT INT TERM

for id in 1 2 3 4 5; do
  mkdir -p "$LOGDIR/d$id"
  mavproxy.py --master="udpin:0.0.0.0:1456$id" \
      --out "udp:127.0.0.1:1457$id" \
      --out "udp:127.0.0.1:14550" \
      --streamrate=-1 --source-system 254 \
      --state-basedir "$LOGDIR/d$id" --daemon \
      > "$LOGDIR/d$id/mavproxy.out" 2>&1 &
  echo "drone $id: listening on UDP 1456$id -> script 1457$id + QGC 14550"
done
echo "Routers up. Logs: $LOGDIR.  Ctrl-C stops all five."
wait
