#!/usr/bin/env python3
"""
swarm_gcs.py - GPS-only leader/follower formation controller
=============================================================

Fleet:   1 Master (SYSID 1, flown by RC pilot or GCS waypoints)
         4 Slaves (SYSID 2..5, ArduCopter in GUIDED, commanded by this script)
Sensors: GPS + barometer only. No proximity sensing of any kind.
Airframe: 7-inch quads, MicoAir743 stack, GEPRC GEP-M1025 GPS, RadioMaster RP3
          ELRS receiver, WiFi telemetry module (see README.md).

Because nothing on the aircraft can *see* another aircraft, safety is layered,
in this order of importance:

  1. GEOMETRY    Formation slots are >= 8-10 m apart horizontally, i.e. larger
                 than (GPS error of A + GPS error of B + tracking error).
  2. ALTITUDE    Every slave lives on its own altitude layer above the master.
                 Joining / re-slotting happens on unique "transit" layers, and
                 each slave's RTL altitude is unique and above everything else.
  3. BEHAVIOUR   Conservative speed limits, rate-limited formation rotation and
                 velocity feed-forward so slaves do not lag behind the master.
  4. MONITORING  This script: closest-point-of-approach (CPA) prediction, soft
                 target repulsion, hard escape overrides, and a HOLD state that
                 is entered on ANY anomaly and NEVER auto-resumes.
  5. ONBOARD FS  Each vehicle's own failsafes (GCS heartbeat, RC, EKF, battery,
                 fence). They are the last line of defence if this script, the
                 PC, a radio, or the operator fails.

Data flow (WiFi telemetry):
  drone N WiFi module --UDP--> GCS_IP:1456N --> MAVProxy #N --> 127.0.0.1:1457N --> this script
  this script --> same socket --> MAVProxy #N --> module N --> drone N
  One port per drone on purpose: a UDP socket that receives several drones
  replies to ALL of them, which would defeat per-drone heartbeat withholding.

Status: bench-tested against the kinematic mock (mock_fleet.py) only.
        Validate in ArduPilot SITL, then with props off, then one slave at a
        time in the field, BEFORE flying the full swarm.

Usage:
  python3 swarm_gcs.py                      # defaults: V formation, 10 m
  python3 swarm_gcs.py --formation diamond --spacing 12 --heading-mode course
  python3 swarm_gcs.py --dry-run            # everything except position targets
Type 'help' at the prompt for operator commands.
"""

import argparse
import dataclasses
import itertools
import logging
import math
import queue
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from pymavlink import mavutil

MAV = mavutil.mavlink

# =============================================================================
# 1. CONFIGURATION  (every number here interacts with the .parm files - change
#                    them together, and re-run sanity_check_config())
# =============================================================================

# ---- Identity ---------------------------------------------------------------
SCRIPT_SYSID = 250      # MUST equal SYSID_MYGCS on every SLAVE. Monitoring GCSs
                        # (QGC / Mission Planner) keep sysid 255, so their
                        # heartbeats can NOT keep a slave's GCS failsafe quiet
                        # if this script dies.
SCRIPT_COMPID = 191     # MAV_COMP_ID_ONBOARD_COMPUTER (any non-autopilot id)

MASTER_ID = 1
SLAVE_IDS = (2, 3, 4, 5)

DEFAULT_LINKS = {       # one local UDP port per vehicle, fed by start_links.sh
    1: "udpin:127.0.0.1:14571",
    2: "udpin:127.0.0.1:14572",
    3: "udpin:127.0.0.1:14573",
    4: "udpin:127.0.0.1:14574",
    5: "udpin:127.0.0.1:14575",
}

# ---- Loop / stream rates ----------------------------------------------------
CONTROL_RATE_HZ = 10.0      # position targets per slave per second (WiFi has the headroom)
HEARTBEAT_PERIOD_S = 1.0
STATUS_PERIOD_S = 2.0
INTERVAL_REFRESH_S = 30.0   # re-assert message intervals (a GCS may change them)
MASTER_POS_RATE_HZ = 10     # GLOBAL_POSITION_INT from master
SLAVE_POS_RATE_HZ = 10      # GLOBAL_POSITION_INT from slaves
AUX_RATE_HZ = 2             # GPS_RAW_INT + EKF_STATUS_REPORT

# ---- Link / data freshness --------------------------------------------------
HB_TIMEOUT_S = 3.0          # no HEARTBEAT this long => link lost. WiFi drops come in
                            # bursts (retries, re-association); 3 s avoids nuisance HOLDs
POS_TIMEOUT_S = 1.5         # no GLOBAL_POSITION_INT this long => stale
AUX_TIMEOUT_S = 3.0         # GPS_RAW_INT / EKF_STATUS_REPORT freshness
SLAVE_LOST_FORCE_FS_S = 8.0 # after this long without a slave's telemetry we
                            # stop heartbeating it, so its onboard GCS failsafe
                            # (RTL on its own layer) is GUARANTEED to fire
                            # instead of it hovering blind forever.

# ---- Navigation quality gates -----------------------------------------------
MIN_FIX_TYPE = 3            # 3 = 3D fix
MIN_SATS = 10
MAX_HDOP = 1.6
GLITCH_JUMP_M = 6.0         # position "innovation" that counts as a GPS jump
GLITCH_HOLDOFF_S = 3.0      # distrust the position for this long after a jump
EKF_POS_HORIZ_ABS = 16      # EKF_STATUS_REPORT.flags bits (ardupilotmega.xml)
EKF_CONST_POS_MODE = 128
EKF_UNINITIALIZED = 1024
EKF_GPS_GLITCHING = 32768

# ---- Formation geometry -----------------------------------------------------
# Slot = (forward, right, layer):
#   forward/right are in units of `spacing`, in the formation's body frame
#   (forward = formation heading, right = 90 deg clockwise of it).
#   layer is metres ABOVE the master's altitude. Adjacent slots alternate
#   layers so a lateral error still leaves vertical separation, but the
#   design never RELIES on vertical separation alone (baro drift is 1-2 m).
FORMATIONS: Dict[str, Dict[int, Tuple[float, float, float]]] = {
    #          slave: (fwd,  right, layer_m)
    "v":       {2: (-1.0, -1.0, 3.0), 3: (-1.0, +1.0, 3.0),
                4: (-2.0, -2.0, 6.0), 5: (-2.0, +2.0, 6.0)},
    "line":    {2: (0.0, -1.0, 3.0),  3: (0.0, +1.0, 3.0),     # line abreast
                4: (0.0, -2.0, 6.0),  5: (0.0, +2.0, 6.0)},
    "trail":   {2: (-1.0, 0.0, 3.0),  3: (-2.0, 0.0, 6.0),     # column
                4: (-3.0, 0.0, 3.0),  5: (-4.0, 0.0, 6.0)},
    # Master in the centre. NOTE: slave 2 flies AHEAD of the master; the
    # pilot must not accelerate harder than the slaves can (keep LOIT_ACC_MAX
    # on the master lower than WPNAV_ACCEL on the slaves).
    "diamond": {2: (+1.0, 0.0, 3.0),  3: (0.0, +1.0, 6.0),
                4: (-1.0, 0.0, 3.0),  5: (0.0, -1.0, 6.0)},
}
DEFAULT_FORMATION = "v"
DEFAULT_SPACING_M = 10.0
MIN_SPACING_M = 8.0
MIN_SLOT_SEPARATION_M = 8.0     # enforced for every pair incl. the master

# Unique per-slave layers used while joining/re-slotting (metres above master)
TRANSIT_LAYER_M = {2: 10.0, 3: 13.0, 4: 16.0, 5: 19.0}
# Unique takeoff altitudes (metres above each slave's own home/pad)
TAKEOFF_ALT_M = {2: 8.0, 3: 11.0, 4: 14.0, 5: 17.0}

# ---- Altitude envelope ("altitude budget") ----------------------------------
MASTER_MIN_ALT_M = 8.0      # below this the layers above can't be guaranteed
MASTER_MAX_ALT_M = 25.0     # 25 + 19 (highest transit layer) = 44 < ceiling
SLAVE_FLOOR_ALT_M = 6.0     # absolute floor for any slave target
SLAVE_CEIL_ALT_M = 45.0     # MUST stay below the lowest slave RTL_ALT (50 m)

# ---- Motion shaping ---------------------------------------------------------
HEADING_MODES = ("course", "heading", "fixed")
DEFAULT_HEADING_MODE = "course"  # rotate with the master's track, not its nose:
                                 # a pilot yawing in a hover won't swing the swarm
COURSE_MIN_SPEED_MS = 1.5   # below this ground speed, course is noise -> freeze
MAX_FORMATION_ROT_DPS = 8.0 # outer slave at 28 m radius moves 3.9 m/s from this
LINK_LATENCY_S = 0.15       # typical EKF->WiFi->script->WiFi delay (measure yours)
MAX_EXTRAPOLATION_S = 1.0
MAX_FF_SPEED_MS = 9.0       # keep below slave WPNAV_SPEED (10 m/s)
MAX_FF_VZ_MS = 1.5

# ---- Join / re-slot state machine -------------------------------------------
ALT_REACHED_M = 1.0         # CLIMB -> TRANSIT when within this of transit alt
JOIN_CAPTURE_M = 3.0        # TRANSIT -> IN when within this of the slot
FALLOUT_ERR_M = 12.0        # IN -> CLIMB (re-join) if slot error exceeds this

# ---- Collision avoidance ----------------------------------------------------
SOFT_R_M = 6.0              # soft repulsion radius (horizontal)
SOFT_V_M = 5.0              # ...applies only if vertical gap is below this
LOOKAHEAD_S = 1.5           # obstacles are projected this far ahead
CPA_HORIZON_S = 3.0         # hard monitor looks this far ahead
CRIT_H_M = 4.0              # predicted horizontal miss distance => conflict
CRIT_V_M = 2.5              # ...if predicted vertical gap is also below this
ESCAPE_CLIMB_M = 4.0        # upper aircraft of a conflicting pair climbs this
HOLD_DECEL_MSS = 2.0        # used to place hold points at the stopping point
MASTER_RTL_CLEARANCE_M = 5.0  # slaves climb this much if master RTLs/lands

# ---- Failsafe escalation ----------------------------------------------------
MASTER_LOSS_ACTION = "RTL"  # "RTL", "LAND" or "NONE" after the delay below
MASTER_LOSS_DELAY_S = 15.0  # continuous master failure before escalation
STAGGER_S = 3.0             # gap between successive slave RTL/LAND/takeoff
AIRBORNE_ALT_M = 1.5

COPTER_MODES = {"STABILIZE": 0, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4,
                "LOITER": 5, "RTL": 6, "LAND": 9, "POSHOLD": 16,
                "BRAKE": 17, "SMART_RTL": 21}
MASTER_RETURNING_MODES = {"RTL", "LAND", "SMART_RTL"}

# SET_POSITION_TARGET_GLOBAL_INT: use position + velocity (as feed-forward) +
# yaw; ignore acceleration and yaw-rate.
TYPE_MASK_POS_VEL_YAW = (MAV.POSITION_TARGET_TYPEMASK_AX_IGNORE
                         | MAV.POSITION_TARGET_TYPEMASK_AY_IGNORE
                         | MAV.POSITION_TARGET_TYPEMASK_AZ_IGNORE
                         | MAV.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

# =============================================================================
# 2. GEOMETRY HELPERS
#    Everything is done in a local flat-earth North/East frame. Over the <1 km
#    extent of a formation the projection error is millimetres - far below
#    GPS noise - so no UTM / geodesic library is needed.
# =============================================================================
EARTH_RADIUS_M = 6378137.0


def ll_to_ne(lat0: float, lon0: float, lat: float, lon: float) -> Tuple[float, float]:
    """(lat, lon) -> metres (north, east) of the reference point (lat0, lon0)."""
    dn = math.radians(lat - lat0) * EARTH_RADIUS_M
    de = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    return dn, de


def ne_to_ll(lat0: float, lon0: float, dn: float, de: float) -> Tuple[float, float]:
    """Inverse of ll_to_ne: offset (dn, de) metres from (lat0, lon0) -> (lat, lon)."""
    lat = lat0 + math.degrees(dn / EARTH_RADIUS_M)
    lon = lon0 + math.degrees(de / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon


def body_to_ne(fwd: float, right: float, psi: float) -> Tuple[float, float]:
    """Rotate a formation-frame offset into North/East.

    psi is a compass heading in radians (0 = north, +clockwise), so:
        north = fwd*cos(psi) - right*sin(psi)
        east  = fwd*sin(psi) + right*cos(psi)
    Check: psi = 90 deg (facing east): fwd -> +east, right -> -north (south). OK.
    """
    c, s = math.cos(psi), math.sin(psi)
    return fwd * c - right * s, fwd * s + right * c


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def closest_approach(rn: float, re: float, rvn: float, rve: float,
                     horizon: float) -> Tuple[float, float]:
    """Horizontal closest point of approach between two constant-velocity tracks.

    r  = position of B relative to A (m), rv = velocity of B relative to A (m/s).
    Minimising |r + rv*t| gives t* = -(r . rv) / |rv|^2, clamped to [0, horizon].
    Returns (t*, miss_distance).
    """
    v2 = rvn * rvn + rve * rve
    t = 0.0 if v2 < 1e-6 else clamp(-(rn * rvn + re * rve) / v2, 0.0, horizon)
    return t, math.hypot(rn + rvn * t, re + rve * t)


def min_slot_separation(formation: str, spacing: float) -> float:
    """Smallest horizontal distance between any two slots (master = origin)."""
    pts = [(0.0, 0.0)] + [(f * spacing, r * spacing)
                          for f, r, _ in FORMATIONS[formation].values()]
    return min(math.dist(a, b) for a, b in itertools.combinations(pts, 2))


# =============================================================================
# 3. VEHICLE STATE + LINK
# =============================================================================
@dataclass
class DroneState:
    """Latest known state of one vehicle. Times are time.monotonic() at RECEIPT."""
    sysid: int
    lat: Optional[float] = None
    lon: Optional[float] = None
    rel_alt: float = 0.0            # m above its own home (baro-dominated)
    vn: float = 0.0                 # m/s, NED
    ve: float = 0.0
    vd: float = 0.0                 # positive DOWN
    hdg: Optional[float] = None     # rad, compass (nose direction)
    t_pos: float = 0.0
    t_hb: float = 0.0
    mode: str = "UNKNOWN"
    armed: bool = False
    fix_type: int = 0
    sats: int = 0
    hdop: float = 99.0
    t_gps: float = 0.0
    ekf_flags: int = 0
    t_ekf: float = 0.0
    glitch_until: float = 0.0


@dataclass
class Target:
    lat: float
    lon: float
    alt: float      # m, relative to the SLAVE's home (MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
    vn: float       # m/s feed-forward
    ve: float
    vd: float
    yaw: float      # rad


class VehicleLink:
    """One MAVLink connection per vehicle. A background thread parses incoming
    messages into a DroneState; all writes go through a lock so the control
    loop and helper threads can share the connection safely."""

    def __init__(self, sysid: int, url: str, log: logging.Logger):
        self.sysid = sysid
        self.url = url
        self.log = log
        self.conn = mavutil.mavlink_connection(
            url, source_system=SCRIPT_SYSID, source_component=SCRIPT_COMPID)
        self._state = DroneState(sysid)
        self._lock = threading.Lock()
        self._tx = threading.Lock()
        self._stop = threading.Event()
        self._prev_fix = None       # (time_boot_ms, lat, lon, vn, ve) for glitch check
        self._t0 = time.monotonic()
        self._t_glitch_log = 0.0
        self.t_interval_req = 0.0
        self._thread = threading.Thread(target=self._rx_loop, name=f"rx{sysid}",
                                        daemon=True)

    # ---- lifecycle / access -------------------------------------------------
    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def snapshot(self) -> DroneState:
        """Consistent copy of the state (never hand out the live object)."""
        with self._lock:
            return dataclasses.replace(self._state)

    # ---- receive ------------------------------------------------------------
    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                msg = self.conn.recv_match(blocking=True, timeout=0.5)
            except Exception as exc:  # socket hiccup: keep the thread alive
                self.log.debug("rx %d error: %s", self.sysid, exc)
                time.sleep(0.2)
                continue
            if msg is None or msg.get_type() == "BAD_DATA":
                continue
            # Ignore anything not from this vehicle (SiK radios report as
            # sysid 51, other GCSs as 255, etc.)
            if msg.get_srcSystem() != self.sysid:
                continue
            now = time.monotonic()
            mtype = msg.get_type()
            with self._lock:
                s = self._state
                if mtype == "HEARTBEAT":
                    # Only the autopilot's heartbeat defines mode/armed state
                    if (msg.get_srcComponent() != MAV.MAV_COMP_ID_AUTOPILOT1
                            or msg.type == MAV.MAV_TYPE_GCS):
                        continue
                    s.t_hb = now
                    s.mode = mavutil.mode_string_v10(msg)
                    s.armed = bool(msg.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED)
                elif mtype == "GLOBAL_POSITION_INT":
                    self._on_position(s, msg, now)
                elif mtype == "GPS_RAW_INT":
                    s.fix_type = msg.fix_type
                    s.sats = msg.satellites_visible if msg.satellites_visible != 255 else 0
                    s.hdop = msg.eph / 100.0 if msg.eph != 65535 else 99.0
                    s.t_gps = now
                elif mtype == "EKF_STATUS_REPORT":
                    s.ekf_flags = msg.flags
                    s.t_ekf = now
            # Logging outside the lock
            if mtype == "COMMAND_ACK" and msg.result != MAV.MAV_RESULT_ACCEPTED:
                self.log.warning("drone %d rejected command %d (result %d)",
                                 self.sysid, msg.command, msg.result)
            elif mtype == "STATUSTEXT":
                self.log.info("[drone %d] %s", self.sysid, msg.text)

    def _on_position(self, s: DroneState, msg, now: float):
        lat, lon = msg.lat * 1e-7, msg.lon * 1e-7
        if msg.lat == 0 and msg.lon == 0:
            return  # no position estimate yet
        vn, ve, vd = msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0

        # --- GPS jump detection -------------------------------------------
        # Predict where the vehicle should be from the previous fix + its own
        # velocity, using the AUTOPILOT's clock (immune to radio jitter).
        # A large innovation means a GPS glitch / multipath jump.
        if self._prev_fix is not None:
            ptb, plat, plon, pvn, pve = self._prev_fix
            dt = (msg.time_boot_ms - ptb) / 1000.0
            if 0.0 < dt < 1.0:
                dn, de = ll_to_ne(plat, plon, lat, lon)
                innovation = math.hypot(dn - pvn * dt, de - pve * dt)
                if innovation > GLITCH_JUMP_M:
                    s.glitch_until = now + GLITCH_HOLDOFF_S
                    if now - self._t_glitch_log > 2.0:
                        self._t_glitch_log = now
                        self.log.warning("drone %d: position jumped %.1f m in %.2f s "
                                         "(GPS glitch?)", self.sysid, innovation, dt)
        self._prev_fix = (msg.time_boot_ms, lat, lon, vn, ve)

        s.lat, s.lon = lat, lon
        s.rel_alt = msg.relative_alt / 1000.0
        s.vn, s.ve, s.vd = vn, ve, vd
        s.hdg = math.radians(msg.hdg / 100.0) if msg.hdg != 65535 else None
        s.t_pos = now

    # ---- transmit -----------------------------------------------------------
    def _send(self, fn, *args):
        try:
            with self._tx:
                fn(*args)
        except Exception as exc:
            self.log.debug("tx %d failed: %s", self.sysid, exc)

    def send_heartbeat(self):
        self._send(self.conn.mav.heartbeat_send, MAV.MAV_TYPE_GCS,
                   MAV.MAV_AUTOPILOT_INVALID, 0, 0, MAV.MAV_STATE_ACTIVE)

    def command_long(self, cmd, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        self._send(self.conn.mav.command_long_send, self.sysid,
                   MAV.MAV_COMP_ID_AUTOPILOT1, cmd, 0, p1, p2, p3, p4, p5, p6, p7)

    def set_mode(self, name: str):
        self.command_long(MAV.MAV_CMD_DO_SET_MODE,
                          MAV.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, COPTER_MODES[name])

    def arm(self):
        self.command_long(MAV.MAV_CMD_COMPONENT_ARM_DISARM, 1)

    def takeoff(self, alt_m: float):
        self.command_long(MAV.MAV_CMD_NAV_TAKEOFF, p7=alt_m)

    def request_interval(self, msg_id: int, hz: float):
        self.command_long(MAV.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)

    def send_target(self, t: Target):
        """SET_POSITION_TARGET_GLOBAL_INT in GUIDED.

        Position is the goal; velocity is feed-forward so the slave moves WITH
        the master instead of chasing a point that is always 1-2 m away. If
        these messages stop, ArduCopter drops the velocity part after
        GUID_TIMEOUT and holds the last position.
        """
        self._send(self.conn.mav.set_position_target_global_int_send,
                   int((time.monotonic() - self._t0) * 1000) & 0xFFFFFFFF,
                   self.sysid, MAV.MAV_COMP_ID_AUTOPILOT1,
                   MAV.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, TYPE_MASK_POS_VEL_YAW,
                   int(round(t.lat * 1e7)), int(round(t.lon * 1e7)), float(t.alt),
                   float(t.vn), float(t.ve), float(t.vd),
                   0.0, 0.0, 0.0, float(t.yaw), 0.0)


# =============================================================================
# 4. HEALTH CHECKS
# =============================================================================
def hb_ok(s: DroneState, now: float) -> bool:
    return s.t_hb > 0 and now - s.t_hb < HB_TIMEOUT_S


def link_ok(s: DroneState, now: float) -> bool:
    """Heartbeats AND position both fresh."""
    return hb_ok(s, now) and s.lat is not None and now - s.t_pos < POS_TIMEOUT_S


def nav_problem(s: DroneState, now: float) -> Optional[str]:
    """Return None if the vehicle's reported position is trustworthy, else why not."""
    if s.lat is None:
        return "no position"
    if now - s.t_gps > AUX_TIMEOUT_S:
        return "no GPS_RAW_INT"
    if s.fix_type < MIN_FIX_TYPE:
        return f"fix_type {s.fix_type}"
    if s.sats < MIN_SATS:
        return f"{s.sats} sats"
    if s.hdop > MAX_HDOP:
        return f"HDOP {s.hdop:.1f}"
    if now - s.t_ekf > AUX_TIMEOUT_S:
        return "no EKF_STATUS_REPORT"
    f = s.ekf_flags
    if not f & EKF_POS_HORIZ_ABS or f & (EKF_CONST_POS_MODE | EKF_UNINITIALIZED | EKF_GPS_GLITCHING):
        return f"EKF flags 0x{f:04x}"
    if now < s.glitch_until:
        return "recent position jump"
    return None


def master_problem(m: DroneState, now: float) -> Optional[str]:
    """Conditions under which the master can no longer be followed at all."""
    if not link_ok(m, now):
        return "master telemetry lost"
    why = nav_problem(m, now)
    if why:
        return f"master navigation bad ({why})"
    if not m.armed:
        return "master disarmed"
    return None


def airborne(s: DroneState, now: float) -> bool:
    return (s.lat is not None and s.armed and s.rel_alt > AIRBORNE_ALT_M
            and now - s.t_pos < 2 * POS_TIMEOUT_S)


def stopping_point(s: DroneState) -> Tuple[float, float]:
    """Where the vehicle will come to rest if told to stop now (v^2 / 2a ahead).
    Using this as a hold point avoids the 'overshoot and reverse' manoeuvre."""
    v = math.hypot(s.vn, s.ve)
    if v < 0.3:
        return s.lat, s.lon
    d = v * v / (2.0 * HOLD_DECEL_MSS)
    return ne_to_ll(s.lat, s.lon, s.vn / v * d, s.ve / v * d)


def sanity_check_config():
    """Fail fast on configuration mistakes that would silently break layering."""
    for name in FORMATIONS:
        assert set(FORMATIONS[name]) == set(SLAVE_IDS), f"{name}: slot/slave mismatch"
        sep = min_slot_separation(name, DEFAULT_SPACING_M)
        assert sep >= MIN_SLOT_SEPARATION_M, f"{name}: slots only {sep:.1f} m apart"
    layers = list(TRANSIT_LAYER_M.values())
    assert len(set(layers)) == len(layers), "transit layers must be unique"
    assert min(layers) > max(l for f in FORMATIONS.values() for *_, l in f.values()), \
        "transit layers must be above all formation layers"
    assert MASTER_MAX_ALT_M + max(layers) < SLAVE_CEIL_ALT_M, \
        "MASTER_MAX_ALT_M + highest transit layer must stay below SLAVE_CEIL_ALT_M"
    assert len(set(TAKEOFF_ALT_M.values())) == len(TAKEOFF_ALT_M), "takeoff alts must be unique"
    assert MASTER_LOSS_ACTION in ("RTL", "LAND", "NONE")


# =============================================================================
# 5. SWARM CONTROLLER
# =============================================================================
class SwarmController:
    """
    Swarm states:
      IDLE     Script sends heartbeats only. Vehicles are under operator control.
      ENGAGED  Slaves are commanded to their formation slots.
      HOLD     Every GUIDED slave is held at its stopping point. Entered on ANY
               anomaly or by the operator. Leaving HOLD always requires an
               explicit 'engage' - the script never re-engages by itself.

    Per-slave phase (while ENGAGED):
      OUT      not part of the formation (never joined, or detached)
      CLIMB    hold current x/y, change altitude to the slave's unique transit layer
      TRANSIT  fly to the slot x/y ON the transit layer (clear of formation layers)
      IN       in the slot, on its formation layer
    """
    IDLE, ENGAGED, HOLD = "IDLE", "ENGAGED", "HOLD"

    def __init__(self, links: Dict[int, VehicleLink], formation: str, spacing: float,
                 heading_mode: str, dry_run: bool, log: logging.Logger):
        self.links = links
        self.log = log
        self.dry_run = dry_run
        self.formation = formation
        self.spacing = spacing
        self.heading_mode = heading_mode

        self.state = self.IDLE
        self.hold_reason = ""
        self.hold_master_related = False
        self.t_master_bad = 0.0
        self.escalated = False
        self.hold_targets: Dict[int, Target] = {}
        self.escaping: set = set()

        self.phase = {sid: "OUT" for sid in SLAVE_IDS}
        self.anchor: Dict[int, Tuple[float, float]] = {}
        self.slot_err: Dict[int, float] = {}

        self.form_psi: Optional[float] = None   # formation heading (rad)
        self.form_omega = 0.0                   # its rate (rad/s)

        self.hb_suppressed: set = set()
        self.lost_since: Dict[int, float] = {}
        self.min_sep = float("inf")
        self._conflict_log_t: Dict[Tuple[int, int], float] = {}

        self.cmdq: "queue.Queue[str]" = queue.Queue()
        self.quit_evt = threading.Event()
        self._t_hb = 0.0
        self._t_status = 0.0

    # ------------------------------------------------------------------ loop
    def run(self):
        period = 1.0 / CONTROL_RATE_HZ
        next_t = time.monotonic()
        while not self.quit_evt.is_set():
            now = time.monotonic()
            snaps = {sid: l.snapshot() for sid, l in self.links.items()}
            self._process_commands(snaps, now)
            self._housekeeping(snaps, now)
            self._step(snaps, now, period)
            if now - self._t_status >= STATUS_PERIOD_S:
                self._t_status = now
                self.log.info(self._status_line(snaps, now))
            # fixed-rate scheduling without drift
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()

    # ------------------------------------------------------ housekeeping
    def _housekeeping(self, snaps, now):
        # Heartbeats: this is what keeps each slave's GCS failsafe from firing.
        if now - self._t_hb >= HEARTBEAT_PERIOD_S:
            self._t_hb = now
            for sid, link in self.links.items():
                if sid not in self.hb_suppressed:
                    link.send_heartbeat()

        # (Re-)assert telemetry rates; don't trust SRn_ alone, a GCS can change them.
        for sid, link in self.links.items():
            if snaps[sid].t_hb > 0 and now - link.t_interval_req > INTERVAL_REFRESH_S:
                link.t_interval_req = now
                pos_hz = MASTER_POS_RATE_HZ if sid == MASTER_ID else SLAVE_POS_RATE_HZ
                link.request_interval(MAV.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, pos_hz)
                link.request_interval(MAV.MAVLINK_MSG_ID_GPS_RAW_INT, AUX_RATE_HZ)
                link.request_interval(MAV.MAVLINK_MSG_ID_EKF_STATUS_REPORT, AUX_RATE_HZ)

        # Lost-slave handling: if we can't see a slave for SLAVE_LOST_FORCE_FS_S,
        # stop heartbeating it so its onboard GCS failsafe (RTL on its unique
        # layer) definitely fires even if our uplink still reaches it.
        for sid in SLAVE_IDS:
            s = snaps[sid]
            if s.t_hb == 0:
                continue
            if hb_ok(s, now):
                self.lost_since.pop(sid, None)
                if sid in self.hb_suppressed and (s.mode != "GUIDED" or not s.armed):
                    self.hb_suppressed.discard(sid)
                    self.log.warning("slave %d back (mode %s) - heartbeats resumed", sid, s.mode)
            else:
                t0 = self.lost_since.setdefault(sid, now)
                if (s.armed and sid not in self.hb_suppressed
                        and now - t0 > SLAVE_LOST_FORCE_FS_S):
                    self.hb_suppressed.add(sid)
                    self.log.error("slave %d silent for %.0f s - withholding heartbeats "
                                   "so its onboard GCS failsafe fires", sid, now - t0)

    # --------------------------------------------------------------- step
    def _step(self, snaps, now, dt):
        # 1. Hard separation monitor runs in every state.
        self._separation_monitor(snaps, now, act=(self.state != self.IDLE))
        if self.state == self.IDLE:
            return

        # 2. Health checks while engaged.
        if self.state == self.ENGAGED:
            self._detach_departed_slaves(snaps, now)
            problem = self._swarm_problem(snaps, now)
            if problem:
                reason, master_related, extra_alt = problem
                self._enter_hold(reason, snaps, now, master_related, extra_alt)
            elif all(p == "OUT" for p in self.phase.values()):
                self.log.warning("no slaves left in formation -> IDLE")
                self.state = self.IDLE
                return

        # 3. Escalation if the master stays broken during HOLD.
        if self.state == self.HOLD:
            self._escalate_if_needed(snaps, now)
            if self.state != self.HOLD:
                return

        # 4. Build targets.
        if self.state == self.ENGAGED:
            targets = self._formation_targets(snaps, now, dt)
            targets = self._deconflict(targets, snaps, now)
        else:
            targets = dict(self.hold_targets)

        # 5. Send, but only to slaves that are actually ours to command.
        for sid, tgt in targets.items():
            s = snaps[sid]
            if link_ok(s, now) and s.armed and s.mode == "GUIDED" and s.rel_alt > AIRBORNE_ALT_M:
                if not self.dry_run:
                    self.links[sid].send_target(tgt)
                self.log.debug("tgt %d: %.7f %.7f %.1f v(%.1f,%.1f,%.1f) yaw %.0f", sid,
                               tgt.lat, tgt.lon, tgt.alt, tgt.vn, tgt.ve, tgt.vd,
                               math.degrees(tgt.yaw))

    # ------------------------------------------------------ health / hold
    def _detach_departed_slaves(self, snaps, now):
        """A slave that left GUIDED (its own failsafe, fence, safety switch) is
        no longer ours. Stop commanding it; it stays an obstacle for others."""
        for sid in SLAVE_IDS:
            s = snaps[sid]
            if self.phase[sid] != "OUT" and hb_ok(s, now) and (s.mode != "GUIDED" or not s.armed):
                self.log.warning("slave %d left GUIDED (mode %s, armed %s) - detached",
                                 sid, s.mode, s.armed)
                self.phase[sid] = "OUT"

    def _swarm_problem(self, snaps, now):
        """Return (reason, master_related, extra_alt) if the swarm must HOLD."""
        m = snaps[MASTER_ID]
        why = master_problem(m, now)
        if why:
            return why, True, 0.0
        if m.mode in MASTER_RETURNING_MODES:
            # Master is coming home through the airspace below the slaves:
            # hold the slaves and lift them clear of its return path.
            return f"master in {m.mode}", False, MASTER_RTL_CLEARANCE_M
        if not MASTER_MIN_ALT_M <= m.rel_alt <= MASTER_MAX_ALT_M:
            return (f"master altitude {m.rel_alt:.1f} m outside "
                    f"{MASTER_MIN_ALT_M:.0f}-{MASTER_MAX_ALT_M:.0f} m envelope"), False, 0.0
        for sid in SLAVE_IDS:
            if self.phase[sid] == "OUT":
                continue
            s = snaps[sid]
            if not link_ok(s, now):
                return f"slave {sid} telemetry lost", False, 0.0
            why = nav_problem(s, now)
            if why:
                return f"slave {sid} navigation bad ({why})", False, 0.0
        return None

    def _enter_hold(self, reason, snaps, now, master_related=False, extra_alt=0.0):
        if self.state == self.HOLD:
            if master_related and not self.hold_master_related:
                self.hold_master_related = True
                self.t_master_bad = now
            return
        self.state = self.HOLD
        self.hold_reason = reason
        self.hold_master_related = master_related
        self.t_master_bad = now
        self.escalated = False
        self.hold_targets = {}
        for sid in SLAVE_IDS:
            s = snaps[sid]
            if s.lat is None:
                continue
            lat, lon = stopping_point(s)
            alt = clamp(s.rel_alt + extra_alt, SLAVE_FLOOR_ALT_M, SLAVE_CEIL_ALT_M)
            yaw = self.form_psi if self.form_psi is not None else (s.hdg or 0.0)
            self.hold_targets[sid] = Target(lat, lon, alt, 0.0, 0.0, 0.0, yaw)
        self.log.error("*** SWARM HOLD: %s *** (type 'engage' to resume when safe)", reason)

    def _escalate_if_needed(self, snaps, now):
        """If the master stays unusable, bring the slaves home on their own layers
        rather than leaving them hovering until their batteries run out."""
        if self.escalated or not self.hold_master_related:
            return
        if master_problem(snaps[MASTER_ID], now) is None:
            self.t_master_bad = now          # recovered: timer restarts
            return
        if now - self.t_master_bad < MASTER_LOSS_DELAY_S:
            return
        self.escalated = True
        if MASTER_LOSS_ACTION == "NONE":
            self.log.error("master still unusable after %.0f s - slaves keep holding",
                           MASTER_LOSS_DELAY_S)
            return
        ids = [sid for sid in SLAVE_IDS
               if snaps[sid].armed and snaps[sid].mode == "GUIDED"]
        self.log.error("master unusable for %.0f s -> %s for slaves %s (staggered)",
                       MASTER_LOSS_DELAY_S, MASTER_LOSS_ACTION, ids)
        self.state = self.IDLE
        for sid in SLAVE_IDS:
            self.phase[sid] = "OUT"
        self._staggered_mode(ids, MASTER_LOSS_ACTION)

    # ------------------------------------------------------ formation math
    def _update_heading(self, m: DroneState, dt: float):
        """Formation heading, slew-rate limited.

        'course'  - direction of travel (atan2(vE, vN)); frozen when slow.
        'heading' - the master's nose (hdg). Yawing in a hover swings the swarm.
        'fixed'   - whatever it was when engaged.
        Rate limiting matters: a slave at radius r moves omega*r just from
        rotation, on top of the master's own speed.
        """
        desired = None
        if self.heading_mode == "course":
            if math.hypot(m.vn, m.ve) >= COURSE_MIN_SPEED_MS:
                desired = math.atan2(m.ve, m.vn)
        elif self.heading_mode == "heading":
            desired = m.hdg
        if self.form_psi is None:
            self.form_psi = desired if desired is not None else (m.hdg or 0.0)
            self.form_omega = 0.0
            return
        if desired is None:
            self.form_omega = 0.0
            return
        max_step = math.radians(MAX_FORMATION_ROT_DPS) * dt
        step = clamp(wrap_pi(desired - self.form_psi), -max_step, max_step)
        self.form_psi = wrap_pi(self.form_psi + step)
        self.form_omega = step / dt

    def _formation_targets(self, snaps, now, dt) -> Dict[int, Target]:
        m = snaps[MASTER_ID]

        # Latency compensation: the master's report is already ~age seconds old
        # and our command will take LINK_LATENCY_S to act -> project it forward.
        age = clamp(now - m.t_pos + LINK_LATENCY_S, 0.0, MAX_EXTRAPOLATION_S)
        mlat, mlon = ne_to_ll(m.lat, m.lon, m.vn * age, m.ve * age)
        malt = m.rel_alt - m.vd * age          # vd is positive down

        self._update_heading(m, dt)
        psi, omega = self.form_psi, self.form_omega
        slots = FORMATIONS[self.formation]
        out: Dict[int, Target] = {}

        for sid in SLAVE_IDS:
            ph = self.phase[sid]
            s = snaps[sid]
            if ph == "OUT" or s.lat is None:
                continue
            fwd, right, layer = slots[sid]

            # Slot position: rotate body offset by formation heading, add to master.
            dn, de = body_to_ne(fwd * self.spacing, right * self.spacing, psi)
            slat, slon = ne_to_ll(mlat, mlon, dn, de)

            # Slot velocity = master velocity + omega x r (rotation of the offset).
            # d/dpsi of (dn, de) = (-de, dn), so v_rot = omega * (-de, dn).
            vn, ve = m.vn - omega * de, m.ve + omega * dn
            sp = math.hypot(vn, ve)
            if sp > MAX_FF_SPEED_MS:
                vn, ve = vn * MAX_FF_SPEED_MS / sp, ve * MAX_FF_SPEED_MS / sp

            raw_slot_alt = malt + layer
            slot_alt = clamp(raw_slot_alt, SLAVE_FLOOR_ALT_M, SLAVE_CEIL_ALT_M)
            vd = 0.0 if slot_alt != raw_slot_alt else clamp(m.vd, -MAX_FF_VZ_MS, MAX_FF_VZ_MS)
            transit_alt = clamp(malt + TRANSIT_LAYER_M[sid], SLAVE_FLOOR_ALT_M, SLAVE_CEIL_ALT_M)

            en, ee = ll_to_ne(s.lat, s.lon, slat, slon)
            err = math.hypot(en, ee)
            self.slot_err[sid] = err

            if ph == "CLIMB":
                # Vertical move only, in place, to this slave's private layer.
                alat, alon = self.anchor[sid]
                tgt = Target(alat, alon, transit_alt, 0.0, 0.0, 0.0, psi)
                if abs(s.rel_alt - transit_alt) < ALT_REACHED_M:
                    self._set_phase(sid, "TRANSIT")
            elif ph == "TRANSIT":
                # Horizontal move above the formation layers.
                tgt = Target(slat, slon, transit_alt, vn, ve, vd, psi)
                if err < JOIN_CAPTURE_M:
                    self._set_phase(sid, "IN")
            else:  # IN
                tgt = Target(slat, slon, slot_alt, vn, ve, vd, psi)
                if err > FALLOUT_ERR_M:
                    # Badly out of position: step UP out of the formation plane
                    # and re-join, rather than cutting across other slots.
                    self.anchor[sid] = stopping_point(s)
                    self._set_phase(sid, "CLIMB")
                    self.log.warning("slave %d %.1f m out of slot - re-joining via "
                                     "transit layer", sid, err)
            out[sid] = tgt
        return out

    def _set_phase(self, sid, ph):
        if self.phase[sid] != ph:
            self.log.info("slave %d: %s -> %s", sid, self.phase[sid], ph)
            self.phase[sid] = ph

    # ------------------------------------------------ collision avoidance
    def _deconflict(self, targets: Dict[int, Target], snaps, now) -> Dict[int, Target]:
        """Soft layer: push a slave's TARGET out of the SOFT_R_M bubble around
        (a) the master and any drone we are not commanding, projected
        LOOKAHEAD_S ahead, and (b) targets of higher-priority slaves (lower
        SYSID wins). In a healthy formation this never triggers - slots are
        further apart than SOFT_R_M. It exists for detached slaves, re-joins
        and a master that wanders into the formation.
        """
        m = snaps[MASTER_ID]
        ref_lat, ref_lon = m.lat, m.lon
        obstacles = []   # (label, n, e, alt)
        for sid, s in snaps.items():
            if not airborne(s, now) or (sid in targets):
                continue
            n, e = ll_to_ne(ref_lat, ref_lon, s.lat, s.lon)
            obstacles.append((sid, n + s.vn * LOOKAHEAD_S, e + s.ve * LOOKAHEAD_S,
                              s.rel_alt - s.vd * LOOKAHEAD_S))
        placed = []
        for sid in sorted(targets):
            t = targets[sid]
            tn, te = ll_to_ne(ref_lat, ref_lon, t.lat, t.lon)
            for label, on, oe, oalt in obstacles + placed:
                if abs(t.alt - oalt) >= SOFT_V_M:
                    continue
                dn, de = tn - on, te - oe
                d = math.hypot(dn, de)
                if d >= SOFT_R_M:
                    continue
                if d < 0.1:  # exactly on top: push to formation's right
                    dn, de, d = -math.sin(self.form_psi), math.cos(self.form_psi), 1.0
                push = SOFT_R_M - d
                tn += dn / d * push
                te += de / d * push
                self.log.debug("slave %d target pushed %.1f m away from %s", sid, push, label)
            t.lat, t.lon = ne_to_ll(ref_lat, ref_lon, tn, te)
            placed.append((sid, tn, te, t.alt))
        return targets

    def _separation_monitor(self, snaps, now, act: bool):
        """Hard layer: predict closest point of approach for every airborne pair.

        Conflict = predicted horizontal miss < CRIT_H_M within CPA_HORIZON_S AND
        predicted vertical gap < CRIT_V_M. Response: swarm HOLD; the HIGHER
        aircraft of the pair climbs ESCAPE_CLIMB_M (if it's a slave we command),
        the lower one brakes. Climbing is chosen because the master is always
        the lowest aircraft and the ground is below everyone.
        """
        air = {sid: s for sid, s in snaps.items() if airborne(s, now)}
        if len(air) < 2:
            self.min_sep = float("inf")
            return
        ref = next(iter(air.values()))
        ne = {sid: ll_to_ne(ref.lat, ref.lon, s.lat, s.lon) for sid, s in air.items()}
        min_sep = float("inf")
        conflicts = []
        for a, b in itertools.combinations(sorted(air), 2):
            sa, sb = air[a], air[b]
            rn, re = ne[b][0] - ne[a][0], ne[b][1] - ne[a][1]
            # reported figure is 3-D (layers count); the conflict test below
            # treats horizontal and vertical separately
            min_sep = min(min_sep, math.hypot(rn, re, sb.rel_alt - sa.rel_alt))
            t, miss = closest_approach(rn, re, sb.vn - sa.vn, sb.ve - sa.ve, CPA_HORIZON_S)
            dz = abs((sb.rel_alt - sb.vd * t) - (sa.rel_alt - sa.vd * t))
            if miss < CRIT_H_M and dz < CRIT_V_M:
                conflicts.append((a, b, t, miss, dz))
        self.min_sep = min_sep

        for a, b, t, miss, dz in conflicts:
            if now - self._conflict_log_t.get((a, b), 0.0) > 2.0:
                self._conflict_log_t[(a, b)] = now
                self.log.error("SEPARATION CONFLICT %d<->%d: miss %.1f m / dz %.1f m in %.1f s",
                               a, b, miss, dz, t)
            if not act:
                continue
            self._enter_hold(f"separation conflict {a}<->{b}", snaps, now)
            hi, lo = (a, b) if air[a].rel_alt >= air[b].rel_alt else (b, a)
            if hi in SLAVE_IDS and hi not in self.escaping:
                s = air[hi]
                lat, lon = stopping_point(s)
                alt = clamp(s.rel_alt + ESCAPE_CLIMB_M, SLAVE_FLOOR_ALT_M, SLAVE_CEIL_ALT_M)
                self.hold_targets[hi] = Target(lat, lon, alt, 0, 0, 0, self.form_psi or 0.0)
                self.escaping.add(hi)
                self.log.error("slave %d escaping upward to %.1f m; %d brakes", hi, alt, lo)
            elif hi == MASTER_ID:
                self.log.error("PILOT: master is ABOVE slave %d - climb or steer away!", lo)

    # ---------------------------------------------------- sequenced actions
    def _staggered_mode(self, ids, mode):
        """Mode changes one slave at a time so they don't all start climbing /
        crossing at once. Runs in its own thread (the control loop keeps going)."""
        def worker():
            for i, sid in enumerate(ids):
                if i:
                    time.sleep(STAGGER_S)
                self.log.warning("slave %d -> %s", sid, mode)
                self.links[sid].set_mode(mode)
        threading.Thread(target=worker, daemon=True).start()

    def _wait_for(self, sid, pred, timeout) -> bool:
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            if pred(self.links[sid].snapshot()):
                return True
            time.sleep(0.1)
        return False

    def _takeoff_sequence(self, ids):
        def worker():
            for sid in ids:
                link, s, now = self.links[sid], self.links[sid].snapshot(), time.monotonic()
                if s.armed:
                    self.log.info("slave %d already armed - skipped", sid)
                    continue
                why = None if link_ok(s, now) else "no telemetry"
                why = why or nav_problem(s, now)
                if why:
                    self.log.error("slave %d takeoff refused: %s", sid, why)
                    continue
                link.set_mode("GUIDED")
                if not self._wait_for(sid, lambda x: x.mode == "GUIDED", 3.0):
                    self.log.error("slave %d did not enter GUIDED - sequence stopped", sid)
                    return
                link.arm()
                if not self._wait_for(sid, lambda x: x.armed, 5.0):
                    self.log.error("slave %d did not arm (check pre-arm messages) - "
                                   "sequence stopped", sid)
                    return
                alt = TAKEOFF_ALT_M[sid]
                link.takeoff(alt)
                self.log.info("slave %d taking off to %.0f m", sid, alt)
                if not self._wait_for(sid, lambda x: x.rel_alt > 0.8 * alt, 30.0):
                    self.log.error("slave %d did not climb - sequence stopped", sid)
                    return
                time.sleep(STAGGER_S)
            self.log.info("takeoff sequence complete")
        threading.Thread(target=worker, daemon=True).start()

    # ---------------------------------------------------- operator commands
    def _parse_ids(self, args):
        if not args or args[0] == "all":
            return list(SLAVE_IDS)
        ids = [int(a) for a in args]
        bad = [i for i in ids if i not in SLAVE_IDS]
        if bad:
            raise ValueError(f"not a slave id: {bad}")
        return ids

    def _process_commands(self, snaps, now):
        while True:
            try:
                line = self.cmdq.get_nowait().strip()
            except queue.Empty:
                return
            if not line:
                continue
            try:
                self._handle(line, snaps, now)
            except (ValueError, IndexError) as exc:
                self.log.error("bad command '%s': %s", line, exc)

    def _rejoin_all(self, snaps):
        """Formation geometry changed: send every in-formation slave back through
        its transit layer so re-slotting never crosses the formation plane."""
        for sid in SLAVE_IDS:
            if self.phase[sid] != "OUT" and snaps[sid].lat is not None:
                self.anchor[sid] = stopping_point(snaps[sid])
                self._set_phase(sid, "CLIMB")

    def _handle(self, line, snaps, now):
        cmd, *args = line.split()
        cmd = cmd.lower()

        if cmd == "help":
            self.log.info(HELP_TEXT)
        elif cmd == "status":
            self.log.info(self._status_line(snaps, now))
            for sid, s in snaps.items():
                self.log.info("  %d: mode=%s armed=%s alt=%.1f fix=%d sats=%d hdop=%.1f "
                              "ekf=0x%04x link=%s nav=%s", sid, s.mode, s.armed, s.rel_alt,
                              s.fix_type, s.sats, s.hdop, s.ekf_flags, link_ok(s, now),
                              nav_problem(s, now) or "ok")
        elif cmd == "engage":
            self._cmd_engage(self._parse_ids(args), snaps, now)
        elif cmd == "hold":
            if self.state == self.IDLE:
                self.log.warning("not engaged; nothing to hold")
            else:
                self._enter_hold("operator", snaps, now)
        elif cmd in ("idle", "release"):
            self.state = self.IDLE
            self.log.warning("IDLE: no more targets. GUIDED slaves hold their last "
                             "position target. Heartbeats continue.")
        elif cmd == "formation":
            name = args[0].lower()
            if name not in FORMATIONS:
                raise ValueError(f"choose from {list(FORMATIONS)}")
            sep = min_slot_separation(name, self.spacing)
            if sep < MIN_SLOT_SEPARATION_M:
                raise ValueError(f"slots would be only {sep:.1f} m apart")
            self.formation = name
            self.log.info("formation -> %s", name)
            if self.state == self.ENGAGED:
                self._rejoin_all(snaps)
        elif cmd == "spacing":
            sp = float(args[0])
            if sp < MIN_SPACING_M or min_slot_separation(self.formation, sp) < MIN_SLOT_SEPARATION_M:
                raise ValueError(f"spacing must keep slots >= {MIN_SLOT_SEPARATION_M} m apart")
            self.spacing = sp
            self.log.info("spacing -> %.1f m", sp)
            if self.state == self.ENGAGED:
                self._rejoin_all(snaps)
        elif cmd == "heading":
            mode = args[0].lower()
            if mode not in HEADING_MODES:
                raise ValueError(f"choose from {HEADING_MODES}")
            self.heading_mode = mode
            self.log.info("heading mode -> %s", mode)
        elif cmd == "takeoff":
            self._takeoff_sequence(self._parse_ids(args))
        elif cmd in ("rtl", "land"):
            ids = self._parse_ids(args)
            if len(ids) == len(SLAVE_IDS) or all(
                    self.phase[s] == "OUT" for s in SLAVE_IDS if s not in ids):
                self.state = self.IDLE
            for sid in ids:
                self.phase[sid] = "OUT"
            self._staggered_mode(ids, cmd.upper())
        elif cmd in ("quit", "quit!"):
            flying = [sid for sid in SLAVE_IDS if airborne(snaps[sid], now)]
            if flying and cmd == "quit":
                self.log.error("slaves %s are airborne. Land them first, or 'quit!' to "
                               "exit anyway (their GCS failsafe will trigger RTL).", flying)
                return
            self.quit_evt.set()
        else:
            raise ValueError("unknown command (type 'help')")

    def _cmd_engage(self, ids, snaps, now):
        m = snaps[MASTER_ID]
        why = master_problem(m, now)
        if why:
            self.log.error("engage refused: %s", why)
            return
        if not MASTER_MIN_ALT_M <= m.rel_alt <= MASTER_MAX_ALT_M:
            self.log.error("engage refused: master at %.1f m, must be %.0f-%.0f m",
                           m.rel_alt, MASTER_MIN_ALT_M, MASTER_MAX_ALT_M)
            return
        if m.mode in MASTER_RETURNING_MODES:
            self.log.error("engage refused: master is in %s", m.mode)
            return

        fresh_engage = self.state != self.ENGAGED
        if fresh_engage:
            # From IDLE or HOLD: re-evaluate every slave from scratch.
            for sid in SLAVE_IDS:
                self.phase[sid] = "OUT"
            self.form_psi = None
            self.escaping.clear()
            self.hold_targets = {}

        joined = []
        for sid in ids:
            s = snaps[sid]
            problems = []
            if not link_ok(s, now):
                problems.append("no telemetry")
            else:
                if nav_problem(s, now):
                    problems.append(nav_problem(s, now))
                if not s.armed or s.mode != "GUIDED":
                    problems.append(f"needs armed+GUIDED (is {s.mode})")
                if s.rel_alt < AIRBORNE_ALT_M:
                    problems.append("not airborne")
            if problems:
                self.log.warning("slave %d not joined: %s", sid, ", ".join(problems))
                continue
            if self.phase[sid] == "OUT":
                self.anchor[sid] = stopping_point(s)
                self._set_phase(sid, "CLIMB")
                joined.append(sid)
        if not joined and fresh_engage:
            self.log.error("engage refused: no eligible slaves")
            return
        self.state = self.ENGAGED
        self.hold_reason = ""
        self.log.warning("ENGAGED: formation=%s spacing=%.0f m heading=%s; joining %s",
                         self.formation, self.spacing, self.heading_mode, joined)

    # ------------------------------------------------------------ status
    def _status_line(self, snaps, now) -> str:
        m = snaps[MASTER_ID]
        psi = f"{math.degrees(self.form_psi) % 360:.0f}" if self.form_psi is not None else "-"
        hdr = f"[{self.state}" + (f": {self.hold_reason}" if self.state == self.HOLD else "")
        hdr += f" | {self.formation} {self.spacing:.0f}m psi={psi}]"
        mp = master_problem(m, now)
        parts = [hdr, f"M:{m.mode} {m.rel_alt:.1f}m {math.hypot(m.vn, m.ve):.1f}m/s "
                      f"{'OK' if mp is None else 'BAD'}"]
        for sid in SLAVE_IDS:
            s = snaps[sid]
            if not hb_ok(s, now):
                parts.append(f"{sid}:LOST" + ("(noHB)" if sid in self.hb_suppressed else ""))
                continue
            err = self.slot_err.get(sid)
            e = f" e{err:.1f}" if err is not None and self.phase[sid] != "OUT" else ""
            parts.append(f"{sid}:{self.phase[sid]}/{s.mode} {s.rel_alt:.1f}m{e}")
        sep = "-" if math.isinf(self.min_sep) else f"{self.min_sep:.1f}m"
        parts.append(f"min 3D sep {sep}")
        return " | ".join(parts)


HELP_TEXT = """
Commands:
  status                     detailed state of every drone
  takeoff [all|ids]          GUIDED + arm + takeoff, one slave at a time, unique altitudes
  engage [all|ids]           join slaves to the formation (repeat to add more later)
  hold                       freeze all slaves at their stopping points
  idle                       stop sending targets (slaves hold last target)
  formation v|line|trail|diamond
  spacing <m>                formation spacing (>= 8 m)
  heading course|heading|fixed
  rtl [all|ids]              staggered RTL (unique RTL_ALT per slave)
  land [all|ids]             staggered LAND in place
  quit / quit!               exit (quit! even with slaves airborne)
Safety: after any HOLD the swarm stays held until you type 'engage'."""


# =============================================================================
# 6. ENTRY POINT
# =============================================================================
def parse_links(spec: Optional[str]) -> Dict[int, str]:
    """'1=udpin:127.0.0.1:14571,2=...' -> {1: 'udpin:...', ...}"""
    if not spec:
        return dict(DEFAULT_LINKS)
    links = {}
    for item in spec.split(","):
        sid, url = item.split("=", 1)
        links[int(sid)] = url
    if set(links) != {MASTER_ID, *SLAVE_IDS}:
        raise SystemExit(f"--links must define sysids {MASTER_ID} and {SLAVE_IDS}")
    return links


def cli_reader(ctrl: SwarmController):
    while not ctrl.quit_evt.is_set():
        try:
            line = input()
        except EOFError:
            return
        ctrl.cmdq.put(line)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--links", help="override: '1=udpin:127.0.0.1:14571,2=...'")
    ap.add_argument("--formation", default=DEFAULT_FORMATION, choices=list(FORMATIONS))
    ap.add_argument("--spacing", type=float, default=DEFAULT_SPACING_M)
    ap.add_argument("--heading-mode", default=DEFAULT_HEADING_MODE, choices=HEADING_MODES)
    ap.add_argument("--dry-run", action="store_true",
                    help="run everything but never send position targets")
    ap.add_argument("--log", default=time.strftime("swarm_%Y%m%d_%H%M%S.log"))
    args = ap.parse_args()

    sanity_check_config()
    if min_slot_separation(args.formation, args.spacing) < MIN_SLOT_SEPARATION_M:
        raise SystemExit("spacing too small for this formation")

    log = logging.getLogger("swarm")
    log.setLevel(logging.DEBUG)
    fh = logging.FileHandler(args.log)
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter("%(asctime)s.%(msecs)03d %(levelname)s %(message)s",
                                      "%H:%M:%S"))
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(logging.INFO)
    ch.setFormatter(logging.Formatter("%(asctime)s %(levelname).1s %(message)s", "%H:%M:%S"))
    log.addHandler(fh)
    log.addHandler(ch)

    links = {sid: VehicleLink(sid, url, log) for sid, url in parse_links(args.links).items()}
    for link in links.values():
        link.start()
    ctrl = SwarmController(links, args.formation, args.spacing, args.heading_mode,
                           args.dry_run, log)
    threading.Thread(target=cli_reader, args=(ctrl,), daemon=True).start()
    log.info("swarm_gcs up (sysid %d%s). Log: %s. Type 'help'.",
             SCRIPT_SYSID, ", DRY RUN" if args.dry_run else "", args.log)

    while not ctrl.quit_evt.is_set():
        try:
            ctrl.run()
        except KeyboardInterrupt:
            # Ctrl-C must not silently drop the swarm: hold, and require 'quit!'
            snaps = {sid: l.snapshot() for sid, l in links.items()}
            if ctrl.state == ctrl.ENGAGED:
                ctrl._enter_hold("operator Ctrl-C", snaps, time.monotonic())
            log.error("Ctrl-C caught - swarm held. Type 'quit!' to really exit.")
    for link in links.values():
        link.stop()
    log.warning("exited - heartbeats stopped; airborne slaves will run their GCS failsafe")


if __name__ == "__main__":
    main()
