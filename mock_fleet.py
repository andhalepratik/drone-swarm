#!/usr/bin/env python3
"""
mock_fleet.py - crude kinematic stand-in for 5 ArduCopters.

Purpose: exercise swarm_gcs.py's plumbing and logic on a laptop in seconds
(message parsing, state machine, failsafe paths). It is NOT a flight-dynamics
simulator - use ArduPilot SITL (sitl_swarm.sh) for that.

The master flies a 60 m square at 4 m/s in "LOITER". Slaves start airborne in
GUIDED over their pads and obey SET_POSITION_TARGET_GLOBAL_INT, DO_SET_MODE,
ARM and TAKEOFF, with a GCS-heartbeat failsafe (-> RTL) like the real thing.

  python3 mock_fleet.py                                  # nominal
  python3 mock_fleet.py --master-gps-fail-at 60          # master loses GPS at t=60 s
  python3 mock_fleet.py --drop-link 3@50                 # slave 3 radio dies at t=50 s
"""
import argparse
import math
import random
import time

from pymavlink import mavutil

MAV = mavutil.mavlink
HOME_LAT, HOME_LON = -35.3632621, 149.1652374   # ArduPilot SITL default (CMAC)
R = 6378137.0
MODE_IDS = {"STABILIZE": 0, "AUTO": 3, "GUIDED": 4, "LOITER": 5, "RTL": 6,
            "LAND": 9, "BRAKE": 17}
MODE_NAMES = {v: k for k, v in MODE_IDS.items()}
GCS_SYSID, GCS_FS_TIMEOUT = 250, 5.0
RTL_ALT = {1: 15.0, 2: 50.0, 3: 55.0, 4: 60.0, 5: 65.0}


def ne_to_ll(n, e):
    return (HOME_LAT + math.degrees(n / R),
            HOME_LON + math.degrees(e / (R * math.cos(math.radians(HOME_LAT)))))


def ll_to_ne(lat, lon):
    return (math.radians(lat - HOME_LAT) * R,
            math.radians(lon - HOME_LON) * R * math.cos(math.radians(HOME_LAT)))


class MockCopter:
    VMAX_H, VMAX_UP, VMAX_DN, AMAX = 10.0, 2.5, 1.5, 3.0

    def __init__(self, sysid, pad_e):
        self.id = sysid
        self.conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{14570 + sysid}",
                                               source_system=sysid, source_component=1)
        self.home = (0.0, pad_e)
        self.n, self.e = self.home
        self.alt = 15.0 if sysid == 1 else 8.0 + 3.0 * (sysid - 2)
        self.vn = self.ve = self.vd = 0.0
        self.mode = "LOITER" if sysid == 1 else "GUIDED"
        self.armed = True
        self.tgt = (self.n, self.e, self.alt)
        self.ff = (0.0, 0.0, 0.0)
        self.t_tgt = 0.0
        self.t_gcs_hb = time.monotonic()
        self.gps_ok = True
        self.link_up = True
        self.noise = [0.0, 0.0]
        self.wp_i = 0
        self.boot = time.monotonic()
        self.t_last = {"hb": 0.0, "pos": 0.0, "aux": 0.0}

    # ------------------------------------------------------------ comms
    def rx(self, now):
        while True:
            m = self.conn.recv_match(blocking=False)
            if m is None:
                return
            if not self.link_up:
                continue
            t = m.get_type()
            if t == "HEARTBEAT" and m.get_srcSystem() == GCS_SYSID:
                self.t_gcs_hb = now
            elif t == "COMMAND_LONG" and m.target_system == self.id:
                res = MAV.MAV_RESULT_ACCEPTED
                if m.command == MAV.MAV_CMD_DO_SET_MODE:
                    self.mode = MODE_NAMES.get(int(m.param2), self.mode)
                    if self.mode == "GUIDED":
                        self.tgt, self.ff = (self.n, self.e, self.alt), (0, 0, 0)
                elif m.command == MAV.MAV_CMD_COMPONENT_ARM_DISARM:
                    self.armed = m.param1 == 1
                elif m.command == MAV.MAV_CMD_NAV_TAKEOFF:
                    self.tgt = (self.n, self.e, m.param7)
                elif m.command == MAV.MAV_CMD_SET_MESSAGE_INTERVAL:
                    pass
                self.conn.mav.command_ack_send(m.command, res)
            elif t == "SET_POSITION_TARGET_GLOBAL_INT" and m.target_system == self.id:
                if self.mode == "GUIDED":
                    n, e = ll_to_ne(m.lat_int * 1e-7, m.lon_int * 1e-7)
                    self.tgt, self.ff, self.t_tgt = (n, e, m.alt), (m.vx, m.vy, m.vz), now

    def tx(self, now):
        if not self.link_up:
            return
        if now - self.t_last["hb"] >= 1.0:
            self.t_last["hb"] = now
            base = MAV.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (MAV.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0)
            self.conn.mav.heartbeat_send(MAV.MAV_TYPE_QUADROTOR, MAV.MAV_AUTOPILOT_ARDUPILOTMEGA,
                                         base, MODE_IDS[self.mode], MAV.MAV_STATE_ACTIVE)
        if now - self.t_last["pos"] >= 0.1:
            self.t_last["pos"] = now
            # small correlated GPS noise; a big random walk when GPS has failed
            k = 3.0 if not self.gps_ok else 0.05
            self.noise = [x * 0.98 + random.gauss(0, k) for x in self.noise]
            lat, lon = ne_to_ll(self.n + self.noise[0], self.e + self.noise[1])
            hdg = int(math.degrees(math.atan2(self.ve, self.vn)) % 360 * 100)
            self.conn.mav.global_position_int_send(
                int((now - self.boot) * 1000), int(lat * 1e7), int(lon * 1e7), 0,
                int(self.alt * 1000), int(self.vn * 100), int(self.ve * 100),
                int(self.vd * 100), hdg)
        if now - self.t_last["aux"] >= 0.5:
            self.t_last["aux"] = now
            fix, sats, eph = (3, 16, 70) if self.gps_ok else (1, 3, 900)
            self.conn.mav.gps_raw_int_send(0, fix, 0, 0, 0, eph, 65535, 0, 65535, sats)
            flags = 0x1F | 0x20 if self.gps_ok else 0x07 | 0x80
            self.conn.mav.ekf_status_report_send(flags, 0.1, 0.1, 0.1, 0.1, 0.0)

    # ------------------------------------------------------------ dynamics
    def step(self, now, dt):
        # onboard GCS failsafe (slaves only, like FS_GCS_ENABLE=1)
        if (self.id != 1 and self.armed and self.mode == "GUIDED" and self.alt > 1
                and now - self.t_gcs_hb > GCS_FS_TIMEOUT):
            print(f"[mock {self.id}] GCS failsafe -> RTL")
            self.mode = "RTL"
        want = (0.0, 0.0, 0.0)
        if not self.armed:
            pass
        elif self.id == 1 and self.mode == "LOITER":         # scripted pilot
            wps = [(0, 0), (60, 0), (60, 60), (0, 60)]
            wn, we = wps[self.wp_i % 4]
            dn, de = wn - self.n, we - self.e
            d = math.hypot(dn, de)
            if d < 2.0:
                self.wp_i += 1
            sp = min(4.0, 0.5 * d)
            want = (dn / max(d, 1e-6) * sp, de / max(d, 1e-6) * sp, 0.0)
        elif self.mode == "GUIDED":
            ff = self.ff if now - self.t_tgt < 3.0 else (0.0, 0.0, 0.0)  # GUID_TIMEOUT
            tn, te, ta = self.tgt
            want = (ff[0] + 0.8 * (tn - self.n), ff[1] + 0.8 * (te - self.e),
                    ff[2] - 0.8 * (ta - self.alt))
        elif self.mode == "RTL":
            if self.alt < RTL_ALT[self.id] - 0.5 and math.hypot(self.n - self.home[0], self.e - self.home[1]) > 2:
                want = (0, 0, -self.VMAX_UP)
            elif math.hypot(self.n - self.home[0], self.e - self.home[1]) > 1:
                dn, de = self.home[0] - self.n, self.home[1] - self.e
                d = math.hypot(dn, de)
                want = (dn / d * min(5, d), de / d * min(5, d), 0)
            else:
                want = (0, 0, 1.0)
        elif self.mode == "LAND":
            want = (0, 0, 1.0)
        # limits
        h = math.hypot(want[0], want[1])
        if h > self.VMAX_H:
            want = (want[0] / h * self.VMAX_H, want[1] / h * self.VMAX_H, want[2])
        wvd = max(-self.VMAX_UP, min(self.VMAX_DN, want[2]))
        dvn, dve = want[0] - self.vn, want[1] - self.ve
        dv = math.hypot(dvn, dve)
        if dv > self.AMAX * dt:
            dvn, dve = dvn / dv * self.AMAX * dt, dve / dv * self.AMAX * dt
        self.vn += dvn
        self.ve += dve
        self.vd += max(-2 * dt, min(2 * dt, wvd - self.vd))
        self.n += self.vn * dt
        self.e += self.ve * dt
        self.alt = max(0.0, self.alt - self.vd * dt)
        if self.alt == 0.0 and self.mode in ("RTL", "LAND") and self.armed:
            self.armed = False
            self.vn = self.ve = self.vd = 0.0
            print(f"[mock {self.id}] landed, disarmed")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=600)
    ap.add_argument("--master-gps-fail-at", type=float)
    ap.add_argument("--drop-link", help="SYSID@SECONDS")
    args = ap.parse_args()
    fleet = {i: MockCopter(i, pad_e=(i - 1) * 6.0) for i in range(1, 6)}
    drop = tuple(map(float, args.drop_link.split("@"))) if args.drop_link else None
    t0 = last = time.monotonic()
    while time.monotonic() - t0 < args.duration:
        now = time.monotonic()
        dt, last = now - last, now
        el = now - t0
        if args.master_gps_fail_at and el >= args.master_gps_fail_at and fleet[1].gps_ok:
            fleet[1].gps_ok = False
            print(f"[mock] t={el:.0f}s master GPS FAILED")
        if drop and el >= drop[1] and fleet[int(drop[0])].link_up:
            fleet[int(drop[0])].link_up = False
            print(f"[mock] t={el:.0f}s slave {int(drop[0])} link DOWN")
        for c in fleet.values():
            c.rx(now)
            c.step(now, dt)
            c.tx(now)
        time.sleep(0.02)


if __name__ == "__main__":
    main()
