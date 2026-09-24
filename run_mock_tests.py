#!/usr/bin/env python3
"""
run_mock_tests.py - automated bench tests: swarm_gcs.py against mock_fleet.py.

Runs the real controller logic (imported from swarm_gcs.py) with a scripted
list of operator commands, against the kinematic mock, then checks the logs
for the expected behaviour. Exit code 0 = all selected tests passed, 1 = a
test failed (so it can gate commits in CI). No hardware or SITL needed.

  python3 run_mock_tests.py nominal   # join V, switch to diamond, then line  (~90 s)
  python3 run_mock_tests.py gps       # master GPS fails at t=40 s           (~75 s)
  python3 run_mock_tests.py link      # slave 3 telemetry dies at t=35 s      (~70 s)
  python3 run_mock_tests.py all       # all three, one after another          (~4 min)

Run from this folder. Output: ctrl_<test>.log (controller), mock_<test>.out (mock).
"""
import logging
import math
import subprocess
import sys
import threading
import time

WHICH = sys.argv[1] if len(sys.argv) > 1 else "nominal"
sys.argv = [sys.argv[0]]          # swarm_gcs must not see our arguments
import swarm_gcs as S              # noqa: E402

MIN_SEP_REQUIRED_M = 3.0           # 3-D, includes vertical layer spacing


def run(mock_args, schedule, duration, tag):
    """Start the mock, drive the controller with 'schedule', return (min_sep, ctrl_log, mock_out)."""
    mock = subprocess.Popen(
        [sys.executable, "mock_fleet.py", "--duration", str(duration + 5)] + mock_args,
        stdout=open(f"mock_{tag}.out", "w"), stderr=subprocess.STDOUT)
    log = logging.getLogger(tag)
    log.setLevel(logging.DEBUG)
    log.handlers = []
    fh = logging.FileHandler(f"ctrl_{tag}.log", "w")
    fh.setLevel(logging.INFO)
    fh.setFormatter(logging.Formatter("%(relativeCreated)6.0f %(levelname).1s %(message)s"))
    log.addHandler(fh)

    S.sanity_check_config()
    links = {sid: S.VehicleLink(sid, url, log) for sid, url in S.DEFAULT_LINKS.items()}
    for link in links.values():
        link.start()
    ctrl = S.SwarmController(links, "v", 10.0, "course", False, log)
    min_sep = [math.inf]

    def scheduler():
        t0 = time.monotonic()
        for t, cmd in schedule:
            time.sleep(max(0.0, t - (time.monotonic() - t0)))
            ctrl.cmdq.put(cmd)
            log.info(">>> " + cmd)
        time.sleep(max(0.0, duration - (time.monotonic() - t0)))
        ctrl.quit_evt.set()

    def monitor():
        while not ctrl.quit_evt.is_set():
            if not math.isinf(ctrl.min_sep) and ctrl.state != "IDLE":
                min_sep[0] = min(min_sep[0], ctrl.min_sep)
            time.sleep(0.2)

    threading.Thread(target=scheduler, daemon=True).start()
    threading.Thread(target=monitor, daemon=True).start()
    ctrl.run()
    for link in links.values():
        link.stop()
    mock.wait()
    fh.close()
    return min_sep[0], open(f"ctrl_{tag}.log").read(), open(f"mock_{tag}.out").read()


def check(tag, conditions):
    """conditions: list of (description, bool). Prints a report, returns True if all pass."""
    ok = all(c for _, c in conditions)
    print(f"== {tag}: {'PASS' if ok else 'FAIL'}")
    for desc, c in conditions:
        print(f"   [{'ok' if c else 'FAIL'}] {desc}")
    return ok


def test_nominal():
    sep, ctrl, _ = run([], [(3, "status"), (4, "engage"), (30, "formation diamond"),
                            (60, "formation line")], 90, "nominal")
    return check("nominal", [
        ("all four slaves reached their slots",
         all(f"{s}:IN/GUIDED" in ctrl for s in S.SLAVE_IDS)),
        ("no separation conflict", "SEPARATION CONFLICT" not in ctrl),
        ("no unexpected HOLD", "SWARM HOLD" not in ctrl),
        (f"min 3-D separation >= {MIN_SEP_REQUIRED_M} m (was {sep:.1f} m)",
         sep >= MIN_SEP_REQUIRED_M),
    ])


def test_gps():
    _, ctrl, _ = run(["--master-gps-fail-at", "40"], [(4, "engage")], 75, "gps")
    return check("gps", [
        ("swarm held on master GPS loss", "SWARM HOLD: master navigation bad" in ctrl),
        ("staggered RTL ordered after the delay", "RTL for slaves" in ctrl),
        ("every slave was sent to RTL",
         all(f"slave {s} -> RTL" in ctrl for s in S.SLAVE_IDS)),
    ])


def test_link():
    _, ctrl, mock = run(["--drop-link", "3@35"], [(4, "engage")], 70, "link")
    return check("link", [
        ("swarm held on slave 3 telemetry loss", "SWARM HOLD: slave 3 telemetry lost" in ctrl),
        ("heartbeats withheld from slave 3", "withholding heartbeats" in ctrl),
        ("slave 3 ran its own GCS failsafe (RTL)", "[mock 3] GCS failsafe -> RTL" in mock),
    ])


TESTS = {"nominal": test_nominal, "gps": test_gps, "link": test_link}

if __name__ == "__main__":
    names = list(TESTS) if WHICH == "all" else [WHICH]
    if any(n not in TESTS for n in names):
        sys.exit(f"unknown test '{WHICH}': choose {list(TESTS)} or all")
    results = [TESTS[n]() for n in names]
    sys.exit(0 if all(results) else 1)
