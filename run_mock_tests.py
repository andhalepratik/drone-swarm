#!/usr/bin/env python3
"""
run_mock_tests.py - automated bench tests: swarm_gcs.py against mock_fleet.py.

Runs the real controller logic (imported from swarm_gcs.py) with a scripted
list of operator commands, against the kinematic mock, and reports the minimum
3-D separation seen while engaged or held. No hardware, no SITL needed.

  python3 run_mock_tests.py nominal   # join V, switch to diamond, then line   (~90 s)
  python3 run_mock_tests.py gps       # master GPS fails at t=40 s            (~75 s)
  python3 run_mock_tests.py link      # slave 3 telemetry dies at t=35 s       (~70 s)

Run from this folder. Output: ctrl_<test>.log (controller) and mock_<test>.out.
"""
import subprocess, sys, time, threading, logging, math
WHICH=sys.argv[1] if len(sys.argv)>1 else 'nominal'; sys.argv=['x']
import swarm_gcs as S
def run(mock_args, schedule, duration, tag):
    mock = subprocess.Popen([sys.executable, 'mock_fleet.py', '--duration', str(duration+5)] + mock_args,
                            stdout=open(f'mock_{tag}.out','w'), stderr=subprocess.STDOUT)
    log = logging.getLogger(tag); log.setLevel(logging.DEBUG); log.handlers=[]
    fh = logging.FileHandler(f'ctrl_{tag}.log','w'); fh.setLevel(logging.INFO)
    fh.setFormatter(logging.Formatter('%(relativeCreated)6.0f %(levelname).1s %(message)s')); log.addHandler(fh)
    S.sanity_check_config()
    links={sid:S.VehicleLink(sid,url,log) for sid,url in S.DEFAULT_LINKS.items()}
    for l in links.values(): l.start()
    c=S.SwarmController(links,'v',10.0,'course',False,log)
    minsep=[1e9]
    def sched():
        t0=time.monotonic()
        for t,cmd in schedule:
            time.sleep(max(0,t-(time.monotonic()-t0))); c.cmdq.put(cmd); log.info('>>> '+cmd)
        time.sleep(max(0,duration-(time.monotonic()-t0))); c.quit_evt.set()
    def mon():
        while not c.quit_evt.is_set():
            if not math.isinf(c.min_sep) and c.state!='IDLE': minsep[0]=min(minsep[0],c.min_sep)
            time.sleep(0.2)
    threading.Thread(target=sched,daemon=True).start(); threading.Thread(target=mon,daemon=True).start()
    c.run()
    for l in links.values(): l.stop()
    mock.wait()
    print(f'== {tag}: min separation while engaged/hold = {minsep[0]:.1f} m')
if __name__=='__main__':
    which=WHICH
    if which=='nominal':
        run([], [(3,'status'),(4,'engage')] + [(30,'formation diamond'),(60,'formation line')], 90, 'nominal')
    elif which=='gps':
        run(['--master-gps-fail-at','40'], [(4,'engage')], 75, 'gps')
    elif which=='link':
        run(['--drop-link','3@35'], [(4,'engage')], 70, 'link')
