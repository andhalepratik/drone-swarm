# 5-Drone ArduPilot Swarm: Setup and Operating Guide

One master quad flown by a pilot, four slave quads flown in formation by `swarm_gcs.py` on the GCS laptop. GPS and barometer only.

**Airframe (all five identical):** 7-inch quad, 6S, MicoAir743 flight-controller stack, GEPRC GEP-M1025 GPS, RadioMaster RP3 ELRS receiver, WiFi telemetry module. MAVLink runs over WiFi (one UDP port per drone). RC runs over ELRS: the pilot's own TX for the master, and one shared "safety TX" broadcasting to all four slaves.

> **Status:** the controller is bench-tested against the kinematic mock only. Your first real validation is ArduPilot SITL (Step 2). Do not skip any step in order.

---

## 1. What's in the package

| File | What it is | Where it runs |
|---|---|---|
| `swarm_gcs.py` | The formation controller: reads the master, commands the slaves, collision monitoring, failsafes, operator CLI | GCS laptop |
| `start_links.sh` | Starts five MAVProxy routers (one per drone) for the real WiFi links | GCS laptop |
| `sitl_swarm.sh` | Starts five ArduCopter SITL simulators wired exactly like the real system | GCS laptop |
| `mock_fleet.py` | Fast, crude 5-drone fake for testing the script's logic in seconds | GCS laptop |
| `run_mock_tests.py` | Automated mock tests: nominal, master GPS loss, slave link loss. Exit code 0 = pass. | GCS laptop, GitHub Actions |
| `.github/workflows/mock-tests.yml` | Runs the mock tests on GitHub on every push | GitHub |
| `.gitignore` | Keeps flight logs (they contain GPS locations) and secrets out of the repo | Git |
| `params/hardware_micoair743.parm` | Base hardware setup: frame, motors, GPS, RC, compass, battery. Same on all five. | Load onto every FC first, during the build |
| `params/master.parm` | ArduPilot parameters for the master (sysid 1) | Load onto master FC |
| `params/slave2.parm` … `slave5.parm` | Parameters for each slave (sysid 2–5, unique RTL altitudes) | Load onto each slave FC |

### Port map (memorise this)

```
drone N WiFi module --UDP--> GCS_IP:1456N --> MAVProxy #N --+--> 127.0.0.1:1457N  (swarm_gcs.py)
                                                            +--> 127.0.0.1:14550  (QGroundControl)
```

| Drone | Sysid | Module sends to | Script listens on | RTL altitude |
|---|---|---|---|---|
| Master | 1 | GCS_IP:14561 | 14571 | 15 m (min) |
| Slave 2 | 2 | GCS_IP:14562 | 14572 | 50 m |
| Slave 3 | 3 | GCS_IP:14563 | 14573 | 55 m |
| Slave 4 | 4 | GCS_IP:14564 | 14574 | 60 m |
| Slave 5 | 5 | GCS_IP:14565 | 14575 | 65 m |

---

## 2. What you need

**Laptop:** Linux (Ubuntu 22.04/24.04 recommended). The `.sh` launchers are bash; on Windows use WSL2 or run the MAVProxy commands from section 6 by hand. Python 3.9 or newer.

**Software:**

```bash
pip install pymavlink MAVProxy
# QGroundControl: download the AppImage from qgroundcontrol.com
```

**Per drone:**

| Part | Model | Connects to |
|---|---|---|
| Flight controller + 4-in-1 ESC | MicoAir743 stack (v1 or v2), ArduCopter 4.5/4.6 | — |
| GPS | GEPRC GEP-M1025**Q** recommended (with compass) | UART3 + I2C (SDA/SCL) |
| RC receiver | RadioMaster RP3, ELRS 2.4 GHz | UART6 (RX6 and TX6) |
| Telemetry | WiFi module (ESP32/ESP8266 MAVLink bridge) | UART1 |
| Battery | 6S LiPo | Stack power input |

**Ground:** a dedicated outdoor WiFi access point on a mast (not a phone hotspot), an Ethernet cable from the AP to the laptop, one ELRS transmitter for the pilot, and a second ELRS transmitter as the slave safety TX.

**People:** a pilot (master TX), a GCS operator (laptop), and a safety operator (safety TX, eyes on the sky).

Make the scripts executable once:

```bash
chmod +x *.sh *.py
```

---

## 3. Step 1: Bench test with the mock (no hardware, 10 minutes)

This proves the software runs on your laptop and lets you learn the CLI.

**Automated:**

```bash
python3 run_mock_tests.py nominal   # joins V, switches to diamond, then line
python3 run_mock_tests.py gps       # master GPS fails -> HOLD, then staggered RTL at +15 s
python3 run_mock_tests.py link      # slave 3 link dies -> HOLD, heartbeats withheld at +8 s
python3 run_mock_tests.py all       # all three (~4 minutes)
```

Each test prints PASS or FAIL with a checklist, and exits with code 1 on failure. Details go to `ctrl_<test>.log` and `mock_<test>.out`.

**Interactive:**

```bash
# terminal A
python3 mock_fleet.py
# terminal B
python3 swarm_gcs.py
```

In the mock, the master flies a 60 m square at 4 m/s and the slaves start already airborne in GUIDED. At the `swarm_gcs.py` prompt, try `status`, `engage`, `formation diamond`, `spacing 12`, `hold`, `engage`, `rtl`. To practise failures, restart the mock with `--master-gps-fail-at 60` or `--drop-link 3@50`.

---

## 4. Step 2: ArduPilot SITL (first real validation)

SITL runs the real ArduCopter firmware with your real `.parm` files.

**One-time build:**

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot && Tools/environment_install/install-prereqs-ubuntu.sh -y && . ~/.profile
./waf configure --board sitl && ./waf copter
```

**Run:**

```bash
# terminal A
ARDUPILOT=~/ardupilot ./sitl_swarm.sh
# terminal B (wait ~30 s for all five to get GPS lock)
python3 swarm_gcs.py
# QGroundControl: connects automatically on UDP 14550 and shows all five vehicles
```

**Fly it:**

1. In QGC, select vehicle 1, set LOITER or GUIDED, arm, and take off to 15 m.
2. In the script: `status` (all five should show link OK and nav ok), then `takeoff 2`, then `engage 2`. Watch slave 2 climb, transit and settle.
3. Add the rest one at a time: `takeoff 3`, `engage 3`, and so on.
4. In QGC, give the master a short mission and switch it to AUTO. Watch the formation follow and rotate.
5. Finish with `rtl`, then RTL the master.

**Rehearse every failure before any real flight:**

| Test | How to trigger in SITL | Expected result |
|---|---|---|
| Master GPS loss | MAVProxy console of vehicle 1: `param set SIM_GPS1_ENABLE 0` (on older builds `SIM_GPS_DISABLE 1`) | HOLD at once; staggered slave RTL after 15 s |
| Master goes home | Set master to RTL in QGC | HOLD; slaves climb 5 m |
| Script dies | Type `quit!` in the script | All slaves RTL within ~5 s, each at its own altitude |
| Collision check | `spacing 8`, then fly the master in sharp turns | Watch `min 3D sep` in the status line; no conflicts expected |
| Safety switch | QGC: set one slave to BRAKE, then LAND | Script detaches it and treats it as an obstacle; `engage` to rejoin |

Note: in SITL, flight-mode changes come from QGC and the script, not RC switches. The ELRS parameters are harmless there.

---

## 5. Step 3: Build and configure each drone

Do this for each quad individually, before loading any swarm-role parameters.

### Identify your board version and flash ArduCopter

The MicoAir743 comes in two versions whose UART numbering differs. Check the label on the board:

| | MicoAir743 (v1) | MicoAir743v2 |
|---|---|---|
| Firmware target | `MicoAir743` | `MicoAir743v2` |
| Onboard compass | IST8310 | QMC5883L |
| RC input (UART6) | SERIAL5 | SERIAL6 |
| ESC telemetry (UART7) | SERIAL6 | SERIAL7 |
| Extra | — | Bluetooth module on SERIAL8 |

First flash: hold the boot button, plug in USB, and load the `..._with_bl.hex` for your board with a DFU tool (STM32CubeProgrammer, or Mission Planner's bootloader option). After that, update from QGC or Mission Planner as usual. The same firmware version must go on all five quads.

### Wiring

| Device | Pads | Notes |
|---|---|---|
| WiFi module | UART1: TX1 → module RX, RX1 → module TX, 5 V from a BEC | Not the FC telemetry 5 V pad if the module draws spikes |
| GEP-M1025 GPS | UART3: TX3 → GPS RX, RX3 → GPS TX, 5 V, GND; SDA/SCL for the compass | Mast-mounted, arrow forward, away from ESC and battery leads |
| RP3 receiver | UART6: RX6 ← RP3 TX, TX6 → RP3 RX, 5 V, GND | Both wires: CRSF is two-way. The SBUS pad is not used. |
| HD VTX (if any) | The board's DisplayPort connector (UART2) | Leave UART2 as DisplayPort |

**Antenna spacing** matters with 2.4 GHz everywhere on this airframe (see section 7). Keep the RP3 antennas, the WiFi antenna and the GPS as far apart as the frame allows: RP3 at the rear, WiFi module on a front arm, GPS on a mast in the middle. Aim for at least 10 cm between any two.

### The compass question (plain M1025 vs M1025Q)

The plain GEP-M1025 has **no compass**; the Q variants have a QMC5883L. On a 7-inch quad the board's internal compass sits right next to the ESC current, so relying on it gives poor heading. For a slave, bad heading shows up as circling ("toilet-bowling") in its slot, which eats into the separation budget.

- **M1025Q (recommended):** after calibration, keep only the external compass. Untick the internal one on the GCS compass page, then run COMPASSMOT.
- **Plain M1025:** either buy the Q version, or at minimum run COMPASSMOT on the internal compass and reject the airframe if interference is above ~30%. Don't fly the swarm on a compass you haven't validated.

### Configure the hardware

1. Load `params/hardware_micoair743.parm` (all five quads), then reboot.
2. **Motor test with props off.** The file sets `FRAME_TYPE,12` (Betaflight motor order, usual for FPV stacks). If a motor spins in the wrong position, use `FRAME_TYPE,1` or remap; if one spins the wrong direction, reverse it in the ESC configurator.
3. Set the current scale (`BATT_AMP_PERVLT`) from the MicoAir ESC manual. Check voltage with a multimeter and current with a watt meter.
4. Standard calibration: accelerometer, compass (see above), radio.
5. Tune: hover, set the harmonic notch from the log, then a full AUTOTUNE. Tracking error is part of the separation budget, so a badly tuned slave is a collision risk.

### Load the swarm-role parameters

- **QGroundControl:** Vehicle Setup → Parameters → Tools → Load from file.
- **Mission Planner:** Config → Full Parameter List → Load from file → Write Params.

Load `master.parm` onto the master and `slaveN.parm` onto slave N. **Label each airframe with its number.** A slave with the wrong file gets the wrong RTL altitude, and two slaves could then share one.

After loading, **reboot**. The sysid changes, so reconnect.

Load order summary: `hardware_micoair743.parm` → calibrate and tune → `master.parm` or `slaveN.parm`. The role file never overwrites the calibration.

### Check these values against your hardware

| Parameter | Why you must check it |
|---|---|
| Names that differ by version | e.g. `GPS_RATE_MS` became `GPS1_RATE_MS` in 4.6; `SYSID_MYGCS` may appear as `MAV_GCS_SYSID` in newer builds. Read the loader's "unknown parameter" list and set those by hand. |
| `SERIAL1_BAUD` | Must equal the WiFi module's UART baud (115 = 115200, 921 = 921600) |
| `SERIAL1_*` / `SR1_*` | Correct for the WiFi module on UART1. Rename only if you wire it elsewhere (e.g. UART4 = SERIAL4). |
| RC input | Nothing to set: UART6 is already the RC port (SERIAL5 on v1, SERIAL6 on v2). Never set `SERIAL2_PROTOCOL` to 23; UART2 is the HD VTX DisplayPort. |
| `GPS_RATE_MS,200` | 5 Hz. The M10 chip can't sustain 10 Hz with all its constellations. |
| `BATT_LOW_VOLT`, `BATT_CRT_VOLT` | Set for 6S (21.0 / 19.8 V). Change them for your pack. |
| `FENCE_RADIUS` | Placeholders (80 m master, 110 m slaves). Set these inside your measured WiFi range after the range test. |

### Switch layout (changed in this version for ELRS)

ELRS uses channel 5 (AUX1) as its own arm flag, so channel 5 is now the arm switch.

| Channel | Master (pilot TX) | Slaves (safety TX) |
|---|---|---|
| 5 | Arm/disarm switch (`RC5_OPTION,153`) | Unused by ArduPilot. Keep it HIGH in flight so ELRS settings lock. |
| 6 | 3-pos mode switch: LOITER / ALT_HOLD / AUTO | 3-pos: **GUIDED / BRAKE / LAND** |
| 7 | RTL switch | **Motor emergency stop** (use a guarded switch) |
| 8 | Motor emergency stop | — |

---

## 6. Step 4: WiFi network

**Access point:**

- Fixed 2.4 GHz channel (1, 6 or 11), 20 MHz width.
- Disable band steering, roaming and power saving.
- WPA2 with a private SSID; client isolation **off**.
- Mount it high, near the flying area.

**Laptop:**

- Connect to the AP by Ethernet, not WiFi.
- Give it a static IP (for example 192.168.1.10) or a DHCP reservation. Every module needs this address.
- Allow UDP 14561–14565 through the firewall (`sudo ufw allow 14561:14565/udp`, or disable ufw on the field laptop).

**Each WiFi module:**

- Join the AP as a client (station mode), with power saving off.
- Send **unicast** UDP to the laptop's IP, on port **1456N** where N is the drone's sysid.
- Set the UART baud equal to `SERIAL1_BAUD`.
- The exact menu names depend on the firmware (ESP32 DroneBridge vs ESP8266 mavesp8266). Tell me which one you have and I'll give the exact settings.

**Start the routers:**

```bash
./start_links.sh        # Ctrl-C stops all five; logs in ~/swarm_logs/<date>/dN/
```

Without bash, run this once per drone, with N = 1..5:

```bash
mavproxy.py --master=udpin:0.0.0.0:1456N --out udp:127.0.0.1:1457N --out udp:127.0.0.1:14550 --streamrate=-1 --source-system 254 --daemon
```

**Checks before the first field day:**

1. **Ping** each module from the laptop for a minute. You want under 20 ms typical and no spikes over 100 ms.
2. **Ports:** QGC shows 5 vehicles with sysids 1–5, and the script's `status` shows all OK.
3. **Stale module:** unplug one module. It must show LOST within 3 s.
4. **Range:** walk one drone (powered, props off) out at flight height. Note where packets start dropping, and set `FENCE_RADIUS` well inside that.
5. **GPS desense:** compare satellite count and HDOP with the WiFi module transmitting vs unplugged. They should be the same.

---

## 7. Step 5: ELRS setup

**Firmware first:** flash every RP3 and both transmitters with the same ELRS major version (e.g. all 3.x). Receivers on a different major version won't connect. The RP3 has a WiFi update mode; the ExpressLRS Configurator can also set the bind phrase directly.

**Pilot TX to master (normal ELRS link):**

- Give it its own bind phrase, used by no other radio.
- Link Mode Normal, telemetry ratio Std.
- Switches as in the table in section 5.
- Model Match optional.

**Safety TX to all four slaves (broadcast):**

1. Choose a second, different bind phrase. Flash it into the safety TX and **all four slave receivers**.
2. Telemetry Ratio: **Off**. No receiver transmits, so the four don't collide.
3. Use a **fixed** TX power. Dynamic power needs telemetry, and there is none here.
4. Handset model: ch5 switch HIGH in flight, ch6 3-pos (low = GUIDED), ch7 guarded switch for motor stop, throttle stick low.
5. **Range-test with all four slaves powered at once.** One user found that mixing a telemetry-on receiver with telemetry-off ones sharply cut the others' range, so every slave receiver must be telemetry-off.

**2.4 GHz coexistence.** The RP3 (ELRS), the WiFi telemetry and the AP all use 2.4 GHz. ELRS hops across the whole band, so no channel plan avoids it. This usually works, but it must be measured, not assumed:

1. With all five drones powered and both ELRS transmitters on, run the ping test from section 6. The WiFi latency and loss must still meet the targets.
2. Range-test ELRS with the WiFi modules transmitting.
3. If either degrades: increase antenna spacing on the airframe first. If that isn't enough, the fixes are 900 MHz ELRS receivers (keeps WiFi on 2.4 GHz) or 5 GHz-capable WiFi modules.

**Ground test of the safety TX** (props off, drones powered and disarmed): flip ch6 through its three positions. QGC must show all four slaves switch GUIDED → BRAKE → LAND together. Then return it to GUIDED.

**What happens if the safety TX is lost in flight:** by default all four slaves RTL (safe, since their RTL altitudes are unique). To let the script keep flying them instead, add `FS_OPTIONS,4` to the slave files, accepting that you have no manual override until the link returns.

---

## 8. Step 6: Props-off rehearsal at the field

```bash
./start_links.sh
python3 swarm_gcs.py --dry-run
```

`--dry-run` does everything (monitoring, state machine, failsafe logic) except send position targets. Carry the master around by hand at the field to confirm positions, heading and link quality look right. Run `status` often.

---

## 9. Step 7: First real flights, one slave at a time

1. Place each slave on its own pad, at least 5 m apart. **Each drone's arming spot is its RTL home.** The master's pad goes in front.
2. Power the AP, then the laptop. Run `start_links.sh`, then QGC.
3. Power the drones. Wait for GPS lock (HDOP ≤ 1.2) on all five.
4. Run `python3 swarm_gcs.py`, then `status`. Everything should be OK.
5. Safety TX: ch5 high, ch6 GUIDED, ch7 motor stop OFF.
6. The pilot arms the master and hovers it at 10–15 m, in LOITER.
7. GCS: `takeoff 2`. Slave 2 arms and climbs vertically to 8 m. Then `engage 2`: it goes to its transit layer, crosses, and settles into its slot.
8. Fly gently for a few minutes. Watch `e` (slot error) and `min 3D sep` in the status line.
9. `rtl 2`, then land the master.

Repeat, adding **one more slave per flight**. Only fly the full swarm once each slave has flown several clean flights.

### Normal full-swarm flight

```
takeoff 2  -> engage 2
takeoff 3  -> engage 3        (wait for each to settle before the next)
takeoff 4  -> engage 4
takeoff 5  -> engage 5
... pilot flies the master in LOITER, or switches to AUTO for a QGC mission ...
formation diamond / spacing 12 / heading course   (optional, in flight)
rtl                            (slaves go home one by one, each on its own layer)
... pilot lands the master last
```

---

## 10. Operator command reference (`swarm_gcs.py`)

| Command | Effect |
|---|---|
| `status` | Detailed state of every drone (mode, armed, alt, GPS fix, sats, HDOP, EKF flags, link) |
| `takeoff [all\|ids]` | GUIDED, arm and take off, one slave at a time, to unique altitudes (8/11/14/17 m) |
| `engage [all\|ids]` | Join slaves to the formation. Also the only way to resume after any HOLD. |
| `hold` | Freeze all slaves at their stopping points |
| `idle` | Stop sending targets (slaves keep their last target; heartbeats continue) |
| `formation v\|line\|trail\|diamond` | Change shape. Slaves re-slot via their transit layers. |
| `spacing <m>` | Change spacing. Refused if slots would be under 8 m apart. |
| `heading course\|heading\|fixed` | What the formation faces: master's track (default), master's nose, or fixed |
| `rtl [all\|ids]` | Staggered RTL, each slave at its unique altitude |
| `land [all\|ids]` | Staggered LAND in place |
| `quit` / `quit!` | Exit. `quit` refuses while slaves are airborne; `quit!` forces it, and they then RTL via GCS failsafe. |

The first Ctrl-C holds the swarm instead of exiting.

**Command-line options:**

```
python3 swarm_gcs.py [--formation v|line|trail|diamond] [--spacing 10]
                     [--heading-mode course|heading|fixed] [--dry-run]
                     [--log file.log] [--links '1=udpin:127.0.0.1:14571,2=...']
```

---

## 11. Who does what in an emergency

| Situation | Automatic response | Human action |
|---|---|---|
| Master GPS/EKF bad | Swarm HOLD; staggered slave RTL after 15 s | Pilot switches master to ALT_HOLD (mode switch middle) and brings it down gently |
| Master telemetry lost | Same as above | Pilot keeps flying the master by RC; lands |
| Master RTL/LAND | HOLD; slaves climb 5 m | GCS: `rtl` the slaves once the master's path is clear |
| One slave's link lost | Swarm HOLD; that slave RTLs by its own failsafe after 8 + 5 s | GCS: watch it home; `rtl` or `engage` the others |
| Laptop/script/AP dies | All slaves RTL within ~5 s on unique layers | Pilot lands the master away from the pads |
| Predicted conflict | HOLD; the higher drone climbs 4 m, the lower one brakes | GCS: find the cause, then `engage` |
| Anything looks wrong | — | Safety operator: **ch6 → BRAKE**. Then LAND, or back to GUIDED and `engage`. |
| Drone out of control, heading for people | — | Motor stop (safety TX ch7 for slaves, pilot ch8 for master). Last resort. |

---

## 12. Settings that must match between the script and the .parm files

If you change one side, change the other and re-run the script. `sanity_check_config()` checks the script's own geometry at start-up.

| In `swarm_gcs.py` | Must match |
|---|---|
| `SCRIPT_SYSID = 250` | `SYSID_MYGCS,250` on every slave |
| `DEFAULT_LINKS` ports 14571–14575 | `start_links.sh` and `sitl_swarm.sh` |
| Master altitude window 8–25 m, transit layers up to +19 m, slave ceiling 45 m | Slave `FENCE_ALT_MAX,75` and the RTL altitudes 50–65 m must stay above everything |
| `LINK_LATENCY_S = 0.15` | Your measured link delay (ping/2 plus processing) |
| Heartbeat/position timeouts (3 s / 1.5 s) | Slave `FS_GCS_TIMEOUT,5` must be longer |

---

## 13. Troubleshooting

| Symptom | Likely cause |
|---|---|
| Drone shows LOST in `status` | Module sending to the wrong IP/port, firewall, or baud mismatch with `SERIAL1_BAUD` |
| QGC shows a sysid twice, or one missing | Two drones loaded with the same `.parm`, or two modules on the same port |
| `engage refused: master at X m` | Master must be between 8 and 25 m |
| `engage refused: master navigation bad` | Wait for a better GPS fix; check `status` for sats/HDOP/EKF |
| Slave `did not arm` | Read its pre-arm messages in QGC (common causes: GPS not ready, safety TX not linked, battery) |
| Slaves wobble or lag in turns | Measure the real latency and set `LINK_LATENCY_S`; slow the master; check AUTOTUNE |
| Slaves all go to BRAKE by themselves | Safety TX ch6 not in the GUIDED position, or the safety TX is off (RC failsafe) |
| Many "glitch" warnings | GPS desense from the WiFi module, or a poor GPS mount |
| No RC input on the FC | RP3 wired to the SBUS pad or only one wire connected; it needs RX6 and TX6. Or ELRS major versions don't match. |
| HD OSD stopped working | `SERIAL2_PROTOCOL` was changed; set it back to DisplayPort (42) |
| GPS not detected | TX/RX swapped on UART3, or `SERIAL3_PROTOCOL` not 5 |
| Slave circles in its slot | Compass interference: use the external compass only, redo COMPASSMOT |
| Drone flips on first takeoff | Motor order: `FRAME_TYPE` wrong (12 vs 1). Always run the motor test. |

---

## 14. Logs to keep after every flight

- `swarm_YYYYMMDD_HHMMSS.log`: the script's full debug log (in the folder you ran it from)
- `~/swarm_logs/<date>/dN/`: MAVProxy telemetry logs (`.tlog`), one folder per drone
- Each flight controller's dataflash `.bin` log (download with QGC or Mission Planner)

---

## 15. Repository and CI

The `.gitignore` keeps flight logs out of the repo. They contain the GPS coordinates of where you fly, so don't force-add them.

On every push, GitHub Actions runs the three mock tests in parallel (`.github/workflows/mock-tests.yml`). A red X on a commit means a change broke formation joining, the master GPS-loss failsafe, or the slave link-loss failsafe. The logs from a failed run are attached to it. A green tick means only that the logic still passes the mock: it is not a substitute for SITL and field tests.

> **Safety notice:** experimental software. Flying multiple autonomous aircraft is dangerous and regulated. You are responsible for complying with local law and for operating safely. Tested so far: mock only.
