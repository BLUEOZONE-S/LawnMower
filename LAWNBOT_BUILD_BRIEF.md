# LawnBot — Raspberry Pi Autonomous Mower Build Brief

A Claude Code implementation spec for porting the validated waypoint-coverage simulator to real hardware: **Raspberry Pi Zero 2 W + Quectel LC29H(DA) RTK GPS + Waveshare Motor Driver HAT (15364) + PiSugar UPS**.

> **Prototype scope:** this is a non-cutting research rover that *behaves* like a robotic mower. There is **no blade and no cutting hardware** — "mowing" means driving the coverage path and logging which cells were covered. It is not a hazardous machine; the safeguards below are about not losing or stranding the robot and keeping test runs clean, not bodily safety.

> The control logic, coverage planner, PID, and GPS/IMU fusion model in this spec are a direct port of an already-validated browser simulator. Treat the sim's behavior as the functional reference: lawn-only A* routing between cut rows, a PID heading controller running on a fused pose estimate, 1 Hz GPS correction, and a crosscut second pass.

---

## 0. Operational safeguards (prototype rover, no blade)

No blade, no cutting, low-speed — this is not a dangerous machine. These safeguards exist so the robot doesn't drive off, run its battery flat, or wander out of the test area, and so runs are repeatable. They are software-level, not hardware kill paths.

- **Stop command**: a single `stop()` that zeroes both motors, callable from the loop, a key press, or a remote command. Wire it to the PiSugar tap-button if convenient.
- **Software watchdog**: the control loop pets a watchdog each cycle; if the loop stalls, motors auto-stop (also enforced by the motor command timeout in `motor_hat.py`).
- **Geofence**: if the estimated position leaves the boundary polygon + margin, stop and flag — keeps the rover on the test plot.
- **RTK-degradation handling**: if fix quality drops below the configured level for longer than N seconds, stop (or slow) — a cm-planned path executed with 1 m accuracy just wanders; this keeps behavior sensible, not safe-vs-injury.
- **Low battery**: park/stop on low charge and let pisugar-server handle safe shutdown so you don't corrupt the SD card.
- **Stuck handling**: never grind in place — if the position stops advancing while motion is commanded, recover briefly then give up and stop (see §13).
- **Manual override**: the operator can take over at any time from the GUI to drive the rover clear, then hand control back (see §12).
- **Bring-up rule**: do first motor tests with the **wheels off the ground** (chassis on a stand) so a runaway duty value can't send it across the room.

---

## 1. Goal

Autonomously cover a defined lawn polygon containing keep-out zones (trees, beds, etc.) with a boustrophedon path plus a perpendicular crosscut pass, navigating only on lawn, tracking waypoints with a PID controller fed by a fused GPS+IMU+odometry pose estimate. "Coverage" is driving over the cells and logging them — there is no cutting. The "deck width" throughout this doc is the **simulated coverage swath** used for stripe spacing, not a physical blade.

---

## 2. Hardware

### 2.1 Bill of materials
| Item | Part | Role |
|---|---|---|
| Compute | Raspberry Pi Zero 2 W | Control loop, planner, fusion |
| GNSS | Quectel LC29H(DA) | 1 Hz RTK position over UART |
| Chassis | **RC car — Ackermann steering** | 2 brushed drive motors (front+rear) + 1 front steering servo |
| Motor driver | Waveshare Motor Driver HAT **15364** | Drives the 2 brushed motors together as one throttle (PCA9685 + TB6612FNG) |
| Power/UPS | PiSugar (S/2/3-class) | Battery, charge mgmt, safe shutdown, RTC |
| IMU **(required, not yet specified)** | recommend **BNO085 / BNO055** | Absolute heading (yaw) |
| Drive encoder **(recommended)** | quadrature on the drive shaft / rear axle | Odometry (distance) between GPS fixes |
| Steering servo | the RC car's front servo | Sets steering angle δ (Ackermann) |
| Servo power | **5–6 V UBEC (~3 A)** | Dedicated servo rail; isolates stall spikes from the Pi |
| Corrections | NTRIP caster account or local base | RTCM3 for RTK-fixed |

> The LC29H(DA) has **no dead reckoning** and only a **1 Hz** fix. The IMU (heading) and a drive encoder (distance) carry the robot through the ~0.6 m it travels between fixes.
>
> **TB6612 current caution:** the HAT's TB6612FNG is ~**1.2 A continuous (≈3.2 A peak) per channel**. RC-car (540-class) motors can exceed this under load. Run low duty, watch for heat/cutout; put each motor on its own channel (A and B) to split current; if it still can't keep up, drive the motors from a higher-current driver and keep the HAT for lighter jobs.

### 2.2 Bus / pin allocation
| Bus | Device | Address / pins | Notes |
|---|---|---|---|
| I2C-1 | PCA9685 on Motor HAT | **0x40** (default; +0x70 ALLCALL) | GPIO2 (SDA), GPIO3 (SCL) |
| I2C-1 | PiSugar power IC | **0x75** + RTC **0x32** (PiSugar 2-class) **or** **0x57**+**0x68** (PiSugar 3) | Confirm with `i2cdetect -y 1`. Never write raw to power IC. |
| UART | LC29H(DA) | GPIO14 (TXD→GPS RX), GPIO15 (RXD←GPS TX), `/dev/serial0` | Disable serial console; default baud commonly 115200 (some boards 460800 — autodetect). |
| GPIO | IMU | I2C (BNO085) or its own pins | Separate addr from above |
| GPIO | Drive encoder | 2 pins (A/B), interrupt-capable | one drive-axis encoder |
| GPIO | **Steering servo** | **GPIO18**, 50 Hz PWM via **pigpio**; power from UBEC | not on the HAT — see note below |
| Button | Stop (optional) | PiSugar tap-button or a GPIO button | Software stop trigger |

No I2C address conflicts: 0x40/0x70 (HAT) vs 0x75/0x32 or 0x57/0x68 (PiSugar). The HAT uses only I2C + power; PiSugar feeds power through the GPIO underside and frees the UART for the GPS.

**Why the servo is on a GPIO, not the HAT:** the PCA9685 has a **single frequency prescaler shared by all 16 channels**. The drive motors want ~1 kHz PWM; a servo wants 50 Hz — you can't have both on one PCA9685, and the 15364 doesn't break out spare servo channels anyway. So the two motors stay on the HAT's PCA9685 at ~1 kHz and the **steering servo runs on a Pi GPIO via the pigpio daemon** (jitter-free hardware-timed 50 Hz). *Alternative:* a separate PCA9685 servo board re-addressed to 0x41 running at 50 Hz.

### 2.3 Power — three rails, one common ground
- **PiSugar → Pi** (5 V logic/compute).
- **RC-car battery → HAT VIN (6–12 V) → the two drive motors** (and the HAT's own regulator). Keep motor current off the Pi's 5 V rail.
- **UBEC (5–6 V) → steering servo**, fed from the RC-car battery. Servos draw spiky stall current; a dedicated rail keeps that off the Pi.
- **Tie all grounds together** (Pi, HAT, UBEC, servo, battery) — the GPIO/pigpio servo signal needs a shared ground reference.
- Add bulk capacitance on the motor supply; motors brown-out the Pi otherwise.

### 2.4 Waveshare HAT (15364) channel map — PCA9685 → TB6612
Reference Waveshare mapping (verify against their `PCA9685.py` / `MotorDriver` example):
- **Motor A**: PWM = channel **0**, AIN1 = channel **1**, AIN2 = channel **2**
- **Motor B**: PWM = channel **5**, BIN1 = channel **3**, BIN2 = channel **4**

Direction logic per motor: forward → IN1=full-on, IN2=full-off; reverse → swap; brake → both on; coast → both off. Speed = duty on the PWM channel. PWM frequency ≈ 1000 Hz (HAT range 40–1000 Hz). Drive **both motors with the same direction + duty** (one throttle). The steering servo is **not** on this PCA9685 — see §2.2.

---

## 3. System setup, deployment & runtime

### 3.1 OS, interfaces & dependencies (Pi OS Lite, 64-bit)

```bash
# Interfaces
sudo raspi-config nonint do_i2c 0          # enable I2C
sudo raspi-config nonint do_serial_hw 0    # enable UART hardware
sudo raspi-config nonint do_serial_cons 1  # disable serial login console
# /boot/firmware/config.txt: enable_uart=1 ; (dtoverlay=disable-bt to free PL011 if needed)

# Deps
sudo apt update && sudo apt install -y python3-pip python3-smbus i2c-tools git
pip3 install pyserial pynmea2 smbus2 adafruit-circuitpython-pca9685 \
             adafruit-blinka pyproj pygnssutils numpy pyyaml

# PiSugar power manager (battery API + safe shutdown)
wget https://cdn.pisugar.com/release/pisugar-power-manager.sh
bash pisugar-power-manager.sh -c release    # select the correct PiSugar model when prompted
# Battery API then available at /tmp/pisugar-server.sock and http://<pi>:8421

# Steering servo daemon (jitter-free hardware-timed PWM)
sudo apt install -y pigpio && sudo systemctl enable --now pigpiod

# Verify buses
i2cdetect -y 1     # expect 0x40 (HAT) and PiSugar addrs
```

### 3.2 Field networking (no router in the yard)
- The yard usually has no Wi-Fi. Run the Pi as its **own Wi-Fi access point** (NetworkManager hotspot or hostapd+dnsmasq) so your phone connects directly and loads the GUI at a fixed IP (e.g. `http://192.168.4.1:8080`).
- **RTK corrections need internet** — the real catch. Options: (a) **phone hotspot** — the Pi joins your phone's hotspot for NTRIP and you browse the GUI over that same hotspot; (b) a **local RTK base** (a second GNSS) broadcasting RTCM over a local radio/Wi-Fi link — no internet needed; (c) accept **RTK-float/DGPS** when offline. The planner's accuracy depends on which you pick.

### 3.3 App deployment & autostart
- Pure Python + static web assets. Use a **venv** (`python3 -m venv .venv`), `pip install -r requirements.txt`.
- Deploy by `git pull` on the Pi, or `rsync`/`scp` from your dev machine (a `deploy.sh` that rsyncs + restarts the service).
- Run under **systemd** (`systemd/lawnbot.service`): `Restart=on-failure`, `After=network-online.target pisugar-server.service pigpiod.service`. Inspect with `journalctl -u lawnbot -f`.

### 3.4 SD-card resilience
- The rover loses power abruptly. Mount logs on **tmpfs**, enable log rotation, and consider an **overlayfs read-only root** for production. PiSugar's low-battery safe-shutdown also protects the card.

### 3.5 Lightweight / performance tuning (Pi Zero 2 W, 512 MB)
- **Pi OS Lite, 64-bit, no desktop** — boot to console; frees RAM + CPU.
- Disable unused services: `bluetooth` (also frees the PL011 UART for the GPS — so no Bluetooth gamepad; use on-screen/USB), `avahi-daemon`, `triggerhappy`, `ModemManager`, `cups`.
- **Heavy work is one-time**: A\* coverage planning runs once per mission, then cached; the 20 Hz control loop is light arithmetic.
- **Rendering lives on the client**: the Pi sends compact JSON state over WebSocket; the phone's browser does all the canvas drawing — the Pi never rasterizes.
- Single `uvicorn` worker + `uvloop`; vectorize with `numpy`.
- Run the **control loop in its own thread at real-time priority** (`SCHED_FIFO` via `os.sched_setscheduler`), separate from the web server, so UI traffic can't jitter control timing.
- CPU governor `performance`; ensure a solid 5 V supply (brownout → throttle). Parse only the GPS sentences you need. Coverage grid ~0.15–0.2 m is the main memory knob.

---

## 4. Architecture

```
                 ┌─────────────┐   RTCM3 over UART
   NTRIP caster ─┤ ntrip.py    ├──────────────┐
                 └─────────────┘               ▼
 LC29H(DA) ──UART──► gnss/lc29h.py ──► (lat,lon,fixq,1Hz) ─┐
 BNO085   ──I2C───► sensors/imu.py ──► (yaw, fast)        ├─► estimator.py ──► pose_est (x,y,θ) @ ctrl rate
 Encoders ──GPIO──► sensors/odometry.py ──► (Δs, fast)    ┘        │
                                                                   ▼
 boundary.yaml ─► nav/geo.py (lat/lon→ENU) ─► nav/planner.py ─► waypoint list
                                                                   │
 pose_est + waypoints ─► nav/controller.py (PID + pure-pursuit) ─► (v, ω)
                          │
                          ▼
 nav/mission.py (reach logic, recover, pass mgmt) ─► drive/kinematics.py ─► (dutyL,dutyR)
                                                                   │
                                                                   ▼
                                                   drive/motor_hat.py (PCA9685+TB6612)
 safety/monitor.py  ◄── battery (pisugar), geofence, watchdog, RTK age, stop cmd  ──► stop drive motors
 safety/stuck.py    ◄── fused-position progress + odom/GPS mismatch  ──► recover → STUCK (stop)

 Operator (phone/laptop) ⇄ ui/server.py (WebSocket @10Hz state + commands)
        │  AUTO: mission control + live gains       │  MANUAL: teleop.py ─► kinematics ─► motors (deadman)
        │  TEACH: nav/teach.py records breadcrumbs ─► boundary.yaml
```

### 4.1 Module responsibilities
| Module | Responsibility | Ported from sim? |
|---|---|---|
| `config.py` | Load `config.yaml` (gains, geometry, thresholds, pins) | — |
| `gnss/lc29h.py` | Open UART, read NMEA + PQTM, parse GGA (lat/lon, **quality**, HDOP, sats), expose latest fix + age | new |
| `gnss/ntrip.py` | NTRIP client → stream RTCM3 to the LC29H UART (use `pygnssutils.GNSSNTRIPClient`) | new |
| `sensors/imu.py` | Absolute yaw (BNO085 fused) at fast rate | new (= sim "IMU heading") |
| `sensors/odometry.py` | Wheel-encoder distance/velocity, track-aware | new (= sim "odom") |
| `estimator.py` | Fuse: dead-reckon (IMU yaw + odom) fast; correct toward GPS on each 1 Hz fix (complementary filter; upgrade path: EKF) | **yes** (sim `est` + ALPHA blend) |
| `nav/geo.py` | lat/lon ↔ local ENU meters; reference origin = boundary centroid | new |
| `nav/geometry.py` | polygon ops, point-in-poly, bbox, interval subtract, **polygon keep-outs** + Douglas–Peucker simplify | **yes** (extended) |
| `nav/planner.py` | Boustrophedon stripes with **polygon + circle** keep-out subtraction; **A\*** lawn-only connectors + string-pull; crosscut second pass | **yes** |
| `nav/controller.py` | **Pure-pursuit** (primary, → steering angle δ) + **PID** heading trim; outputs δ saturated at δ_max | **yes** (extended) |
| `nav/mission.py` | State machine (IDLE/TEACH/AUTO/PAUSED/MANUAL/RECOVER/STUCK/DONE), waypoint sequencing, reach radius, pass tracking, **resume-from-nearest** | **yes** (extended) |
| `nav/teach.py` | Drive-to-map: breadcrumb recording, simplify, **auto loop-closure** for keep-outs, writes `boundary.yaml` | new |
| `drive/pca9685.py` | Low-level PCA9685 over I2C (freq, per-channel duty/full-on/off) | new |
| `drive/motor_hat.py` | Waveshare 15364 TB6612: drive **both** motors as one throttle `set_throttle(signed_duty)` | new |
| `drive/servo.py` | pigpio 50 Hz steering servo: `set_steer(delta)` → calibrated pulse width | new |
| `drive/kinematics.py` | **Ackermann**: (v, δ) → both-motor throttle duty + servo pulse; R_min aware | **yes** (sim inverse, car model) |
| `power/pisugar.py` | Query `get battery`, `get battery_power_plugged` via `/tmp/pisugar-server.sock` | new |
| `safety/monitor.py` | Battery/geofence/watchdog/RTK-age/stop-cmd → stop drive motors | new |
| `safety/stuck.py` | Progress-stall + odom-vs-GPS mismatch detection, bounded recovery, **60 s give-up → STUCK** | new |
| `teleop.py` | Manual command handling, arcade/tank mapping, **deadman heartbeat** | new |
| `ui/server.py` | FastAPI + WebSocket: push live state ~10 Hz, accept commands; serves dashboard | new |
| `ui/static/*` | Canvas dashboard (port of the simulator renderer) + controls + teach screen | new (reuse sim) |
| `telemetry/logger.py` | CSV/JSON log of pose, fix quality, gains, coverage | new |
| `main.py` | Scheduler: control loop @ 20 Hz, GPS @ 1 Hz, safety @ 10 Hz | new |

---

## 5. Coordinate frames (critical port detail)

The sim works in flat **meters**; real GPS is **lat/lon**. All planning/control happens in a local ENU tangent plane.

- Choose a reference origin `(lat0, lon0)` = centroid of the boundary.
- Small-area equirectangular (fine for a yard < ~200 m):
  - `east_m  = (lon - lon0) * cos(lat0) * 111320`
  - `north_m = (lat - lat0) * 110540`
  - inverse for any commanded point back to lat/lon if needed.
- For accuracy/larger sites, use `pyproj` to project to the local UTM zone instead.
- The boundary polygon and keep-out circles are stored in lat/lon (see §6), converted to ENU once at startup, then fed to `planner.py` exactly as the sim's `poly`/`obstacles`.

---

## 6. Boundary & keep-out teaching (drive-to-map)

Define the geofence and all no-go zones by **driving the rover** — no typing coordinates. Both modes run under manual teleop (§12) and are done in **RTK-fixed** for cm accuracy. The live GUI map (§11) shows everything as it's recorded.

### A) Perimeter mapping → the geofence
1. Operator taps **Map Perimeter** and drives the rover around the outside edge of the lawn.
2. The recorder drops a breadcrumb every `teach.sample_m` (≈0.25 m) of travel (distance-deduplicated).
3. The map shows the growing track plus a live **"distance to start"** readout.
4. On **Finish**, connect last→first, run **Douglas–Peucker** simplification (`teach.simplify_m` ≈0.15 m) to a clean vertex list, validate (closed, simple, area ≥ min), and store as the boundary polygon.
5. Optionally **inset** the boundary inward by body radius so the planner stays safely inside the recorded edge.

### B) Keep-out mapping → no-go zones, with auto loop-closure
1. Operator taps **Map Keep-out** and drives a loop **around** a tree / rock / bed.
2. **Auto loop-closure**: once the loop's cumulative length > `teach.loop_min_perimeter_m` (≈2 m) **and** the live position returns within `teach.loop_closure_m` (≈0.5 m) of the loop's start, the system detects a completed circle, closes the polygon, and registers the enclosed area as a **no-go keep-out**.
3. The UI shows a closure hint — a highlighted start marker and **"X.X m to close"** — so the operator knows when the loop will snap shut. Closure either auto-confirms or prompts, per `teach.auto_confirm_closure`.
4. Simplify + validate the loop (reject self-intersecting or tiny loops). Repeat per object; each becomes its own keep-out polygon.

### Data model — keep-outs are polygons (and still support circles)
Driven loops are stored as **polygons**, so `nav/geometry.py` and `nav/planner.py` must generalize keep-out handling beyond circles: point-in-polygon for the drivable grid + body inflation, and scan-line **interval subtraction against keep-out polygons** so a driven blob around a tree carves the correct hole in the coverage stripes. Legacy circular keep-outs remain valid.

### Output — `boundary.yaml` (extended)
```yaml
origin: {lat: 45.0000000, lon: -73.0000000}
boundary:            # ordered lat/lon perimeter (driven + simplified)
  - {lat: ..., lon: ...}
keepouts:            # each a driven polygon (or legacy circle)
  - {type: polygon, label: "oak tree", points: [{lat: ..., lon: ...}, ...]}
  - {type: circle,  label: "boulder",  lat: ..., lon: ..., r_m: 0.7}
inset_m: 0.12        # optional boundary inset
```

### Editing (all from the GUI)
Undo last breadcrumb, discard current track, delete or rename a saved zone, re-open a zone to redraw. `tools/teach_boundary.py` is a CLI fallback, but the GUI teach screen (§11) is the primary path.

## 7. Drive kinematics — Ackermann / car-like (`drive/kinematics.py`)

The platform is an **RC car**: two brushed drive motors (front + rear) for propulsion and a **steering servo** on the front wheels. That's **Ackermann** steering, not differential — it **cannot pivot in place** and has a **minimum turning radius**.

Bicycle model:
- Controller outputs: forward speed `v` and **steering angle `δ`** (not a differential `ω`).
- Yaw rate `ω = v · tan(δ) / L`, where `L = wheelbase` (front–rear axle distance).
- **Minimum turning radius** `R_min = L / tan(δ_max)`.

Actuation:
- **Throttle** — drive **both motors together**, same direction + duty: `duty = clamp(v / V_MAX, -1, +1)` with deadband comp. (Both motors = AWD and split current; one motor alone also works.) Sign → TB6612 direction pins; magnitude → PWM duty.
- **Steering** — map `δ` → servo pulse: `us = us_center + (δ/δ_max)·(us_max − us_center)`, clamp to `[us_min, us_max]`, output on the **pigpio servo pin @ 50 Hz**.
- Calibrate `V_MAX`/deadband (`tools/motor_calibrate.py`), and `L`, `δ_max`, `us_center/min/max` (a steering-calibration step in the GUI or a CLI).
- Reverse = both motors reversed; combine with steering for **3-point turns** (no pivot).

---

## 8. Control loop (mirror the sim, with real timing)

Run at **CTRL_HZ = 20** (configurable). GPS arrives at **1 Hz**; never block the loop on it.

Per control tick:
1. `estimator.update()` → fused pose `(x, y, θ)` (dead-reckon every tick; snap toward GPS only when a new fix landed — same complementary blend as the sim's `ALPHA`).
2. Target = current waypoint. Compute `bearing` from pose→waypoint, `err = wrap(bearing − θ)`, signed cross-track.
3. **Controller** → **steering angle `δ`** (+ speed):
   - **Pure-pursuit is the primary controller** for Ackermann: aim at a lookahead point `Ld` along the path, `δ = atan(2·L·sin(α)/Ld)`, clamped to `±δ_max`. Sparse 1 Hz fixes tolerate this far better than point-chasing.
   - A **PID** (`Kp,Ki,Kd`, anti-windup, saturation) on heading error trims → steering as an alternative/inner loop. Output is a **steering angle saturated at `δ_max`**, not a differential yaw rate.
4. Forward speed `v = V_NOMINAL · max(0.3, cos(δ))` — ease off in tight steering; never command a turn sharper than `R_min`.
5. `mission.update()`: if `range < REACH` advance waypoint; manage pass 1→2; enter **RECOVER** (steer hard to current waypoint, reduce speed) if pose off-lawn. Honors the state machine — autonomy is suppressed in PAUSED/MANUAL/STUCK, and on **resume from MANUAL it re-localizes to the nearest not-yet-covered waypoint** (or re-plans from current position) rather than chasing a stale target.
6. `kinematics` → **both-motor throttle duty + servo angle** → `motor_hat` + `servo`.
7. `safety.check()` may override everything to a stop.
8. Pet watchdog; log telemetry.

GPS task (1 Hz, async): read UART, parse GGA, extract **fix quality** (`0 invalid, 1 single, 2 DGPS, 4 RTK-fixed, 5 RTK-float`), update estimator + fix age. NTRIP task streams RTCM continuously to the GPS UART.

---

## 9. Planner port notes (`nav/planner.py`)

Bring over verbatim in logic:
- Boustrophedon scan-line generation along an axis (`h`/`v`), splitting each stripe and **subtracting keep-out intervals** (obstacle radius + ½ deck + margin).
- Serpentine ordering with direction flip per stripe.
- For each consecutive cut-point pair: if straight line stays on lawn (LOS check on a drivable grid = lawn − obstacle/body clearance), keep straight; else **A\*** on the drivable grid + **string-pull** to route around through lawn only.
- **Crosscut**: run a second pass perpendicular to the first; connect the passes with the same LOS/A\* logic.
- Stripe spacing = deck width so passes slightly overlap.
- Compute once at mission start (well within the Pi Zero 2 W's budget; grid at ~0.2 m). Cache the waypoint list.

### Turning-radius constraint (Ackermann)
The rover can't pivot, so coverage turns must fit `R_min`:
- Set **stripe spacing** + a **headland margin** so the U-turn between adjacent rows fits the minimum radius, **or** use a **skip-row** pattern: mow every other row outbound and catch the skipped rows on the return, turning a tight 180° into two gentler turns.
- A\* connectors still keep travel on lawn, but executed paths are curvature-limited — keep a lawn margin at the boundary for turn-around room.
- Treat *"do the U-turns fit?"* as a plan-time check given `R_min`, stripe spacing and boundary clearance; if not, auto-switch to skip-row.
- Pivot-in-place is unavailable — RECOVER and STUCK recovery use **3-point turns** (§13).

---

## 10. Safety monitor (`safety/monitor.py`)

Runs at ≥10 Hz, independent of nav. Any trip → stop drive motors, set state, log. (No blade — these keep the rover contained and the run clean.)
- **Battery** (PiSugar): `< RETURN_PCT` (e.g. 25%) → finish/park; `< STOP_PCT` (e.g. 12%) → stop; let pisugar-server handle `safe_shutdown_level`.
- **Geofence**: pose outside boundary + `GEO_MARGIN` → stop.
- **RTK age/quality**: `fix_age > FIX_MAX_AGE` or quality worse than `MIN_FIX_QUALITY` for `> DEGRADE_SEC` → stop (or downshift speed).
- **Watchdog**: control loop heartbeat missed → stop.
- **Stop command**: software/button stop sets the disarm flag.
- Motors also have a **command timeout** in `motor_hat.py`: if no fresh command within `MOTOR_TIMEOUT_MS`, auto-stop (covers a crashed loop).

---

## 11. Operator interface (web GUI)

A phone/laptop dashboard, **served by the Pi**, that looks and behaves like the simulator console — live map + telemetry — plus mission control, manual override, and the teaching workflow. The Pi is headless; the operator connects over Wi-Fi. **The existing simulator HTML is the canonical look-and-feel and rendering reference.**

### Tech (kept light for the Pi Zero 2 W)
- **Backend**: FastAPI + uvicorn (or aiohttp). One **WebSocket** pushes live state (server→client, ≈10 Hz JSON) and carries commands (client→server). REST for one-shot actions (save/load boundary, plan mission).
- **Frontend**: static HTML/CSS/JS with a **Canvas** renderer — reuse the simulator's drawing code and visual style verbatim where possible (same field transform, grass/cut cells, keep-outs + inflation rings, planned path, rover triangle + heading vector, GPS crosshair, fused-estimate ring). No heavy framework; serve from `ui/static/`.
- Served on `http://<pi>:<ui.port>` (e.g. 8080). PiSugar's own web UI stays on 8421.

### Live map layers (mirror the sim)
Lawn polygon · keep-out zones with inflation rings · planned coverage path (pass 1 + crosscut) · covered cells · current waypoint + reach radius · rover pose + heading arrow · raw 1 Hz GPS fix (held) · fused estimate · breadcrumb track during teaching. Auto-fit with pan/zoom, north-up, scale bar, follow-rover toggle.

### Telemetry panel (live)
- Fix mode/quality (single / DGPS / RTK-float / RTK-fixed) + **fix age**, satellites, HDOP.
- Battery % + charging (PiSugar) + estimated runtime.
- **Mode/state**: IDLE / TEACH / AUTO / PAUSED / MANUAL / RECOVER / STUCK / DONE.
- Heading, target bearing, heading error, cross-track, ω command, **P/I/D term breakdown** (as in the sim PID panel).
- Coverage %, distance run, mission time, current pass, waypoint i/N.
- **Event/alert feed**: stuck, geofence breach, fix loss, low battery, loop-closure.

### Controls
- Mission: **Plan · Start · Pause · Resume · Stop · Re-plan from current position**.
- **Mode toggle AUTO ⇄ MANUAL** (see §12).
- **Teach screen**: Map Perimeter · Map Keep-out · Finish · Undo · Save (see §6).
- **Live tuning sliders**: Kp / Ki / Kd, nominal speed, lookahead — pushed to the controller live, same affordance as the sim.
- Layer toggles.

### Connection robustness
The UI is an operator **console, not the control authority**. State push and autonomy continue regardless of the UI. If the browser disconnects, autonomy keeps running but **manual/teleop stops immediately** via the deadman (§12).

---

## 12. Manual control / teleop override

Purpose: instantly take over to nudge the rover out of trouble or reposition it, then hand control back to autonomy.

### Behavior
- **MANUAL toggle pauses autonomy**: the planner/controller stop emitting commands; operator inputs go straight to `drive/kinematics` → motors.
- **Inputs** (any of): on-screen virtual joystick (touch), arrow/WASD keys, or a USB/Bluetooth **gamepad via the browser Gamepad API**.
- **Mapping**: stick Y → forward speed `v` (scaled by a conservative manual speed-limit slider); stick X → yaw rate `ω`. Arcade/tank selectable.
- **Deadman / heartbeat**: every teleop command carries a timestamp; if none arrives within `teleop.deadman_ms` (≈300 ms) — key released, app backgrounded, Wi-Fi dropped — **motors stop** (reuses the motor command-timeout).
- **Geofence still enforced** in MANUAL (warn + stop at the fence) unless an explicit "override fence" is held — useful to free the rover if a bad map traps it.

### Handing back to AUTO
On resume, **do not chase a stale waypoint**: re-localize to the nearest not-yet-covered waypoint on the planned path, or offer **Re-plan from here**. This is the intended fix flow after STUCK: switch MANUAL → drive clear → Resume AUTO.

---

## 13. Stuck detection & give-up protocol

The rover must **never grind indefinitely**.

### Detection (only while it should be moving — AUTO/RECOVER with commanded `v>0`; never in PAUSED/MANUAL/awaiting-fix)
- Track **net displacement** of the fused position over a sliding window `stuck.window_s` (≈20 s). If displacement < `stuck.min_progress_m` (≈0.3 m) while motion was commanded → **progress stall**.
- Faster cross-check: **wheel odometry reports motion but GPS/fused position isn't advancing** ⇒ wheels slipping or blocked → trigger sooner.

### Escalation (bounded, then stop)
1. On first stall, attempt a **bounded recovery maneuver**: stop → back up → steer + pull forward (a **3-point turn**, since it can't pivot) → retry the waypoint. Limited to `stuck.recovery_tries` (≈2).
2. If still not progressing and total stuck time reaches `stuck.giveup_s` (**60 s, per spec**): enter **STUCK** — **stop the motors, stop trying**, raise a prominent UI alert + log entry, and wait for the operator.
3. From STUCK: operator uses MANUAL teleop (§12) to drive clear then **Resume AUTO** (re-localizes to nearest waypoint), or **Stop** the mission.

> The 60 s "same GPS location ⇒ stop trying" rule is the hard ceiling. Recovery maneuvers happen **within** that budget, not on top of it.

---

## 14. config.yaml (starting values — field-tune)

```yaml
platform: ackermann
control:
  ctrl_hz: 20
  v_nominal: 0.45         # m/s; start slow
  reach_m: 0.22
  max_steer_deg: 30       # = delta_max (servo limit)
  pure_pursuit: {enabled: true, lookahead_m: 0.6}   # primary for Ackermann
  pid: {kp: 3.0, ki: 0.30, kd: 0.35, imax: 1.0}     # trims heading -> steering
geometry:
  wheelbase_m: 0.25       # front-rear axle distance — MEASURE
  steer_max_deg: 30       # full lock at the wheels — MEASURE
  # min_turn_radius_m = wheelbase / tan(steer_max)  (computed)
  deck_m: 0.50            # coverage swath width
  body_clearance_m: 0.12
drive:
  pca9685_addr: 0x40
  pwm_hz: 1000
  motor_rear:  {pwm_ch: 0, in1_ch: 1, in2_ch: 2}   # verify vs Waveshare example
  motor_front: {pwm_ch: 5, in1_ch: 3, in2_ch: 4}
  drive_both: true        # both motors share one throttle
  v_max: 1.2              # m/s at 100% duty — from calibration
  deadband: 0.08
steering:
  gpio_pin: 18            # pigpio hardware-timed PWM (needs pigpiod)
  pwm_hz: 50
  us_center: 1500         # wheels straight — CALIBRATE
  us_min: 1000            # full lock one way
  us_max: 2000            # full lock other way
  invert: false
gnss:
  port: /dev/serial0
  baud: 115200            # try 460800 if no data
  min_fix_quality: 4      # 4=RTK-fixed (5=float acceptable? decide)
  fix_max_age_s: 3.0
ntrip:
  host: ""                # caster
  port: 2101
  mountpoint: ""
  user: ""
  password: ""
estimator:
  gps_blend_alpha: 0.5
  imu_yaw_offset_deg: 0.0
safety:
  geo_margin_m: 0.5
  return_pct: 25
  stop_pct: 12
  degrade_sec: 5
  motor_timeout_ms: 300
ui:
  port: 8080
  push_hz: 10
teleop:
  deadman_ms: 300
  manual_v_max: 0.35
  manual_steer_deg: 30
  enforce_geofence: true
stuck:
  window_s: 20
  min_progress_m: 0.3
  recovery_tries: 2
  giveup_s: 60          # hard ceiling: same GPS location ⇒ stop trying
teach:
  sample_m: 0.25
  simplify_m: 0.15
  loop_min_perimeter_m: 2.0
  loop_closure_m: 0.5
  auto_confirm_closure: false
  boundary_inset_m: 0.12
```

---

## 15. Repo layout

```
lawnbot/
  config.yaml
  main.py
  lawnbot/
    config.py
    gnss/{lc29h.py, ntrip.py}
    sensors/{imu.py, odometry.py}
    estimator.py
    nav/{geo.py, geometry.py, planner.py, controller.py, mission.py, teach.py}
    drive/{pca9685.py, motor_hat.py, servo.py, kinematics.py}
    power/pisugar.py
    safety/{monitor.py, stuck.py}
    teleop.py
    ui/{server.py, static/{index.html, app.js, style.css}}
    telemetry/logger.py
  tools/{teach_boundary.py, motor_calibrate.py, gps_monitor.py}
  systemd/lawnbot.service
  tests/
  boundary.yaml
```
Provide `systemd/lawnbot.service` (auto-start, `Restart=on-failure`, after network + pisugar).

---

## 16. Build phases (incremental — verify each before the next)

1. **Bench I/O**: spin **both motors** fwd/rev at low duty (wheels off the ground); **center + sweep the steering servo** and record full-lock pulse widths + `δ_max`. Confirm it can't pivot — turns need forward motion.
2. **GPS bring-up**: `gps_monitor.py` prints lat/lon + fix quality; confirm a fix.
3. **NTRIP / RTK-fixed**: stream RTCM, confirm quality reaches 4 (allow ≤60 s convergence). Log accuracy.
4. **IMU + odometry**: verify yaw and per-wheel distance; calibrate `V_MAX`, `TRACK`, deadband.
5. **Estimator**: drive a known square by hand; confirm fused ENU track matches reality within a few cm and that 1 Hz fixes correct drift smoothly.
6. **Closed-loop point-to-point**: command a single waypoint; tune PID, then try pure-pursuit.
7. **Web UI bring-up**: serve the dashboard; confirm live telemetry + map render (replay against a recorded log first, then live).
8. **Teleop**: drive manually via the UI joystick / keys / gamepad; confirm the **deadman stops motors** on disconnect/key-release.
9. **Teach workflow**: drive the perimeter and map one keep-out with **auto loop-closure**; confirm `boundary.yaml` saves and re-loads into the planner.
10. **Planner**: load the taught boundary + keep-out; confirm lawn-only A\* connectors and crosscut.
11. **Full coverage run**: run the mission on a small real plot; measure coverage %, watch RECOVER/geofence behavior.
12. **Stuck protocol**: physically block the rover; confirm bounded recovery then **STUCK stop within 60 s**, no grinding; recover via teleop + Resume.
13. **Safeguard soak**: trip each safeguard deliberately (low battery, geofence, RTK loss, watchdog, stop command) and confirm the rover stops.

---

## 17. Acceptance criteria

- Holds RTK-fixed and stops if it degrades beyond config.
- Never commands a path segment crossing non-lawn (planner LOS/A\* guarantees).
- Control loop ≥ 20 Hz steady; estimator error vs RTK-fixed < ~5 cm during straight runs.
- Coverage ≥ 95% of lawn area (minus the unreachable ring around keep-outs/boundary) with crosscut on.
- Every safeguard independently stops the drive motors within < 0.5 s.
- Clean recovery after a manual push off-path.
- **GUI** shows live state at ≥5 Hz and the map matches reality; controls and live gain sliders take effect.
- **Manual override** takes control in < 200 ms; the **deadman stops motors** within `deadman_ms` of a dropped connection.
- **Stuck** is detected and the rover **stops within 60 s** with no grinding; it resumes cleanly to the nearest waypoint after a manual clear.
- The **geofence and all keep-outs can be created by driving only** (no typed coordinates); a keep-out loop **auto-closes** and produces a valid `boundary.yaml` that loads into the planner.

---

## 18. Open items to confirm before coding
- IMU part number (BNO085 strongly recommended for onboard yaw fusion).
- Encoder type/CPR on the drive motors.
- Exact PiSugar model ("S"): confirm I2C addresses with `i2cdetect` and select the matching model in the installer.
- Motor supply topology (shared vs separate pack for the two drive motors).
- NTRIP caster credentials / whether a local base is used.
- Verify the Waveshare 15364 PCA9685→TB6612 channel map against the unit's own example code.
- Manual-control input device: on-screen joystick only, or also a USB/Bluetooth gamepad?
- Confirm polygon keep-outs (from driven loops) are acceptable vs. the sim's circular model — planner must generalize either way.
- Auto-confirm loop closure, or always prompt the operator?
- **Measure `wheelbase_m` and full-lock `steer_max_deg`** → these set `R_min`, which constrains stripe spacing / skip-row.
- Calibrate the steering servo pulse limits (`us_center/min/max`) and check steering isn't reversed.
- **Confirm the TB6612 can handle the RC motors' current** (else move drive to a higher-current driver).
- Servo power source (UBEC rating) and that all grounds are common.
- Field NTRIP plan: phone hotspot vs local base vs accept float/DGPS offline.

---

## References
- LC29H series / DR&RTK app note (DA = RTK-only, 1 Hz): https://www.quectel.com/product/gnss-lc29h/ · https://forums.quectel.com/uploads/short-url/ncdKSn5cRYyDbOpfYmvGGs1vEIA.pdf
- LC29H hardware design (RTK-fixed cm, <60 s convergence): https://forums.quectel.com/uploads/short-url/u7fE3GaEAynRmZ3J7ihhjXrwxTF.pdf
- Waveshare Motor Driver HAT (PCA9685 + TB6612, 0x40): https://www.waveshare.com/wiki/Motor_Driver_HAT · manual: https://www.waveshare.com/w/upload/8/81/Motor_Driver_HAT_User_Manual_EN.pdf
- PiSugar power manager + I2C addresses: https://github.com/PiSugar/PiSugar/wiki/PiSugar-Power-Manager-(Software) · https://docs.pisugar.com/docs/product-wiki/battery/pisugar2/pisugar-2
- NTRIP client: https://github.com/semuconsulting/pygnssutils
