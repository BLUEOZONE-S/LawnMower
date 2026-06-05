# LawnBot

Autonomous coverage rover on a Raspberry Pi Zero 2 W + RC-car chassis. No blade — this is a research rover that *behaves* like a robotic mower (drives a coverage path and logs covered cells).

Canonical design spec: [`LAWNBOT_BUILD_BRIEF.md`](LAWNBOT_BUILD_BRIEF.md). This README is the **operator setup guide** — read top to bottom the first time, then refer back as needed.

---

## Contents

1. [Bill of materials](#1-bill-of-materials)
2. [How the interface works](#2-how-the-interface-works)
3. [Hardware assembly (wiring)](#3-hardware-assembly-wiring)
4. [Pi software setup](#4-pi-software-setup-from-a-fresh-sd-card)
5. [Moving the project from Windows to the Pi](#5-moving-the-project-from-windows-to-the-pi)
6. [First-boot bring-up](#6-first-boot-bring-up)
7. [Running as a service (autostart)](#7-running-as-a-service-autostart)
8. [Field networking (Wi-Fi hotspot + NTRIP)](#8-field-networking-wi-fi-hotspot--ntrip)
9. [Iterating: redeploying after code changes](#9-iterating-redeploying-after-code-changes)
10. [Repo layout](#10-repo-layout)
11. [Dev-box tests (Windows)](#11-dev-box-tests-windows)
12. [Safety notes](#12-safety-notes)

---

## 1. Bill of materials

| | Part | Role |
|---|---|---|
| Compute | Raspberry Pi Zero 2 W | Control loop, planner, fusion |
| SD card | ≥16 GB, A1/A2-class | OS + app |
| Chassis | RC car (Ackermann steering) | 2 brushed drive motors (front+rear AWD) + 1 front steering servo |
| Motor driver | Waveshare Motor Driver HAT **15364** | Drives the 2 brushed motors (PCA9685 @ 0x40 + TB6612FNG) |
| GNSS | Quectel LC29H(DA) | 1 Hz RTK over UART |
| Power/UPS | PiSugar (S/2/3-class) | Battery, charge mgmt, safe shutdown, RTC |
| Servo power | 5–6 V UBEC (~3 A) | Dedicated servo rail |
| IMU (recommended) | BNO085 | Absolute heading (yaw) for fusion |
| Encoder (recommended) | Quadrature on drive shaft | Odometry between 1 Hz fixes |
| Misc | Jumper wires, JST/Dupont, heatshrink, common-ground rail | Wiring |

---

## 2. How the interface works

**You don't drive this from a Python prompt.** The code runs as a background service on the Pi; you talk to it through a **web browser**.

| Layer | Where | How you reach it |
|---|---|---|
| Runtime / controller | Python on the Pi (systemd autostart) | invisible — boots with the Pi |
| Operator interface | Web UI served by the Pi | `http://<pi>:8080` from any phone or laptop on the same network |
| Bring-up CLIs | Python on the Pi | SSH in, run `tools/motor_calibrate.py` or `tools/gps_monitor.py` |

In the field, the Pi runs its **own Wi-Fi access point** so your phone joins the Pi directly — no router needed. See [Section 8](#8-field-networking-wi-fi-hotspot--ntrip).

---

## 3. Hardware assembly (wiring)

### 3.1 Power topology — three rails, one common ground

```
RC battery (7.4 V LiPo, 2S) ──┬──► HAT VIN ───────► both brushed motors (TB6612)
                              │
                              └──► UBEC (5–6 V) ──► steering servo
PiSugar (its own LiPo) ─────────────────────────► Pi 5 V (compute only)

      ──► GND of EVERYTHING tied together ◄──
      (Pi GND, HAT GND, UBEC GND, servo GND, battery GND)
```

Why three rails:
- **Motors brown out the Pi if they share its rail.** Bulk capacitance on the HAT helps but separation is the right answer.
- **Servos draw spiky stall current.** UBEC isolates that from the Pi too.
- **Common ground is mandatory.** The pigpio servo signal on GPIO18 needs a shared ground reference with the UBEC powering the servo.

### 3.2 Motor HAT → drive motors

Mount the Waveshare 15364 HAT directly on the Pi's 40-pin header. It uses only I2C (SDA/SCL/3V3/5V) + GND from the header.

Connect the two RC-car drive motors to the HAT's screw terminals:

| HAT terminal | Wire to |
|---|---|
| **Motor A** (A+, A-) | rear drive motor (any polarity — `invert_rear: true` in config flips it later) |
| **Motor B** (B+, B-) | front drive motor |
| **VIN, GND** | RC battery (6–12 V) |

Software channel mapping (`config.yaml > drive`):
- Motor A — PWM ch 0, IN1 ch 1, IN2 ch 2
- Motor B — PWM ch 5, IN1 ch 3, IN2 ch 4
- Both motors driven with **identical direction + duty** (AWD shares current)
- PWM ≈ 1 kHz

> ⚠️ **TB6612 current ceiling: ~1.2 A continuous per channel (~3.2 A peak).** RC 540-class motors can exceed this under load. Bench-test at low duty first; watch for thermal cutout. If it can't keep up, swap to a higher-current driver.

### 3.3 Steering servo → Pi GPIO 18 + UBEC (NOT the HAT)

The PCA9685 on the HAT runs all 16 channels at ONE frequency. Motors want ~1 kHz; servos want 50 Hz. So the servo goes on a Pi GPIO under pigpio (hardware-timed 50 Hz, jitter-free).

| Servo pin | Goes to |
|---|---|
| **Signal** (white/yellow) | **Pi GPIO 18** (pin 12 on the header) |
| **+V** (red) | **UBEC 5–6 V output** — NOT the Pi 5 V rail |
| **GND** (black/brown) | Common ground |

Wire the UBEC's input to the RC battery (same battery feeding the HAT VIN is fine).

### 3.4 GPS (LC29H DA) → Pi UART

| LC29H pin | Pi pin | Pi header pin |
|---|---|---|
| TX | **GPIO 15 (RXD)** | pin 10 |
| RX | **GPIO 14 (TXD)** | pin 8 |
| VCC | **3.3 V** | pin 1 or 17 |
| GND | Common ground | pin 6/9/14/20/25/30/34/39 |

The UART is freed for the GPS by **disabling the serial console** and **disabling Bluetooth** (Bluetooth normally owns the high-quality PL011 UART on the Pi). Both are done in Section 4.

> Connect the GPS **antenna** before powering on. Active antennas need a clear sky view — bring-up outdoors.

### 3.5 IMU (BNO085) → I2C bus 1

Shares the I2C bus with the Motor HAT and PiSugar. Default address **0x4A** (ADR pin low / floating); 0x4B if ADR is tied high. No conflict with any other device on the bus.

| BNO085 pin | Pi pin |
|---|---|
| SDA | **GPIO 2** (pin 3) |
| SCL | **GPIO 3** (pin 5) |
| VCC | 3.3 V (pin 1) |
| GND | Common ground |

**If you have an Adafruit / Stemma QT breakout** (recommended), the boot-mode pins are already wired correctly on the board — just connect SDA/SCL/3V3/GND.

**If you have a bare BNO085 chip / generic breakout** without those pins handled, you also need to tie:
- **PS0 = GND**, **PS1 = GND** → selects I2C mode (vs UART / SPI)
- **BOOTN = 3.3 V** → normal boot (not bootloader)
- **RSTN = 3.3 V** (or via a 10 kΩ pull-up) → out of reset

The interrupt pin (INTN) is **not** used by this project — the driver polls. Leave it disconnected.

If you skip the IMU initially, the runtime falls back to a stub yaw of 0 — the rover will drift between GPS fixes but bring-up still works.

### 3.6 Drive encoder (optional) → GPIO

A/B quadrature pins on any two interrupt-capable GPIOs. Defaults the code expects:

| Encoder pin | Pi pin |
|---|---|
| A | **GPIO 23** (pin 16) |
| B | **GPIO 24** (pin 18) |
| VCC | 3.3 V or 5 V (whichever the encoder takes) |
| GND | Common ground |

> The encoder pins are passed to `QuadratureEncoder(pin_a, pin_b, …)` at construction time (see `lawnbot/sensors/odometry.py`). The runtime currently uses `StubOdometry` — when you wire a real encoder, replace the stub instantiation in `main.py` with `QuadratureEncoder(23, 24, counts_per_rev=…, wheel_circumference_m=…)`.

Like the IMU, this is optional for bring-up. Without it, dead-reckoning between fixes uses the commanded velocity only (poor under wheel slip).

### 3.7 PiSugar

The PiSugar sits **under** the Pi (between Pi and HAT, or below the Pi if your stack-up allows). It feeds 5 V to the Pi through the GPIO header pads from below. Follow the PiSugar wiki for the model you have:

- PiSugar 2: <https://docs.pisugar.com/docs/product-wiki/battery/pisugar2/pisugar-2>
- PiSugar 3 / S: <https://docs.pisugar.com/docs/product-wiki/battery/pisugar3>

After mounting, run `i2cdetect -y 1` and confirm the PiSugar's I2C address shows up (see [Section 4.6](#46-verify-buses)).

### 3.8 Bench-test rule before powering on

**Wheels off the ground.** Chassis on a stand. Always. A runaway PWM duty at first bring-up will send the rover across the room.

### 3.9 Pin & address assignment matrix (single-page verification)

Cross-reference this against your wiring **before** powering up. Every entry is BCM (GPIO N) and physical (pin N) — those numberings are NOT the same.

**GPIO usage on the Pi 40-pin header:**

| GPIO (BCM) | Pin (physical) | Used by | Direction / signal |
|---|---|---|---|
| GPIO 0  | pin 27 | **HAT EEPROM (ID_SD)** — reserved | reserved by Pi at boot, do not repurpose |
| GPIO 1  | pin 28 | **HAT EEPROM (ID_SC)** — reserved | reserved by Pi at boot, do not repurpose |
| GPIO 2  | pin 3  | I2C-1 SDA (HAT 0x40, IMU 0x4A, PiSugar) | digital, open-drain, shared |
| GPIO 3  | pin 5  | I2C-1 SCL (HAT 0x40, IMU 0x4A, PiSugar) | digital, open-drain, shared |
| GPIO 14 | pin 8  | UART TXD → LC29H RX | digital out (Pi sends RTCM here) |
| GPIO 15 | pin 10 | UART RXD ← LC29H TX | digital in (Pi receives NMEA here) |
| GPIO 18 | pin 12 | Steering servo signal (pigpio 50 Hz PWM) | digital out, hardware-PWM capable |
| GPIO 23 | pin 16 | Drive encoder A (optional) | digital in, pull-up, interrupt |
| GPIO 24 | pin 18 | Drive encoder B (optional) | digital in, pull-up, interrupt |
| all other GPIO | — | **free** | — |

> **No analog inputs are used.** The Pi has no on-board ADC. Every sensor on this rover is either I2C (HAT, IMU, PiSugar), UART (GPS), or digital pulse (encoder, servo). If you later add an analog sensor, route it through an MCP3008 or ADS1115 on I2C — don't expect the Pi to read voltages directly.

**Power pins used:**

| Pin | Role |
|---|---|
| pin 1 (3V3) | LC29H VCC + BNO085 VCC. Active GNSS antenna passthrough adds ~10 mA — combined load ~50 mA, within budget. |
| pin 2 / 4 (5V) | PiSugar feeds 5V to the Pi *into* these pads from below; nothing else draws from them |
| pin 6/9/14/20/25/30/34/39 (GND) | tie ALL grounds (Pi, HAT, UBEC, servo, RC battery) here |

**I2C bus 1 address map (run `i2cdetect -y 1` to verify):**

| Address | Device | Notes |
|---|---|---|
| 0x32 | PiSugar 2 RTC | only present on PiSugar 2/S |
| 0x40 | Waveshare 15364 HAT (PCA9685) | always present once HAT is mounted |
| 0x4A | BNO085 IMU | (0x4B if ADR tied high) |
| 0x57 | PiSugar 3 | only present on PiSugar 3 |
| 0x68 | PiSugar 3 RTC | only present on PiSugar 3 |
| 0x70 | PCA9685 ALLCALL alias | shows up alongside 0x40 |
| 0x75 | PiSugar 2 / S power IC | only present on PiSugar 2/S |

**PCA9685 (Motor HAT) channel allocation:**

| PCA9685 channel | Connected to | Role |
|---|---|---|
| 0 | TB6612 PWMA | Motor A (rear) speed |
| 1 | TB6612 AIN1 | Motor A direction |
| 2 | TB6612 AIN2 | Motor A direction |
| 3 | TB6612 BIN1 | Motor B (front) direction |
| 4 | TB6612 BIN2 | Motor B direction |
| 5 | TB6612 PWMB | Motor B speed |
| 6–15 | unused | not broken out on the 15364 |

**Frequency separation (the reason the servo is NOT on the HAT):**

| Source | Frequency | Driver |
|---|---|---|
| PCA9685 (channels 0-5, motors) | ~1 kHz (HAT range 40-1000 Hz) | Adafruit / smbus2 over I2C |
| GPIO 18 (servo) | 50 Hz | pigpio DMA on a Pi GPIO |

These run on **physically different PWM generators** — no shared timing, no interference. That's the whole reason the brief routes the servo to a Pi GPIO instead of a PCA9685 spare channel.

**Implications of `dtoverlay=disable-bt`:**

- ✅ PL011 (good UART) moves to GPIO 14/15 for the GPS — required for reliable 115200/460800 baud
- ✅ Wi-Fi is unaffected (separate radio function)
- ❌ Bluetooth is off → **no Bluetooth gamepad** for teleop. The web UI joystick, keyboard, and USB gamepads (via browser Gamepad API) all still work. If you need BT, use `dtoverlay=miniuart-bt` instead, accepting slower BT and slightly degraded UART for the GPS.

---

## 4. Pi software setup (from a fresh SD card)

### 4.1 Flash the OS

1. Download **Raspberry Pi Imager**: <https://www.raspberrypi.com/software/>
2. Choose **OS: Raspberry Pi OS Lite (64-bit)** (under "Other general-purpose OS" → "Raspberry Pi OS (other)")
3. Click the gear icon and pre-configure:
   - **Hostname:** `lawnbot`
   - **User:** `pi`, set a password
   - **Wi-Fi:** your home Wi-Fi (so the first boot is reachable). You'll switch to AP mode later.
   - **Enable SSH:** yes, password auth
   - **Locale:** your timezone + keyboard layout
4. Write the SD card.

### 4.2 First boot

Insert the SD into the Pi, power it on, and from the Windows dev box:

```powershell
ssh pi@lawnbot.local
# accept fingerprint, enter password
```

If `lawnbot.local` doesn't resolve, find the Pi's IP in your router admin UI and use that instead.

### 4.3 One-shot setup script (preferred)

Once you've copied the repo to the Pi (see [Section 5](#5-moving-the-project-from-windows-to-the-pi) — do that first), there's a single script that does everything in §4.4 through §4.10 below:

```bash
cd /home/pi/lawnbot
sudo bash setup_pi.sh
```

It is **idempotent** — safe to re-run after editing `requirements.txt` or if something failed midway. It:

1. Confirms the OS / hardware
2. `apt update` + installs system packages
3. Enables I2C, UART hardware, disables the serial console
4. Edits `/boot/firmware/config.txt` (`enable_uart=1`, `dtoverlay=disable-bt`, `arm_boost=1`)
5. Enables + starts `pigpiod`
6. Installs the PiSugar power-manager (asks you to pick the model — set `LAWNBOT_SKIP_PISUGAR=1` to skip)
7. Disables `bluetooth`, `avahi-daemon`, `triggerhappy`, `ModemManager`, `cups`
8. Sets the CPU governor to `performance`
9. Mounts `/var/log/lawnbot` on tmpfs (added to `/etc/fstab`)
10. Creates the Python venv and runs `pip install -r requirements.txt`
11. Installs `/etc/systemd/system/lawnbot.service` (does **not** enable it — that comes after Phase 1+2 in §6)
12. Verifies every step, runs `i2cdetect -y 1`, imports every Python module, runs `pytest`, and prints a colored pass/warn/fail summary
13. Tells you if a reboot is needed (UART changes need one)

After it finishes (and optionally reboots), you can sanity-check at any time with the **read-only** verifier:

```bash
bash verify_pi.sh
```

This makes no changes — it just reports what's wired, what's enabled, what's missing.

> Sections 4.4 through 4.10 below describe the same steps **manually** for the case where you'd rather do it by hand or you need to deviate. If you ran `setup_pi.sh`, skip ahead to [Section 6](#6-first-boot-bring-up).

### 4.4 Update + base packages

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y \
    git python3-pip python3-venv python3-smbus i2c-tools \
    pigpio \
    rsync htop
```

### 4.5 Enable hardware interfaces

```bash
sudo raspi-config nonint do_i2c 0          # enable I2C
sudo raspi-config nonint do_serial_hw 0    # enable UART hardware
sudo raspi-config nonint do_serial_cons 1  # disable serial login console
```

Then edit `/boot/firmware/config.txt`:

```bash
sudo nano /boot/firmware/config.txt
```

Add or confirm these lines (under `[all]` near the bottom):

```
enable_uart=1
dtoverlay=disable-bt       # frees PL011 UART for the GPS
arm_boost=1                # squeeze extra clock on the Zero 2 W
```

### 4.6 Light up pigpiod (servo daemon)

```bash
sudo systemctl enable --now pigpiod
systemctl status pigpiod --no-pager
```

You should see `Active: active (running)`.

### 4.7 Verify buses

Reboot once to apply the UART/Bluetooth changes:

```bash
sudo reboot
# reconnect after ~30 s
ssh pi@lawnbot.local
```

Then check the buses see your hardware:

```bash
i2cdetect -y 1
```

Expect to see at least:
- `0x40` — the Motor HAT (PCA9685)
- `0x70` — PCA9685 ALLCALL alias
- A PiSugar address: `0x57` + `0x68` (PiSugar 3) or `0x75` + `0x32` (PiSugar 2)
- `0x4A` — BNO085 IMU if wired

GPS check:

```bash
ls -l /dev/serial0
# should be a symlink to /dev/ttyAMA0 (or ttyS0)
sudo cat /dev/serial0 | head -n 20
# should dump NMEA sentences like $GNGGA,... if antenna has sky view
```

### 4.8 Install the PiSugar power manager

```bash
wget https://cdn.pisugar.com/release/pisugar-power-manager.sh
bash pisugar-power-manager.sh -c release
# Select your PiSugar model when prompted.
```

This installs `pisugar-server` which exposes the battery API on `/tmp/pisugar-server.sock`. Verify:

```bash
echo "get battery" | nc -U /tmp/pisugar-server.sock
# → "battery: 87.5" or similar
```

### 4.9 Disable unused services (reclaim RAM/CPU)

```bash
sudo systemctl disable --now bluetooth avahi-daemon triggerhappy \
    ModemManager cups cups-browsed 2>/dev/null || true
```

`avahi-daemon` is the one that makes `lawnbot.local` work over mDNS — keep it on if you depend on hostname resolution.

### 4.10 CPU governor → performance

```bash
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
sudo systemctl restart cpufrequtils 2>/dev/null || true
# Or one-shot for the current boot:
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

### 4.11 (Optional) Mount logs on tmpfs

Logs on a tmpfs survive a power-cut without risking SD-card corruption.

```bash
sudo mkdir -p /var/log/lawnbot
echo "tmpfs /var/log/lawnbot tmpfs defaults,size=64M,noatime 0 0" | sudo tee -a /etc/fstab
sudo mount /var/log/lawnbot
```

---

## 5. Moving the project from Windows to the Pi

You have three options; pick whichever fits your workflow. **Option A (git)** is recommended once you have a remote — it's the cleanest for iteration. **Option B (rsync)** is the fastest from-Windows option without a remote. **Option C (one-shot SCP)** works if you just want the bits there once.

### Option A — git push/pull (recommended)

**On Windows** (once, to initialize):

```powershell
cd C:\GITHUB\LawnMower
git init
git add -A
git commit -m "Initial commit"
# Create a repo on GitHub / GitLab / your remote, then:
git remote add origin git@github.com:<you>/lawnbot.git
git push -u origin main
```

**On the Pi:**

```bash
cd /home/pi
git clone git@github.com:<you>/lawnbot.git lawnbot
# or HTTPS: git clone https://github.com/<you>/lawnbot.git lawnbot
cd lawnbot
```

To update later: `git pull` on the Pi.

### Option B — rsync from Windows (no remote required)

Use **Git Bash**, **WSL**, or any shell that has `rsync`. From `C:\GITHUB\LawnMower`:

```bash
PI_HOST=lawnbot.local ./deploy.sh
```

The `deploy.sh` script rsyncs the working tree, installs deps in the Pi venv, and restarts the systemd service.

Manual equivalent:

```bash
rsync -avz --delete \
  --exclude='.venv' --exclude='__pycache__' --exclude='.git' \
  ./ pi@lawnbot.local:/home/pi/lawnbot/
```

### Option C — one-shot SCP

From a PowerShell prompt with OpenSSH:

```powershell
scp -r C:\GITHUB\LawnMower\* pi@lawnbot.local:/home/pi/lawnbot/
```

### 5.1 Create the venv and install Python deps (on the Pi)

After any of A/B/C, on the Pi:

```bash
cd /home/pi/lawnbot
python3 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r requirements.txt
```

Pi Zero 2 W is slow — `numpy` and `fastapi` together take 5–10 minutes the first time. Subsequent installs hit the wheel cache and are fast.

### 5.2 Quick import sanity check

```bash
.venv/bin/python -c "
import lawnbot.config, lawnbot.drive.motor_hat, lawnbot.drive.servo, \
       lawnbot.gnss.lc29h, lawnbot.nav.planner, lawnbot.ui.server
print('OK')
"
```

If you get `ModuleNotFoundError`, you missed a `pip install -r requirements.txt`. If you get `pigpio` connection errors at this stage, `pigpiod` isn't running — `sudo systemctl start pigpiod`.

---

## 6. First-boot bring-up

**Do these in order. Don't skip ahead.** Each phase verifies one layer of the stack — if Phase 1 fails, Phase 6 will too, but you won't know why.

### Phase 1 — Motors + servo (wheels OFF the ground)

```bash
cd /home/pi/lawnbot
.venv/bin/python -m tools.motor_calibrate
```

This drops you into an interactive prompt. Run through:

```
> rf 0.25     # rear motor forward 25% duty — confirm direction
> rr 0.25     # rear motor reverse
> ff 0.25     # front motor forward
> fr 0.25     # front motor reverse
> bf 0.25     # both motors forward (AWD)
> br 0.25
> s           # stop
> c           # center steering
> sweep       # full servo sweep
> l           # full left lock
> r           # full right lock
> q           # quit
```

**Action items:**
- If a motor spins the wrong way, set `invert_rear: true` (or `invert_front: true`) in `config.yaml`.
- Record the actual pulse widths at full lock — adjust `steering.us_min` / `us_max` / `us_center` in `config.yaml`.
- Measure the wheelbase (front axle ↔ rear axle, in meters) and the max steer angle at the front wheels. Update `geometry.wheelbase_m` and `geometry.steer_max_deg`. These set `R_min` (minimum turning radius).

### Phase 2 — GPS

Plug the GPS antenna in, take the rover outside (sky view required).

```bash
.venv/bin/python -m tools.gps_monitor
```

You should see fixes printed once per second:

```
45.5012345, -73.4567890  qual=single     sats=10  hdop=1.20  age=0.32s
```

Wait for `qual=single` first (10–60 s with a cold start). RTK-fixed requires NTRIP corrections — that's Phase 3 (see [Section 8.2](#82-ntrip-rtk-corrections)).

### Phase 3+ — see brief §16

Phases 3-13 in [`LAWNBOT_BUILD_BRIEF.md`](LAWNBOT_BUILD_BRIEF.md): NTRIP/RTK, IMU + encoders, estimator, closed-loop point-to-point, full coverage run. They use the web UI, not the CLI tools.

---

## 7. Running as a service (autostart)

Once Phase 1 + 2 pass, register the systemd service so the app starts on boot.

```bash
sudo cp /home/pi/lawnbot/systemd/lawnbot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable lawnbot
sudo systemctl start lawnbot
```

Verify it's running:

```bash
sudo systemctl status lawnbot --no-pager
sudo journalctl -u lawnbot -f
```

You should see:

```
lawnbot: config loaded — ctrl_hz=20 ui_port=8080
lawnbot: opening motor HAT + servo
lawnbot: opening GPS UART /dev/serial0 @ 115200
lawnbot: control thread started
lawnbot: control thread: SCHED_FIFO acquired
Uvicorn running on http://0.0.0.0:8080
```

Open the UI from your phone or laptop:

```
http://lawnbot.local:8080
   or
http://<pi-ip>:8080
```

If the page loads but the WebSocket says "disconnected", check `journalctl -u lawnbot -f` for the error.

### Stopping / restarting the service

```bash
sudo systemctl stop lawnbot      # stop autostart this boot
sudo systemctl restart lawnbot   # apply a config change
sudo systemctl disable lawnbot   # stop autostarting on boot
```

---

## 8. Field networking (Wi-Fi hotspot + NTRIP)

### 8.1 Pi as a Wi-Fi access point (yard mode)

In the yard there's no router, so the Pi runs its own AP and your phone joins it. Use NetworkManager (default on Bookworm):

```bash
sudo nmcli device wifi hotspot \
    ifname wlan0 ssid LawnBot password 'changeme-1234'
# Make it permanent:
sudo nmcli connection modify Hotspot connection.autoconnect yes
sudo nmcli connection modify Hotspot ipv4.method shared
```

Connect your phone to SSID `LawnBot`, password `changeme-1234`, then browse to `http://10.42.0.1:8080` (NetworkManager's default AP subnet).

To switch back to your home Wi-Fi temporarily for `apt update` / `git pull`:

```bash
sudo nmcli connection down Hotspot
sudo nmcli connection up <your-home-wifi-name>
```

### 8.2 NTRIP (RTK corrections)

RTK-fixed requires correction data from a base station. Pick one:

**a) Phone hotspot** — the Pi joins your phone's hotspot, your phone has cellular, NTRIP corrections flow through the phone. Edit `config.yaml`:

```yaml
ntrip:
  host: "your-caster.example.com"
  port: 2101
  mountpoint: "RTCM3_NEAR"
  user: "<username>"
  password: "<password>"
```

Restart the service: `sudo systemctl restart lawnbot`.

**b) Local base station** — a second GNSS receiver in a known location, broadcasting RTCM3 over a local radio link. No internet needed. Configure the base separately; point the Pi's NTRIP client at the base's NTRIP server.

**c) Accept RTK-float / DGPS** when offline. Set `gnss.min_fix_quality: 5` (float) or `2` (DGPS) in `config.yaml`. Planner accuracy degrades to meters instead of centimeters.

---

## 9. Iterating: redeploying after code changes

After the first install, you only need one command to push changes from Windows:

```bash
PI_HOST=lawnbot.local ./deploy.sh
```

What it does:
1. rsyncs the working tree (excluding `.venv`, `.git`, logs)
2. installs any new deps into the venv
3. restarts the `lawnbot` systemd service
4. tails the last 30 lines of the journal

Pure git workflow:

```bash
# On Windows:
git push
# On the Pi:
cd /home/pi/lawnbot && git pull && sudo systemctl restart lawnbot
```

To watch logs while you iterate:

```bash
sudo journalctl -u lawnbot -f
```

---

## 10. Repo layout

```
LawnMower/
  LAWNBOT_BUILD_BRIEF.md   the design contract
  README.md                this file
  config.yaml              pins, gains, thresholds (single source of truth)
  requirements.txt         Python deps
  deploy.sh                rsync + restart on the Pi
  systemd/lawnbot.service  autostart unit
  lawnbot/
    main.py                scheduler — control loop + safety + UI
    config.py              YAML → frozen dataclasses
    drive/
      pca9685.py           PCA9685 I2C driver
      motor_hat.py         Waveshare 15364 TB6612 — set_throttle, watchdog
      servo.py             pigpiod 50 Hz steering servo on GPIO18
      kinematics.py        Ackermann (v, δ) → throttle + steer
    gnss/
      lc29h.py             threaded NMEA reader
      ntrip.py             NTRIP RTCM forwarder
    sensors/
      imu.py               BNO085 + Stub fallback
      odometry.py          quadrature encoder + Stub fallback
    estimator.py           complementary filter (IMU + odom + GPS)
    nav/
      geo.py               lat/lon ↔ ENU
      geometry.py          polygon + drivable mask + DP
      planner.py           boustrophedon + A* + crosscut
      controller.py        pure-pursuit + PID
      mission.py           AUTO/MANUAL/RECOVER/STUCK state machine
      teach.py             drive-to-map breadcrumb recorder
    power/pisugar.py       battery API over UDS
    safety/
      monitor.py           battery / geofence / RTK / watchdog
      stuck.py             60 s give-up
    teleop.py              deadman heartbeat
    telemetry/logger.py    JSONL rotating log
    ui/
      server.py            FastAPI + WebSocket
      static/{index.html, app.js, style.css}  Canvas dashboard
  tools/
    motor_calibrate.py     Phase 1 bench
    gps_monitor.py         Phase 2
    teach_boundary.py      CLI teach fallback
  tests/                   pure-math tests — run on Windows, no hardware
```

---

## 11. Dev-box tests (Windows)

Pure-math modules import and test on Windows without any hardware:

```powershell
cd C:\GITHUB\LawnMower
python -m pip install -r requirements.txt
python -m pytest tests/
```

Hardware modules (`pca9685.py`, `servo.py`, `lc29h.py`, `sensors/*`, `gnss/ntrip.py`) guard their imports so the package still loads on Windows — instantiating those classes raises a clear error, but `import lawnbot.*` works fine for syntax checks and unit tests.

---

## 12. Safety notes

No blade, but still:

- **First motor test:** wheels off the ground. Always.
- `motors.stop()` is callable from anywhere; the motor watchdog auto-stops on a stale command (300 ms default).
- **Geofence:** the rover stops if its fused position leaves the boundary polygon + margin.
- **RTK degradation:** if fix quality drops below `min_fix_quality` for longer than `degrade_sec`, the rover stops.
- **Low battery:** the PiSugar handles safe-shutdown so the SD card doesn't corrupt.
- **Stuck:** 60 s hard ceiling — if it can't progress, motors stop and it waits for the operator (use the UI joystick to free it, then Resume).
- **MANUAL teleop deadman:** if the browser disconnects or you release the stick, motors stop within 300 ms.
- **TB6612 current ceiling:** ~1.2 A continuous per channel. RC 540 motors can exceed this. Watch for thermal cutout; bench-test at low duty.
