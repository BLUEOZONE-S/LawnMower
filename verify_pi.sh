#!/usr/bin/env bash
#
# verify_pi.sh — read-only check of LawnBot setup.
# Run after setup_pi.sh + reboot to confirm everything is wired and
# configured. Does NOT change anything. Safe to run any time.
#
#     bash verify_pi.sh

C_GREEN='\033[1;32m'; C_YELLOW='\033[1;33m'; C_RED='\033[1;31m'; C_BLUE='\033[1;34m'; C_RESET='\033[0m'
PASS=0; WARN=0; FAIL=0
ok()   { echo -e "${C_GREEN}✓${C_RESET} $*"; PASS=$((PASS+1)); }
warn() { echo -e "${C_YELLOW}!${C_RESET} $*"; WARN=$((WARN+1)); }
fail() { echo -e "${C_RED}✗${C_RESET} $*"; FAIL=$((FAIL+1)); }
hdr()  { echo -e "\n${C_BLUE}─── $* ───${C_RESET}"; }

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"

hdr "Platform"
MODEL="$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo unknown)"
echo "  model: $MODEL"
echo "  kernel: $(uname -srm)"
[ "$(uname -m)" = "aarch64" ] && ok "64-bit kernel" || warn "32-bit kernel — expected 64-bit"

hdr "Boot config"
CONFIG_TXT="/boot/firmware/config.txt"
[ -f "$CONFIG_TXT" ] || CONFIG_TXT="/boot/config.txt"
echo "  reading: $CONFIG_TXT"
for key in "enable_uart=1" "dtoverlay=disable-bt"; do
    if grep -qE "^[[:space:]]*${key//\//\\/}([[:space:]]|$)" "$CONFIG_TXT"; then
        ok "$key set"
    else
        fail "$key MISSING from $CONFIG_TXT"
    fi
done
grep -qE "^[[:space:]]*arm_boost=1" "$CONFIG_TXT" && ok "arm_boost=1 set" || warn "arm_boost=1 missing (small perf win)"

hdr "UART (GPS)"
if [ -e /dev/serial0 ]; then
    REAL="$(readlink -f /dev/serial0)"
    echo "  /dev/serial0 → $REAL"
    if [ "$REAL" = "/dev/ttyAMA0" ]; then
        ok "PL011 UART (high quality) is on /dev/serial0"
    else
        warn "mini-UART on /dev/serial0 — reboot to apply disable-bt"
    fi
else
    fail "/dev/serial0 missing — UART not enabled / no reboot since enabling"
fi

# Serial console must be off (otherwise getty competes with the GPS for the port).
if systemctl is-enabled --quiet serial-getty@ttyAMA0.service 2>/dev/null; then
    fail "serial-getty@ttyAMA0 is ENABLED — will conflict with GPS. Disable: sudo systemctl disable serial-getty@ttyAMA0"
elif systemctl is-enabled --quiet serial-getty@ttyS0.service 2>/dev/null; then
    warn "serial-getty@ttyS0 is enabled — may conflict if /dev/serial0 → ttyS0"
else
    ok "no serial-getty active on UART"
fi

hdr "I2C bus 1"
if [ -e /dev/i2c-1 ]; then
    ok "/dev/i2c-1 present"
    if command -v i2cdetect >/dev/null 2>&1; then
        i2cdetect -y 1 2>/dev/null | sed 's/^/    /'
        FOUND="$(i2cdetect -y 1 2>/dev/null | awk 'NR>1 {for (i=2; i<=NF; i++) if ($i ~ /^[0-9a-f]{2}$/) print $i}')"
        echo "$FOUND" | grep -qx "40" && ok "  0x40 Motor HAT (PCA9685)" || warn "  0x40 Motor HAT not seen"
        echo "$FOUND" | grep -qx "4a" && ok "  0x4A BNO085 IMU" || warn "  0x4A BNO085 IMU not seen (optional)"
        if echo "$FOUND" | grep -qx "57" || echo "$FOUND" | grep -qx "68"; then
            ok "  PiSugar 3 (0x57 / 0x68)"
        elif echo "$FOUND" | grep -qx "75" || echo "$FOUND" | grep -qx "32"; then
            ok "  PiSugar 2/S (0x75 / 0x32)"
        else
            warn "  no PiSugar detected"
        fi
    fi
else
    fail "/dev/i2c-1 missing — enable I2C and reboot"
fi

hdr "pigpiod (servo daemon)"
if systemctl is-active --quiet pigpiod; then
    ok "pigpiod active"
    if pigs t >/dev/null 2>&1; then
        ok "pigpio client→daemon roundtrip OK"
    else
        warn "pigs client can't reach pigpiod (sudo systemctl restart pigpiod?)"
    fi
else
    fail "pigpiod NOT active — sudo systemctl enable --now pigpiod"
fi

hdr "PiSugar power manager"
if [ -S /tmp/pisugar-server.sock ]; then
    if BATT=$(echo "get battery" | timeout 2 nc -U /tmp/pisugar-server.sock 2>/dev/null); then
        ok "pisugar-server responds: $BATT"
    else
        warn "pisugar-server socket present but unresponsive"
    fi
else
    warn "pisugar-server socket missing — battery monitoring disabled"
fi

hdr "Python venv + project modules"
VENV="${PROJECT_DIR}/.venv"
if [ -d "$VENV" ]; then
    ok "venv: $VENV"
    if [ -x "$VENV/bin/python" ]; then
        if "$VENV/bin/python" -c "
import lawnbot.config, lawnbot.drive.kinematics, lawnbot.nav.planner, lawnbot.nav.controller
import lawnbot.estimator, lawnbot.safety.monitor, lawnbot.ui.server
import lawnbot.drive.pca9685, lawnbot.drive.motor_hat, lawnbot.drive.servo
import lawnbot.gnss.lc29h, lawnbot.gnss.ntrip
import lawnbot.sensors.imu, lawnbot.sensors.odometry
" 2>/tmp/import-err.log; then
            ok "all critical modules import cleanly"
        else
            fail "module import failed:"
            sed 's/^/    /' /tmp/import-err.log
        fi
    else
        fail "venv exists but python missing — re-run setup_pi.sh"
    fi
else
    fail "venv missing — run: sudo bash setup_pi.sh"
fi

hdr "systemd unit"
if [ -f /etc/systemd/system/lawnbot.service ]; then
    ok "lawnbot.service installed"
    if systemctl is-enabled --quiet lawnbot 2>/dev/null; then
        ok "lawnbot.service ENABLED (autostarts on boot)"
        if systemctl is-active --quiet lawnbot; then
            ok "lawnbot.service currently RUNNING"
            UI_PORT="$(grep -E '^[[:space:]]*port:' "$PROJECT_DIR/config.yaml" 2>/dev/null | head -1 | awk '{print $2}')"
            UI_PORT="${UI_PORT:-8080}"
            IP="$(hostname -I | awk '{print $1}')"
            echo "    → UI at: http://${IP:-<pi-ip>}:${UI_PORT}"
        else
            warn "lawnbot.service enabled but not running — journalctl -u lawnbot -n 30"
        fi
    else
        warn "lawnbot.service NOT enabled — enable after Phase 1+2 bring-up"
    fi
else
    warn "lawnbot.service not installed — run setup_pi.sh"
fi

hdr "Disk + logs"
if mountpoint -q /var/log/lawnbot; then
    ok "/var/log/lawnbot mounted on tmpfs"
else
    warn "/var/log/lawnbot not on tmpfs — SD-card writes for every log line"
fi
DF_ROOT="$(df -h / | awk 'NR==2 {print $5 " used (" $3 "/" $2 ")"}')"
echo "  rootfs: $DF_ROOT"

hdr "CPU governor"
if [ -r /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]; then
    GOV="$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)"
    if [ "$GOV" = "performance" ]; then
        ok "scaling_governor = performance"
    else
        warn "scaling_governor = $GOV (expected: performance)"
    fi
fi

echo
echo "============================================================"
echo -e "  ${C_GREEN}${PASS} passed${C_RESET}, ${C_YELLOW}${WARN} warnings${C_RESET}, ${C_RED}${FAIL} failed${C_RESET}"
echo "============================================================"

if [ "$FAIL" -gt 0 ]; then exit 2; fi
if [ "$WARN" -gt 0 ]; then exit 1; fi
exit 0
