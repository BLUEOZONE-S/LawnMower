#!/usr/bin/env bash
#
# LawnBot — one-shot Pi Zero 2 W setup script.
#
# Run ON THE PI after flashing Raspberry Pi OS Lite 64-bit and copying this
# repo to /home/pi/lawnbot:
#
#     cd /home/pi/lawnbot
#     sudo bash setup_pi.sh
#
# What it does (each step is idempotent — safe to re-run):
#   1.  Confirm OS / hardware sanity
#   2.  apt update + install system packages
#   3.  Enable I2C, UART hardware, disable serial console
#   4.  Edit /boot/firmware/config.txt (enable_uart, disable-bt, arm_boost)
#   5.  Enable + start pigpiod
#   6.  Install PiSugar power-manager (interactive — asks for model)
#   7.  Disable unused services (bluetooth, avahi, triggerhappy, ModemManager, cups)
#   8.  Set CPU governor to "performance"
#   9.  Mount /var/log/lawnbot on tmpfs (so logs survive power-cut without
#       chewing SD-card writes)
#   10. Create the Python venv and install requirements.txt
#   11. Install /etc/systemd/system/lawnbot.service (NOT enabled — you do
#       that after Phases 1+2 in §6 of the README pass)
#   12. Verify everything and print a status report
#   13. Note whether a reboot is needed (UART / Bluetooth changes need one)
#
# Re-running this script after editing requirements.txt re-installs the
# Python deps. Re-running after a config drift re-applies system settings.

set -u  # bare -e is too strict — we want soft-fail with reporting on optional steps
trap 'echo; echo "❌ aborted at line $LINENO"; exit 1' ERR

# ---- shared helpers -------------------------------------------------------

C_GREEN='\033[1;32m'; C_YELLOW='\033[1;33m'; C_RED='\033[1;31m'; C_BLUE='\033[1;34m'; C_RESET='\033[0m'

PASS=()
WARN=()
FAIL=()
REBOOT_NEEDED=0

section()  { echo -e "\n${C_BLUE}=== $* ===${C_RESET}"; }
ok()       { echo -e "${C_GREEN}✓${C_RESET} $*"; PASS+=("$*"); }
warn()     { echo -e "${C_YELLOW}!${C_RESET} $*"; WARN+=("$*"); }
fail()     { echo -e "${C_RED}✗${C_RESET} $*"; FAIL+=("$*"); }
note()     { echo -e "${C_BLUE}·${C_RESET} $*"; }

ensure_root() {
    if [ "$(id -u)" -ne 0 ]; then
        echo "This script needs root for apt + systemctl + /boot/firmware edits."
        echo "Re-run with: sudo bash $0"
        exit 1
    fi
}

# Replace or append a key=value pair in a config file, idempotently.
# Args: file key=value
set_kv_in_file() {
    local file="$1" kv="$2"
    local key="${kv%%=*}"
    if [ ! -f "$file" ]; then
        touch "$file"
    fi
    if grep -qE "^[[:space:]]*${key}=" "$file"; then
        sed -i "s|^[[:space:]]*${key}=.*|${kv}|" "$file"
    else
        printf "%s\n" "$kv" >> "$file"
    fi
}

# Add an `dtoverlay=...` line idempotently. Args: overlay-spec
set_overlay() {
    local spec="$1"
    local file="/boot/firmware/config.txt"
    if [ ! -f "$file" ]; then file="/boot/config.txt"; fi   # pre-Bookworm fallback
    if ! grep -qE "^[[:space:]]*dtoverlay=${spec//\//\\/}([[:space:]]|$)" "$file"; then
        echo "dtoverlay=${spec}" >> "$file"
        return 0
    fi
    return 1
}

# ---- 1. Sanity check ------------------------------------------------------

ensure_root

section "1. System sanity"

if grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null; then
    MODEL="$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo unknown)"
    ok  "running on: ${MODEL}"
    if ! echo "$MODEL" | grep -qi "Zero 2"; then
        warn "this script targets Pi Zero 2 W — yours is different but should still work"
    fi
else
    fail "this doesn't look like a Raspberry Pi"
    exit 1
fi

if [ -f /etc/os-release ]; then
    . /etc/os-release
    ok  "OS: ${PRETTY_NAME}"
    if [ "${VERSION_CODENAME:-}" != "bookworm" ]; then
        warn "the docs assume Bookworm; you're on ${VERSION_CODENAME:-unknown} — paths might differ"
    fi
fi

if [ "$(uname -m)" = "aarch64" ]; then
    ok  "kernel arch: aarch64 (64-bit) ✓"
else
    warn "kernel arch is $(uname -m) — the docs assume 64-bit (aarch64)"
fi

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
note "project dir: ${PROJECT_DIR}"
PI_USER="${SUDO_USER:-pi}"
note "running on behalf of user: ${PI_USER}"

# ---- 2. apt + system packages --------------------------------------------

section "2. apt install (system packages)"

export DEBIAN_FRONTEND=noninteractive

apt update -qq
note "apt update done"

PKGS=(
    git
    python3 python3-pip python3-venv python3-smbus
    i2c-tools
    pigpio
    rsync
    htop
    netcat-openbsd
    cpufrequtils
)

if apt install -y "${PKGS[@]}" >/tmp/apt-install.log 2>&1; then
    ok "installed: ${PKGS[*]}"
else
    fail "apt install failed — see /tmp/apt-install.log"
    tail -n 20 /tmp/apt-install.log
fi

# ---- 3. raspi-config: I2C, UART, no serial console ------------------------

section "3. Hardware interfaces"

if command -v raspi-config >/dev/null 2>&1; then
    raspi-config nonint do_i2c 0       && ok "I2C enabled"            || warn "could not enable I2C"
    raspi-config nonint do_serial_hw 0 && ok "serial hardware enabled" || warn "could not enable serial hardware"
    raspi-config nonint do_serial_cons 1 && ok "serial console disabled" || warn "could not disable serial console"
else
    warn "raspi-config not present — set I2C / UART / serial console manually"
fi

# Verify the kernel modules are loaded
if lsmod | grep -q i2c_bcm2835 || lsmod | grep -q i2c_bcm2708; then
    ok "i2c-bcm kernel module loaded"
else
    warn "i2c-bcm module not loaded — may need a reboot"
fi

# ---- 4. /boot/firmware/config.txt edits ----------------------------------

section "4. Boot config (/boot/firmware/config.txt)"

CONFIG_TXT="/boot/firmware/config.txt"
if [ ! -f "$CONFIG_TXT" ]; then
    CONFIG_TXT="/boot/config.txt"
fi
note "editing: $CONFIG_TXT"

# Back up once
[ ! -f "${CONFIG_TXT}.lawnbot.bak" ] && cp "$CONFIG_TXT" "${CONFIG_TXT}.lawnbot.bak"

CHANGED=0
for kv in "enable_uart=1" "arm_boost=1"; do
    if ! grep -qE "^[[:space:]]*${kv}([[:space:]]|$)" "$CONFIG_TXT"; then
        set_kv_in_file "$CONFIG_TXT" "$kv"
        CHANGED=1
        ok "added: $kv"
    else
        ok "already set: $kv"
    fi
done

if set_overlay "disable-bt"; then
    ok "added: dtoverlay=disable-bt"
    CHANGED=1
else
    ok "already set: dtoverlay=disable-bt"
fi

# Disable Bluetooth modem service (needed once dtoverlay=disable-bt is set)
systemctl disable --now hciuart 2>/dev/null && ok "hciuart disabled" || warn "hciuart already off"

if [ $CHANGED -eq 1 ]; then
    REBOOT_NEEDED=1
    note "boot config changed → reboot required at the end"
fi

# ---- 5. pigpiod ----------------------------------------------------------

section "5. pigpiod (servo daemon)"

systemctl enable --now pigpiod >/dev/null 2>&1 && ok "pigpiod enabled + running" \
    || fail "pigpiod failed to start"

if systemctl is-active --quiet pigpiod; then
    ok "pigpiod is active"
else
    fail "pigpiod is NOT active — sudo systemctl status pigpiod"
fi

# ---- 6. PiSugar power manager --------------------------------------------

section "6. PiSugar power manager"

if [ -S /tmp/pisugar-server.sock ]; then
    ok "pisugar-server socket exists — already installed"
else
    if [ "${LAWNBOT_SKIP_PISUGAR:-0}" = "1" ]; then
        warn "skipping PiSugar install (LAWNBOT_SKIP_PISUGAR=1)"
    else
        note "downloading PiSugar installer (https://cdn.pisugar.com/release/pisugar-power-manager.sh)"
        if curl -fsSL -o /tmp/pisugar-installer.sh https://cdn.pisugar.com/release/pisugar-power-manager.sh; then
            echo
            echo "  ➜  The PiSugar installer is INTERACTIVE — it will ask you to"
            echo "     pick your PiSugar model. Pick yours (2 / 3 / S) when prompted."
            echo "     (Skip with LAWNBOT_SKIP_PISUGAR=1 if you don't have a PiSugar yet.)"
            echo
            bash /tmp/pisugar-installer.sh -c release \
                && ok "pisugar-power-manager installed" \
                || warn "PiSugar installer didn't complete — re-run manually if needed"
        else
            warn "couldn't fetch PiSugar installer (no internet?) — skipping"
        fi
    fi
fi

# ---- 7. Disable unused services ------------------------------------------

section "7. Disable unused services (reclaim RAM/CPU)"

for svc in bluetooth avahi-daemon triggerhappy ModemManager cups cups-browsed; do
    if systemctl list-unit-files 2>/dev/null | grep -q "^${svc}.service"; then
        systemctl disable --now "$svc" >/dev/null 2>&1 \
            && ok "disabled: $svc" \
            || warn "could not disable $svc"
    else
        note "$svc not installed (already absent)"
    fi
done

note "(avahi-daemon disabled means 'lawnbot.local' mDNS won't work — use the Pi's IP)"
note "(bluetooth disabled means no Bluetooth gamepad — USB / on-screen joystick still work)"

# ---- 8. CPU governor -----------------------------------------------------

section "8. CPU governor → performance"

if [ -d /sys/devices/system/cpu/cpu0/cpufreq ]; then
    echo 'GOVERNOR="performance"' > /etc/default/cpufrequtils
    systemctl restart cpufrequtils 2>/dev/null || true
    echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor >/dev/null 2>&1 || true
    CUR_GOV="$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo unknown)"
    if [ "$CUR_GOV" = "performance" ]; then
        ok "cpufreq governor: performance"
    else
        warn "cpufreq governor is '$CUR_GOV' — expected 'performance'"
    fi
else
    warn "cpufreq not exposed — skipping governor"
fi

# ---- 9. tmpfs for logs ---------------------------------------------------

section "9. /var/log/lawnbot on tmpfs"

mkdir -p /var/log/lawnbot
chown "${PI_USER}:${PI_USER}" /var/log/lawnbot

if ! grep -q "/var/log/lawnbot" /etc/fstab; then
    echo "tmpfs /var/log/lawnbot tmpfs defaults,size=64M,noatime,uid=$(id -u "$PI_USER"),gid=$(id -g "$PI_USER") 0 0" >> /etc/fstab
    ok "/var/log/lawnbot added to /etc/fstab"
fi

if mountpoint -q /var/log/lawnbot; then
    ok "/var/log/lawnbot already mounted (tmpfs)"
else
    mount /var/log/lawnbot 2>/dev/null && ok "/var/log/lawnbot mounted (tmpfs)" \
        || warn "couldn't mount /var/log/lawnbot — will mount on reboot"
fi

# ---- 10. Python venv + deps ---------------------------------------------

section "10. Python venv + requirements.txt"

VENV="${PROJECT_DIR}/.venv"
if [ ! -d "$VENV" ]; then
    sudo -u "$PI_USER" python3 -m venv "$VENV"
    ok "venv created at $VENV"
else
    ok "venv already exists at $VENV"
fi

note "upgrading pip + installing requirements.txt (slow on Pi Zero 2 W — 5–10 min first time)"
if sudo -u "$PI_USER" "$VENV/bin/pip" install --upgrade pip >/tmp/pip-upgrade.log 2>&1; then
    ok "pip upgraded"
else
    warn "pip upgrade had issues — see /tmp/pip-upgrade.log"
fi

if sudo -u "$PI_USER" "$VENV/bin/pip" install -r "$PROJECT_DIR/requirements.txt" >/tmp/pip-install.log 2>&1; then
    ok "Python deps installed"
else
    fail "pip install -r requirements.txt failed — see /tmp/pip-install.log"
    tail -n 20 /tmp/pip-install.log
fi

# ---- 11. systemd unit (installed, NOT enabled) ---------------------------

section "11. systemd unit (installed but disabled)"

UNIT_SRC="${PROJECT_DIR}/systemd/lawnbot.service"
UNIT_DST="/etc/systemd/system/lawnbot.service"

if [ -f "$UNIT_SRC" ]; then
    # Rewrite ExecStart + WorkingDirectory if the project lives somewhere other than /home/pi/lawnbot.
    sed -e "s|/home/pi/lawnbot|${PROJECT_DIR}|g" \
        -e "s|^User=pi|User=${PI_USER}|" \
        "$UNIT_SRC" > "$UNIT_DST"
    chmod 0644 "$UNIT_DST"
    systemctl daemon-reload
    ok "lawnbot.service installed at $UNIT_DST (not yet enabled)"
    note "enable with: sudo systemctl enable --now lawnbot   (AFTER Phase 1+2 bench checks pass)"
else
    fail "missing $UNIT_SRC"
fi

# ---- 12. Verification ----------------------------------------------------

section "12. Verification"

# Re-check config.txt
for key in "enable_uart=1" "arm_boost=1"; do
    if grep -qE "^[[:space:]]*${key}([[:space:]]|$)" "$CONFIG_TXT"; then
        ok "config.txt has $key"
    else
        fail "config.txt MISSING $key"
    fi
done
if grep -qE "^[[:space:]]*dtoverlay=disable-bt([[:space:]]|$)" "$CONFIG_TXT"; then
    ok "config.txt has dtoverlay=disable-bt"
else
    fail "config.txt MISSING dtoverlay=disable-bt"
fi

# Check /dev/serial0 → expected once /boot/firmware/config.txt takes effect (post-reboot)
if [ -e /dev/serial0 ]; then
    REAL="$(readlink -f /dev/serial0)"
    ok "/dev/serial0 → ${REAL}"
    if [ "$REAL" = "/dev/ttyAMA0" ]; then
        ok "PL011 UART is on /dev/serial0 (good — high-quality UART)"
    else
        warn "/dev/serial0 is on $REAL — reboot to apply disable-bt so PL011 takes over"
    fi
else
    warn "/dev/serial0 doesn't exist yet — reboot needed"
fi

# pigpiod
if systemctl is-active --quiet pigpiod; then
    ok "pigpiod active"
else
    fail "pigpiod NOT active"
fi

# I2C presence
if [ -e /dev/i2c-1 ]; then
    ok "/dev/i2c-1 present"
    FOUND="$(i2cdetect -y 1 2>/dev/null | awk 'NR>1 {for (i=2; i<=NF; i++) if ($i ~ /^[0-9a-f]{2}$/) print $i}')"
    if [ -n "$FOUND" ]; then
        ok "I2C devices detected: $(echo $FOUND | tr '\n' ' ')"
        echo "$FOUND" | grep -qx "40" && ok "  · 0x40 Motor HAT (PCA9685) present" || warn "  · 0x40 Motor HAT not detected (mount HAT?)"
        echo "$FOUND" | grep -qx "4a" && ok "  · 0x4A BNO085 IMU present" || note "  · 0x4A BNO085 IMU not detected (optional)"
        if echo "$FOUND" | grep -qx "57" || echo "$FOUND" | grep -qx "68"; then
            ok "  · PiSugar 3 addresses present"
        elif echo "$FOUND" | grep -qx "75" || echo "$FOUND" | grep -qx "32"; then
            ok "  · PiSugar 2/S addresses present"
        else
            note "  · no PiSugar I2C addresses detected (mount PiSugar?)"
        fi
    else
        warn "no I2C devices responded — nothing wired yet, or buses not enabled (try after reboot)"
    fi
else
    warn "/dev/i2c-1 missing — reboot to load i2c-bcm2835"
fi

# pisugar-server
if [ -S /tmp/pisugar-server.sock ]; then
    if BATT=$(echo "get battery" | timeout 2 nc -U /tmp/pisugar-server.sock 2>/dev/null); then
        ok "pisugar-server responds: ${BATT}"
    else
        warn "pisugar-server socket present but unresponsive"
    fi
else
    note "pisugar-server socket not present — install or skip"
fi

# Python imports
section "13. Python import smoke test"

if sudo -u "$PI_USER" "$VENV/bin/python" -c "
import lawnbot.config, lawnbot.drive.kinematics, lawnbot.nav.planner, lawnbot.nav.controller
import lawnbot.estimator, lawnbot.safety.monitor, lawnbot.ui.server
import lawnbot.drive.pca9685, lawnbot.drive.motor_hat, lawnbot.drive.servo
import lawnbot.gnss.lc29h, lawnbot.gnss.ntrip
import lawnbot.sensors.imu, lawnbot.sensors.odometry
print('OK')
" 2>&1; then
    ok "all 14 critical modules import cleanly"
else
    fail "some module imports failed — see output above"
fi

# Pure-math tests (no hardware)
if [ -d "${PROJECT_DIR}/tests" ]; then
    if sudo -u "$PI_USER" "$VENV/bin/python" -m pytest "${PROJECT_DIR}/tests/" -q --no-header >/tmp/pytest.log 2>&1; then
        TESTS_PASSED="$(tail -n 5 /tmp/pytest.log | grep -oE '[0-9]+ passed' | head -1)"
        ok "pytest: ${TESTS_PASSED}"
    else
        warn "pytest had failures — see /tmp/pytest.log"
        tail -n 15 /tmp/pytest.log
    fi
fi

# ---- Final report --------------------------------------------------------

echo
echo "============================================================"
echo "  Setup summary"
echo "============================================================"
echo -e "  ${C_GREEN}${#PASS[@]} passed${C_RESET}, ${C_YELLOW}${#WARN[@]} warnings${C_RESET}, ${C_RED}${#FAIL[@]} failed${C_RESET}"

if [ ${#FAIL[@]} -gt 0 ]; then
    echo
    echo -e "${C_RED}Failures:${C_RESET}"
    for f in "${FAIL[@]}"; do echo "  ✗ $f"; done
fi

if [ ${#WARN[@]} -gt 0 ]; then
    echo
    echo -e "${C_YELLOW}Warnings (likely benign, but read them):${C_RESET}"
    for w in "${WARN[@]}"; do echo "  ! $w"; done
fi

echo
if [ $REBOOT_NEEDED -eq 1 ]; then
    echo -e "${C_YELLOW}A reboot is required to apply UART / Bluetooth changes.${C_RESET}"
    echo "    sudo reboot"
    echo
    echo "After reboot:"
    echo "  · ssh back in"
    echo "  · run:  sudo bash setup_pi.sh   (it's idempotent — re-runs only what's pending)"
    echo "    or:   bash verify_pi.sh       (verify-only, no changes)"
fi

echo
echo "Next bring-up steps (README §6):"
echo "  1. wheels OFF the ground"
echo "  2. .venv/bin/python -m tools.motor_calibrate    # Phase 1"
echo "  3. .venv/bin/python -m tools.gps_monitor        # Phase 2 — needs sky view"
echo "  4. sudo systemctl enable --now lawnbot         # autostart once Phases 1+2 pass"
echo

if [ ${#FAIL[@]} -gt 0 ]; then
    exit 2
fi
exit 0
