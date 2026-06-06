#!/usr/bin/env bash
#
# Configure the pisugar-server daemon for whichever PiSugar model you've
# wired up, start it, and confirm the socket responds.
#
# Why this exists: pisugar-server REFUSES to start with `--model ''` (empty
# string) — its packaged systemd unit ships with that default, so installing
# the .deb leaves the service in a permanent failure loop until someone sets
# the model. The interactive .sh installer prompts for the model, but if you
# get the .deb via apt or the installer fails partway through, you end up
# here.
#
# Usage:
#   sudo bash tools/configure_pisugar.sh                 # interactive picker
#   sudo bash tools/configure_pisugar.sh "PiSugar S"     # quiet, specific model
#
# Accepted model aliases (case-insensitive):
#   "PiSugar S"      → "PiSugar 2 (2-LEDs)"    # slim/portable, 2-LED firmware
#   "PiSugar 2"      → "PiSugar 2 (4-LEDs)"    # original 4-LED
#   "PiSugar 2 Pro"  → "PiSugar 2 Pro"
#   "PiSugar 3"      → "PiSugar 3"
#   or any exact firmware string in /usr/bin/pisugar-server --help

set -u

DEFAULTS_FILE="/etc/default/pisugar-server"
SVC="pisugar-server"

if [ "$(id -u)" -ne 0 ]; then
    echo "Re-run with sudo: sudo bash $0 $*"
    exit 1
fi

if [ ! -f "$DEFAULTS_FILE" ]; then
    echo "$DEFAULTS_FILE not found — install pisugar-server first:"
    echo "    curl -sSL https://cdn.pisugar.com/release/pisugar-power-manager.sh | sudo bash"
    exit 1
fi

# ---- map a friendly alias to the firmware's exact model string ------------
canonical_model() {
    local q="$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]' | tr -d ' ')"
    case "$q" in
        pisugars|s|pisugar2s|pisugars\(2-leds\))      echo "PiSugar 2 (2-LEDs)" ;;
        pisugar2|pisugar2\(4-leds\))                  echo "PiSugar 2 (4-LEDs)" ;;
        pisugar2pro)                                  echo "PiSugar 2 Pro" ;;
        pisugar3)                                     echo "PiSugar 3" ;;
        *) echo "$1" ;;                                # pass through unknown — let the daemon validate
    esac
}

MODEL="${1:-}"
if [ -z "$MODEL" ]; then
    echo
    echo "Which PiSugar model is wired up?"
    echo "  1) PiSugar S        (slim/portable, 2 LEDs)"
    echo "  2) PiSugar 2        (4 LEDs)"
    echo "  3) PiSugar 2 Pro"
    echo "  4) PiSugar 3"
    read -rp "Choice [1-4]: " choice
    case "$choice" in
        1) MODEL="PiSugar S" ;;
        2) MODEL="PiSugar 2" ;;
        3) MODEL="PiSugar 2 Pro" ;;
        4) MODEL="PiSugar 3" ;;
        *) echo "Bad choice"; exit 1 ;;
    esac
fi

CANON="$(canonical_model "$MODEL")"
echo "Configuring pisugar-server with model=\"$CANON\""

# ---- rewrite the OPTS line idempotently -----------------------------------
# Match: --model 'anything'  or  --model "anything"  or  --model anything
if grep -q -- "--model" "$DEFAULTS_FILE"; then
    # Pattern: --model followed by either '...', "..." or a bare token, replace with "..."
    sed -i -E "s|--model[[:space:]]+('[^']*'|\"[^\"]*\"|[^[:space:]]+)|--model \"$CANON\"|" "$DEFAULTS_FILE"
else
    echo "OPTS file has no --model option — leaving it alone. Inspect $DEFAULTS_FILE manually."
    exit 1
fi

echo "--- $DEFAULTS_FILE (after) ---"
grep "^OPTS" "$DEFAULTS_FILE"
echo

# ---- enable + start the daemon -------------------------------------------
systemctl enable "$SVC" >/dev/null 2>&1
systemctl restart "$SVC"
sleep 2

echo "--- service state ---"
systemctl is-active "$SVC" && echo "  ✓ $SVC is active" || { echo "  ✗ $SVC failed to start"; journalctl -u "$SVC" -n 10 --no-pager; exit 2; }

# ---- prove the daemon can answer ----------------------------------------
if [ ! -S /tmp/pisugar-server.sock ]; then
    echo "  ✗ /tmp/pisugar-server.sock missing"
    exit 2
fi

ans_model=$(echo "get model" | timeout 1 nc -U /tmp/pisugar-server.sock 2>&1 | head -1)
ans_batt=$(echo "get battery" | timeout 1 nc -U /tmp/pisugar-server.sock 2>&1 | head -1)
ans_voltage=$(echo "get battery_v" | timeout 1 nc -U /tmp/pisugar-server.sock 2>&1 | head -1)

echo
echo "--- daemon reports ---"
echo "  $ans_model"
echo "  $ans_batt"
echo "  $ans_voltage"

if [[ "$ans_voltage" == *"I2C not connected"* ]] || [[ -z "$ans_batt" ]]; then
    echo
    echo "  ! daemon is up but can't reach the PiSugar over I2C."
    echo "    Check that the PiSugar is physically seated on the GPIO header,"
    echo "    that 'i2cdetect -y 1' shows your expected address(es),"
    echo "    and that the PiSugar battery has charge (USB-C in)."
fi

exit 0
