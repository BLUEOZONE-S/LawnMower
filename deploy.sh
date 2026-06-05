#!/usr/bin/env bash
# Sync this repo to the Pi and restart the service.
# Usage: PI_HOST=lawnbot.local ./deploy.sh
set -euo pipefail

PI_HOST="${PI_HOST:-lawnbot.local}"
PI_USER="${PI_USER:-pi}"
PI_PATH="${PI_PATH:-/home/${PI_USER}/lawnbot}"

rsync -avz --delete \
  --exclude='.venv' --exclude='__pycache__' --exclude='.git' \
  --exclude='logs' --exclude='telemetry/*.csv' --exclude='telemetry/*.jsonl' \
  ./ "${PI_USER}@${PI_HOST}:${PI_PATH}/"

ssh "${PI_USER}@${PI_HOST}" "cd ${PI_PATH} && \
  (test -d .venv || python3 -m venv .venv) && \
  .venv/bin/pip install -q -r requirements.txt && \
  sudo systemctl restart lawnbot || true && \
  sudo journalctl -u lawnbot -n 30 --no-pager"
