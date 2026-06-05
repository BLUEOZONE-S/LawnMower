"""Telemetry logger — JSONL on tmpfs, rotating size.

JSONL because the UI / post-run analysis wants structured records, and
because grep + jq on the Pi is good enough. CSV path is provided for the
narrow case where you want a spreadsheet.

Logs go under /var/log/lawnbot by default (mount that on tmpfs in fstab).
"""
from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path


class JsonlLogger:
    def __init__(self, path: str | Path, max_bytes: int = 8 * 1024 * 1024, keep: int = 3):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.max_bytes = max_bytes
        self.keep = keep
        self._lock = threading.Lock()
        self._fh = open(self.path, "a", encoding="utf-8")

    def log(self, record: dict) -> None:
        record.setdefault("t", time.time())
        line = json.dumps(record, separators=(",", ":")) + "\n"
        with self._lock:
            self._fh.write(line)
            self._fh.flush()
            if self._fh.tell() > self.max_bytes:
                self._rotate_locked()

    def close(self) -> None:
        with self._lock:
            try:
                self._fh.close()
            except Exception:
                pass

    def _rotate_locked(self) -> None:
        self._fh.close()
        for i in range(self.keep, 0, -1):
            src = self.path.with_suffix(self.path.suffix + f".{i}")
            if i == self.keep and src.exists():
                src.unlink(missing_ok=True)
            else:
                older = self.path.with_suffix(self.path.suffix + f".{i + 1}")
                if src.exists():
                    src.rename(older)
        rotated = self.path.with_suffix(self.path.suffix + ".1")
        if self.path.exists():
            os.replace(self.path, rotated)
        self._fh = open(self.path, "a", encoding="utf-8")
