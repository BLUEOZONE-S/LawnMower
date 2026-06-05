"""Online PID auto-tuner (Twiddle / coordinate-descent).

Drives the controller's gains (Kp, Ki, Kd, lookahead) to minimize a windowed
cost function over (heading_err, cross_track) telemetry. Runs as a background
thread; samples are fed by :meth:`AutoTuner.sample` from the control loop.

Algorithm:
  1. Measure baseline cost over EVAL_S seconds.
  2. For each gain g in round-robin:
       a. Try g + dp[g]. Evaluate cost over EVAL_S.
       b. If improved → keep new value, dp[g] *= 1.1.
       c. Else try g - dp[g]. Evaluate.
          If improved → keep, dp[g] *= 1.1.
          Else → revert, dp[g] *= 0.9.
  3. Repeat. Converges when dp[g] shrinks below a tolerance.

Twiddle is gradient-free and noise-robust — it just needs a stable, repeatable
cost. We use a stationary window of (heading_err² + W·cross_track²) collected
DURING the mission, so the controller is genuinely being exercised over the
planned path while we tune.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass


log = logging.getLogger("lawnbot.autotune")


@dataclass
class AutoTuneStatus:
    running: bool = False
    iteration: int = 0
    best_cost: float = float("inf")
    last_cost: float = float("inf")
    gain_under_test: str = ""
    direction: str = ""
    gains: dict | None = None
    dp: dict | None = None


class AutoTuner:
    """Coordinate-descent / Twiddle tuner for the pure-pursuit + PID controller."""

    # Gains we tune. lookahead_m is part of pure-pursuit, not the PID, but it
    # has the biggest effect on path-following quality so it belongs here too.
    GAINS = ("kp", "ki", "kd", "lookahead_m")

    INITIAL_DP = {"kp": 0.4, "ki": 0.05, "kd": 0.05, "lookahead_m": 0.15}
    MIN_GAIN = {"kp": 0.5, "ki": 0.0, "kd": 0.0, "lookahead_m": 0.30}
    MAX_GAIN = {"kp": 8.0, "ki": 2.0, "kd": 1.5, "lookahead_m": 3.0}

    EVAL_SECONDS = 4.0       # how long to collect samples per candidate
    SETTLE_SECONDS = 0.5     # quiet period after a gain change before sampling
    CROSS_WEIGHT = 0.5       # cost = mean(heading_err² + W·cross_track²)
    MIN_SAMPLES = 20         # avoid evaluating on an empty buffer
    CONVERGE_DP_SUM = 0.05   # if sum(dp.values()) < this, declare converged
    MAX_ITERS = 200          # hard ceiling

    def __init__(self, controller, control_cfg):
        self.controller = controller
        self.control_cfg = control_cfg
        self._lock = threading.Lock()
        self._buf: list[tuple[float, float]] = []
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.status = AutoTuneStatus()

    # ---- public API -----------------------------------------------------

    def start(self) -> bool:
        with self._lock:
            if self.status.running:
                return False
            self.status = AutoTuneStatus(running=True)
            self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="autotune")
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        with self._lock:
            self.status.running = False

    def sample(self, heading_err: float, cross_track: float) -> None:
        """Called from the control loop every tick when running."""
        if not self.status.running:
            return
        with self._lock:
            self._buf.append((float(heading_err), float(cross_track)))

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "running": self.status.running,
                "iteration": self.status.iteration,
                "best_cost": (None if math.isinf(self.status.best_cost)
                              else round(self.status.best_cost, 6)),
                "last_cost": (None if math.isinf(self.status.last_cost)
                              else round(self.status.last_cost, 6)),
                "gain_under_test": self.status.gain_under_test,
                "direction": self.status.direction,
                "gains": self.status.gains,
                "dp": self.status.dp,
            }

    # ---- internals ------------------------------------------------------

    def _read_gains(self) -> dict:
        return {
            "kp": self.controller.cfg.pid.kp,
            "ki": self.controller.cfg.pid.ki,
            "kd": self.controller.cfg.pid.kd,
            "lookahead_m": self.controller.cfg.pure_pursuit.lookahead_m,
        }

    def _apply_gain(self, name: str, value: float) -> None:
        value = max(self.MIN_GAIN[name], min(self.MAX_GAIN[name], float(value)))
        self.controller.update_gains(**{name: value})

    def _evaluate(self) -> float:
        """Settle, then accumulate (heading_err, cross_track) for EVAL_SECONDS."""
        # Settle so the prior gain's effect has decayed before we measure.
        if self._stop.wait(self.SETTLE_SECONDS):
            return float("inf")
        with self._lock:
            self._buf.clear()
        # Sample for EVAL_SECONDS of wall-clock. The control loop is pushing
        # into self._buf concurrently while we wait.
        end = time.monotonic() + self.EVAL_SECONDS
        while time.monotonic() < end:
            if self._stop.wait(0.05):
                break
        with self._lock:
            samples = list(self._buf)
            self._buf.clear()
        if len(samples) < self.MIN_SAMPLES:
            return float("inf")
        W = self.CROSS_WEIGHT
        cost = sum(h * h + W * c * c for h, c in samples) / len(samples)
        return cost

    def _loop(self) -> None:
        log.info("autotune: starting from gains=%s", self._read_gains())
        dp = dict(self.INITIAL_DP)
        best_gains = self._read_gains()
        self._set_status(gain_under_test="(baseline)", direction="", gains=best_gains, dp=dp)
        best_cost = self._evaluate()
        if math.isinf(best_cost):
            log.warning("autotune: baseline cost was infinite — aborting")
            self._set_status(running=False)
            return
        self._set_status(best_cost=best_cost, last_cost=best_cost)
        log.info("autotune: baseline cost=%.4f", best_cost)

        idx = 0
        iteration = 0
        while not self._stop.is_set() and iteration < self.MAX_ITERS:
            iteration += 1
            name = self.GAINS[idx]
            step = dp[name]
            base = best_gains[name]

            # ---- try base + step ----
            new = base + step
            self._apply_gain(name, new)
            self._set_status(iteration=iteration, gain_under_test=name,
                             direction=f"+{step:.3g}", gains=self._read_gains(), dp=dict(dp))
            cost = self._evaluate()
            if self._stop.is_set(): break
            self._set_status(last_cost=cost)
            log.info("autotune iter%d %s +%.3g → cost=%.4f (best=%.4f)",
                     iteration, name, step, cost, best_cost)

            if cost < best_cost:
                best_cost = cost
                best_gains = self._read_gains()
                dp[name] *= 1.1
            else:
                # ---- try base - step ----
                new = base - step
                self._apply_gain(name, new)
                self._set_status(direction=f"-{step:.3g}", gains=self._read_gains(), dp=dict(dp))
                cost = self._evaluate()
                if self._stop.is_set(): break
                self._set_status(last_cost=cost)
                log.info("autotune iter%d %s -%.3g → cost=%.4f (best=%.4f)",
                         iteration, name, step, cost, best_cost)
                if cost < best_cost:
                    best_cost = cost
                    best_gains = self._read_gains()
                    dp[name] *= 1.1
                else:
                    # Revert and shrink
                    self._apply_gain(name, base)
                    dp[name] *= 0.9

            self._set_status(best_cost=best_cost, gains=best_gains, dp=dict(dp))

            if sum(dp.values()) < self.CONVERGE_DP_SUM:
                log.info("autotune: converged at iter %d, gains=%s", iteration, best_gains)
                break
            idx = (idx + 1) % len(self.GAINS)

        # Ensure best gains are applied before exiting.
        for n, v in best_gains.items():
            self._apply_gain(n, v)
        self._set_status(running=False, gain_under_test="(done)", direction="",
                        gains=best_gains, dp=dict(dp), best_cost=best_cost)
        log.info("autotune: done. best_cost=%.4f gains=%s", best_cost, best_gains)

    def _set_status(self, **kw) -> None:
        with self._lock:
            for k, v in kw.items():
                setattr(self.status, k, v)
