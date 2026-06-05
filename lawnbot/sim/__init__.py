"""Simulation backend — virtual Ackermann world + drop-in sensor/motor drivers.

Enables running the full LawnBot stack on a dev machine (Windows/macOS/Linux)
without any Pi hardware. The control loop, planner, estimator, UI, teach
flow, teleop, safety monitor — all of it runs unchanged; only the
hardware-touching leaf drivers are swapped for sim equivalents that talk to
the shared SimWorld.

Activate by setting ``sim.enabled: true`` in config (see ``config.sim.yaml``).
"""
