"""Teleoperation orchestration loop.

The engine polls the XR input source at a fixed rate and pushes each snapshot
to the adapter. The adapter's per-subsystem threads (started here) consume
those snapshots at their own rates. All math lives in ``core/mapping.py``;
all hardware specifics live in the adapter.
"""

from __future__ import annotations

import time

from xrobotoolkit_teleop.core.robot_adapter import RobotAdapter
from xrobotoolkit_teleop.core.xr_input import XRInputSource


class TeleopEngine:
    """Drive a ``RobotAdapter`` from an ``XRInputSource``.

    The engine runs only the XR poll loop; per-subsystem control loops run in
    daemon threads owned by the adapter.
    """

    def __init__(
        self,
        xr_input: XRInputSource,
        adapter: RobotAdapter,
        xr_poll_hz: float = 100.0,
    ):
        self.xr_input = xr_input
        self.adapter = adapter
        self.xr_poll_hz = xr_poll_hz
        self.dt = 1.0 / xr_poll_hz

    def run_forever(self) -> None:
        print("Teleoperation running. Press Ctrl+C to exit.")
        self.adapter.start_subsystem_threads()
        try:
            while True:
                t0 = time.time()
                self.adapter.push_xr_state(self.xr_input.poll())
                sleep = self.dt - (time.time() - t0)
                if sleep > 0.0:
                    time.sleep(sleep)
        finally:
            self.adapter.stop_subsystem_threads()
