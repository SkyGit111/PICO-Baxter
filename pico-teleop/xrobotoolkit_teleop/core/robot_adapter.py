"""Hardware execution interface for teleoperation.

A single ``RobotAdapter`` ABC covers every actuator subsystem (arms, head servo,
base, height, cameras). Subsystems a particular robot doesn't have just inherit
the no-op default. Per-robot files populate ``arm_mappers`` / ``head_mapper``
in ``__init__``; the concrete ``update_arms`` / ``update_head`` methods here
walk those mappers and dispatch to the ``command_*`` hooks.

Each subsystem runs in its own daemon thread at its own rate (``arm_rate_hz``,
``head_rate_hz``). The only state shared between threads is a single XR-state
snapshot, written by the engine and read by each subsystem under one lock.
Per-subsystem mapper state, hardware handles, and command buffers are touched
by exactly one thread, so no further locking is needed.
"""

from __future__ import annotations

import threading
import time
from abc import ABC, abstractmethod

from xrobotoolkit_teleop.core.camera import CameraStream
from xrobotoolkit_teleop.core.mapping import ArmPoseMapper, HeadAngleMapper
from xrobotoolkit_teleop.core.types import (
    ArmName,
    BaseTarget,
    HeadAngles,
    HeightTarget,
    LiftVelocity,
    Pose,
    RobotState,
    XRState,
)


class RobotAdapter(ABC):
    """Standard hardware interface consumed by ``TeleopEngine``."""

    def __init__(self) -> None:
        # Populated by subclasses (typically in __init__) before connect().
        self.arm_mappers: dict[ArmName, ArmPoseMapper] = {}
        self.head_mapper: HeadAngleMapper | None = None
        self.head_toggle_button: str = "X"

        # Per-subsystem control rates (override per robot via config).
        self.arm_rate_hz: float = 50.0
        self.head_rate_hz: float = 25.0
        self.base_rate_hz: float = 20.0
        self.lift_rate_hz: float = 20.0

        self._head_enabled: bool = False
        self._head_prev_toggle: bool = False
        self._head_neutral: Pose | None = None

        # Mobile base + lift (off by default; per-robot configs flip these on).
        self.base_enabled: bool = False
        self.lift_enabled: bool = False
        self.base_max_linear_mps: float = 0.12
        self.base_max_angular_radps: float = 0.35
        self.base_deadzone: float = 0.15
        # Lift: right thumbstick Y → ``command_lift_velocity`` (deadzone = stop; else constant ±pct).
        self.lift_joystick_deadzone: float = 0.15
        self.lift_joystick_invert: bool = False
        self.lift_speed_pct: int = 50

        # Threading: one shared XR snapshot, one lock, one stop event.
        self._xr_lock = threading.Lock()
        self._latest_xr: XRState | None = None
        self._stop_event = threading.Event()
        self._threads: list[threading.Thread] = []

    # ------------------------------------------------------------------ lifecycle

    @abstractmethod
    def connect(self) -> None:
        """Open hardware connections and initialize command state."""

    @abstractmethod
    def shutdown(self) -> None:
        """Stop command streaming and release hardware resources."""

    @abstractmethod
    def read_state(self) -> RobotState:
        """Read measured robot state."""

    # ------------------------------------------------------------------ arms

    @abstractmethod
    def get_end_effector_pose(self, arm: ArmName) -> Pose:
        """Return the current measured end-effector pose for one arm."""

    @abstractmethod
    def command_end_effector_pose(self, arm: ArmName, pose: Pose) -> None:
        """Command one arm to a Cartesian end-effector pose (runs IK internally)."""

    @abstractmethod
    def command_gripper(self, arm: ArmName, value: float) -> None:
        """Command one gripper with a normalized value in ``[0, 1]``."""

    # ------------------------------------------------------------------ head servo

    def command_head_servo(self, target: HeadAngles) -> None:
        """Send pan/tilt angles to head servos. No-op for robots without a head."""

    # ------------------------------------------------------------------ base / height (future)

    def command_base(self, target: BaseTarget) -> None:
        """Command the mobile base (body-frame velocities). No-op by default."""

    def command_height(self, target: HeightTarget) -> None:
        """Command the lift/torso height. No-op by default."""

    def command_lift_velocity(self, target: LiftVelocity) -> None:
        """Open-loop lift speed percent ``[-100, 100]`` (positive = up). No-op by default."""

    # ------------------------------------------------------------------ cameras

    def cameras(self) -> dict[str, CameraStream]:
        """Return camera streams keyed by name. Empty by default."""
        return {}

    # ------------------------------------------------------------------ XR snapshot

    def push_xr_state(self, state: XRState) -> None:
        """Producer side: engine calls this each XR poll."""
        with self._xr_lock:
            self._latest_xr = state

    def _read_xr(self) -> XRState | None:
        """Consumer side: subsystem threads read the latest snapshot."""
        with self._xr_lock:
            return self._latest_xr

    # ------------------------------------------------------------------ subsystem step
    # Walk mappers and call command_* hooks. Each step is called by exactly one
    # subsystem thread, so per-subsystem state is single-threaded.

    def update_arms(self, xr_state: XRState) -> None:
        for arm, mapper in self.arm_mappers.items():
            ctrl = xr_state.left_controller if mapper.controller == "left_controller" else xr_state.right_controller
            self.command_gripper(arm, ctrl.trigger)
            current = self.get_end_effector_pose(arm)
            target = mapper.update(ctrl, current)
            if target is not None:
                self.command_end_effector_pose(arm, target)

    def update_head(self, xr_state: XRState) -> None:
        if self.head_mapper is None:
            return

        toggle = bool(xr_state.buttons.get(self.head_toggle_button, False))
        if toggle and not self._head_prev_toggle:
            self._head_enabled = not self._head_enabled
            state = "enabled" if self._head_enabled else "disabled"
            print(f"Head servo following {state}.")
            if self._head_enabled:
                self._head_neutral = xr_state.headset
                self.command_head_servo(self.head_mapper.center())
        self._head_prev_toggle = toggle

        if not self._head_enabled or xr_state.headset is None or self._head_neutral is None:
            return

        target = self.head_mapper.map_pose(self._head_neutral, xr_state.headset)
        self.command_head_servo(target)

    # ------------------------------------------------------------------ base (chassis)

    def update_base(self, xr_state: XRState) -> None:
        """Map left-controller joystick to a body-frame velocity command.

        Convention: joystick ``y`` → ``linear.x`` (forward/back); ``x`` →
        ``angular.z`` (yaw). Yaw is negated so stick left matches turn-left for
        this stack’s ``cmd_vel`` sign convention.
        """
        jx, jy = xr_state.left_controller.joystick
        dz = max(0.0, min(1.0, float(self.base_deadzone)))
        scale = max(1e-6, 1.0 - dz)

        def axis_to_unit(v: float) -> float:
            a = float(v)
            if abs(a) < dz:
                return 0.0
            s = 1.0 if a > 0.0 else -1.0
            return s * (abs(a) - dz) / scale

        # Per-axis linear response; remapped so full stick → max speed after deadzone.
        vx = axis_to_unit(jy) * self.base_max_linear_mps
        vyaw = -axis_to_unit(jx) * self.base_max_angular_radps

        self.command_base(BaseTarget(vx=vx, vyaw=vyaw))

    # ------------------------------------------------------------------ lift

    def _lift_speed_from_right_joystick_y(self, xr_state: XRState) -> int:
        """Right thumbstick Y → signed lift speed percent (``-100..100``).

        Inside the joystick deadzone: ``0`` (stop). Outside: constant magnitude
        ``lift_speed_pct`` in the stick direction (no stick-deflection ramp).
        """
        _, jy = xr_state.right_controller.joystick
        raw = float(jy)
        if self.lift_joystick_invert:
            raw = -raw
        dz = max(0.0, min(1.0, float(self.lift_joystick_deadzone)))
        cap = max(0, min(100, int(self.lift_speed_pct)))
        if abs(raw) < dz:
            return 0
        sign = 1 if raw > 0.0 else -1
        return sign * cap

    def update_lift(self, xr_state: XRState) -> None:
        """Right stick Y → ``command_lift_velocity`` each tick (inside dz = stop; else ±cap)."""
        speed_pct = self._lift_speed_from_right_joystick_y(xr_state)
        self.command_lift_velocity(LiftVelocity(speed_pct=int(speed_pct)))

    # ------------------------------------------------------------------ subsystem threads

    def start_subsystem_threads(self) -> None:
        """Spawn one daemon thread per active subsystem at its configured rate."""
        self._stop_event.clear()
        if self.arm_mappers:
            self._threads.append(threading.Thread(target=self._arm_loop, name="arm-loop", daemon=True))
        if self.head_mapper is not None:
            self._threads.append(threading.Thread(target=self._head_loop, name="head-loop", daemon=True))
        if self.base_enabled or self.lift_enabled:
            self._threads.append(
                threading.Thread(target=self._base_lift_loop, name="base-lift-loop", daemon=True)
            )
        for t in self._threads:
            t.start()

    def stop_subsystem_threads(self, join_timeout_s: float = 1.0) -> None:
        """Signal all subsystem threads to exit and join them."""
        self._stop_event.set()
        for t in self._threads:
            t.join(timeout=join_timeout_s)
        self._threads.clear()

    def _arm_loop(self) -> None:
        period = 1.0 / self.arm_rate_hz
        while not self._stop_event.is_set():
            t0 = time.time()
            xr = self._read_xr()
            if xr is not None:
                self.read_state()
                self.update_arms(xr)
            sleep = period - (time.time() - t0)
            if sleep > 0.0:
                time.sleep(sleep)

    def _head_loop(self) -> None:
        period = 1.0 / self.head_rate_hz
        while not self._stop_event.is_set():
            t0 = time.time()
            xr = self._read_xr()
            if xr is not None:
                self.update_head(xr)
            sleep = period - (time.time() - t0)
            if sleep > 0.0:
                time.sleep(sleep)

    def _base_lift_loop(self) -> None:
        rate = min(self.base_rate_hz, self.lift_rate_hz)
        period = 1.0 / rate
        while not self._stop_event.is_set():
            t0 = time.time()
            xr = self._read_xr()
            if xr is not None:
                if self.base_enabled:
                    self.update_base(xr)
                if self.lift_enabled:
                    self.update_lift(xr)
            sleep = period - (time.time() - t0)
            if sleep > 0.0:
                time.sleep(sleep)
