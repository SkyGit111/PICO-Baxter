# PICO-Baxter

PICO 4 Ultra based teleoperation and visual-feedback system for the
Baxter robot.

## Repository structure

- `bridge/`: Baxter-side control, testing and XR bridge scripts.
- `pico-teleop/`: Snapshot of the upstream teleoperation framework.
- `XR-PC-Service-Python/`: Snapshot of the Python XR PC Service.
- `pico-PC-Service-pybind/`: Snapshot of the pybind PC Service.
- `vision/`: D455 RGB visual-feedback sender.
- `config/`: Shared configuration.
- `scripts/`: Startup, installation and diagnostic scripts.
- `docs/`: Architecture and operation documentation.

## Control path

PICO 4 Ultra
→ XR PC Service
→ XR input source
→ teleoperation engine
→ Baxter control bridge
→ Baxter SDK

## Planned visual-feedback path

Intel RealSense D455 RGB
→ low-latency H.264 sender
→ PICO Remote Vision

The robot control path and video path remain independent processes.

See `UPSTREAM_VERSIONS.md` for imported upstream versions.
