# XRoboToolkit RealMan Teleop

PICO teleoperation runtime for RealMan hardware.

## Runtime Flow

```text
PICO headset
  -> XRoboToolkit-PC-Service-Python
  -> xrobotoolkit_sdk / XrClient
  -> SdkXRInputSource
  -> TeleopEngine
  -> RobotAdapter
  -> RealManRobotAdapter
  -> RealMan hardware
```

The repository has been trimmed to the RealMan production path. Simulation, UR, ARX, Galaxea, Placo, MuJoCo, Meshcat, visualization scripts, and their robot assets have been removed from this package.

## What Remains

- `scripts/hardware/teleop_realman_hardware.py`: RealMan launcher.
- `scripts/misc/test_xr_input_source.py`: XR-only live input check.
- `xrobotoolkit_teleop/core/`: shared teleop types, XR input, engine, and adapter API.
- `xrobotoolkit_teleop/hardware/realman_robot_adapter.py`: RealMan FK/IK, command streaming, gripper control, and dry-run mode.
- `xrobotoolkit_teleop/hardware/interface/realman.py`: low-level RealMan arm wrapper.
- `control/`: RealMan controller, RealMan Python SDK wrapper, and gripper TCP/UDP clients.

## Install

Install the XR SDK binding separately, or use the setup script:

```bash
bash setup_conda.sh --conda xr-robotics
conda activate xr-robotics
bash setup_conda.sh --install
```

The setup script installs `XRoboToolkit-PC-Service-Pybind` and this package. RealMan SDK libraries still need to be available for the target platform.

## Run

Start `XRoboToolkit-PC-Service-Python` separately and connect the PICO headset.

Check live XR input:

```bash
python scripts/misc/test_xr_input_source.py
```

Run RealMan dry-run:

```bash
python scripts/hardware/teleop_realman_hardware.py --dry-run
```

Run live RealMan hardware:

```bash
python scripts/hardware/teleop_realman_hardware.py
```

Custom network configuration:

```bash
python scripts/hardware/teleop_realman_hardware.py \
  --left-arm-ip 169.254.128.18 \
  --right-arm-ip 169.254.128.19 \
  --arm-port 8080 \
  --local-ip 169.254.128.20 \
  --scale-factor 2.0 \
  --control-rate-hz 50
```

## Controls

| Input | Action |
| --- | --- |
| Left grip | Hold to command left arm pose |
| Right grip | Hold to command right arm pose |
| Left trigger | Command left gripper, 0=open to 1=closed |
| Right trigger | Command right gripper, 0=open to 1=closed |

Releasing grip clears the pose reference and stops arm pose command updates.

## License

MIT. See [LICENSE](LICENSE).
