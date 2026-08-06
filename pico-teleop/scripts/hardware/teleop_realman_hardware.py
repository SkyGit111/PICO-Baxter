"""RealMan hardware teleop entry point.

Run XRoboToolkit-PC-Service-Python separately before starting this script.

When head camera + remote stream are enabled, the same RealSense device used by
the adapter also serves H264 to the Pico Unity client (same flow as
``test_realsense_stream.py``): Camera panel -> Listen -> ``<this PC IP>:port``.
"""

import logging

import tyro

from xrobotoolkit_teleop.core.teleop_engine import TeleopEngine
from xrobotoolkit_teleop.core.xr_input import SdkXRInputSource
from xrobotoolkit_teleop.hardware.frame_sender import (
    DEFAULT_LISTEN_PORT,
    FrameSender,
)
from xrobotoolkit_teleop.hardware.realman import RealmanConfig, build_realman


def main(
    left_arm_ip: str = RealmanConfig.left_arm_ip,
    right_arm_ip: str = RealmanConfig.right_arm_ip,
    arm_port: int = RealmanConfig.arm_port,
    local_ip: str = RealmanConfig.local_ip,
    scale_factor: float = 1.0,
    arm_rate_hz: float = 50.0,
    head_rate_hz: float = 25.0,
    xr_poll_hz: float = 100.0,
    dry_run: bool = False,
    head_servo_enabled: bool = True,
    head_camera_enabled: bool = True,
    remote_camera_stream: bool = True,
    camera_listen_host: str = "0.0.0.0",
    camera_listen_port: int = DEFAULT_LISTEN_PORT,
    camera_bitrate_mbps: float = 10.0,
    base_enabled: bool = True,
    base_cmd_topic: str = "/base_cmd_vel",
    base_max_linear_mps: float = 0.12,
    base_max_angular_radps: float = 0.35,
    lift_enabled: bool = True,
    lift_joystick_deadzone: float = 0.15,
    lift_joystick_invert: bool = False,
    lift_speed_pct: int = 50,
):
    """Launch RealMan teleoperation."""
    config = RealmanConfig(
        left_arm_ip=left_arm_ip,
        right_arm_ip=right_arm_ip,
        arm_port=arm_port,
        local_ip=local_ip,
        arm_rate_hz=arm_rate_hz,
        head_rate_hz=head_rate_hz,
        dry_run=dry_run,
        head_servo_enabled=head_servo_enabled,
        head_camera_enabled=head_camera_enabled,
        base_enabled=base_enabled,
        base_cmd_topic=base_cmd_topic,
        base_max_linear_mps=base_max_linear_mps,
        base_max_angular_radps=base_max_angular_radps,
        lift_enabled=lift_enabled,
        lift_joystick_deadzone=lift_joystick_deadzone,
        lift_joystick_invert=lift_joystick_invert,
        lift_speed_pct=lift_speed_pct,
    )
    config.mapping.scale_factor = scale_factor

    print("=" * 60)
    print("RealMan Robot Teleoperation")
    print("=" * 60)
    print("XR input:   XRoboToolkit SDK / XrClient")
    print("Robot:      RealmanAdapter")
    print(f"Left arm:   {left_arm_ip}:{arm_port}")
    print(f"Right arm:  {right_arm_ip}:{arm_port}")
    print(f"Local IP:   {local_ip}")
    print(f"Scale:      {scale_factor}")
    print(f"Arm rate:   {arm_rate_hz} Hz")
    print(f"Head rate:  {head_rate_hz} Hz")
    print(f"XR poll:    {xr_poll_hz} Hz")
    print(f"Dry run:    {'Enabled' if dry_run else 'Disabled'}")
    print(f"Head servo: {'Enabled' if head_servo_enabled else 'Disabled'}")
    print(f"Head cam:   {'Enabled' if head_camera_enabled else 'Disabled'}")
    print(
        f"Chassis:    {'Enabled (' + base_cmd_topic + ', max v=' + str(base_max_linear_mps) + ' m/s, max w=' + str(base_max_angular_radps) + ' rad/s)' if base_enabled else 'Disabled'}"
    )
    if lift_enabled:
        lift_desc = (
            f"Enabled (right stick Y; dz={lift_joystick_deadzone}, "
            f"invert={lift_joystick_invert}, ±{lift_speed_pct}% constant outside dz)"
        )
    else:
        lift_desc = "Disabled"
    print(f"Lift:       {lift_desc}")
    stream_active = head_camera_enabled and remote_camera_stream
    print(
        f"Remote cam: {'Enabled (Unity Listen -> <PC LAN IP>:' + str(camera_listen_port) + ')' if stream_active else 'Disabled'}"
    )
    print("=" * 60)
    print()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    xr_input = SdkXRInputSource()
    adapter = build_realman(config)
    engine = TeleopEngine(xr_input=xr_input, adapter=adapter, xr_poll_hz=xr_poll_hz)

    sender: FrameSender | None = None
    try:
        adapter.connect()
        if stream_active:
            cam = adapter.cameras().get("head")
            if cam is None:
                print(
                    "Warning: remote camera stream requested but head camera did not start; "
                    "fix USB/camera errors above or disable --head-camera-enabled."
                )
            else:
                sender = FrameSender(
                    camera=cam,
                    listen_host=camera_listen_host,
                    listen_port=camera_listen_port,
                    bitrate_bps=int(camera_bitrate_mbps * 1024 * 1024),
                )
                sender.start()
                print(
                    f"Headset video: listening {camera_listen_host}:{camera_listen_port} "
                    f"(Unity Listen: use this PC's LAN IP reachable from the Pico, same port)."
                )
        engine.run_forever()
    except KeyboardInterrupt:
        print("\nTeleoperation interrupted by user.")
    finally:
        if sender is not None:
            print("Stopping remote camera stream...")
            sender.stop()
            sender = None
        xr_input.close()
        adapter.shutdown()


if __name__ == "__main__":
    tyro.cli(main)
