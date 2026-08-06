# xrobotoolkit_sdk (pure Python)

A **pure-Python** client for the XRoboToolkit PC Service. It is a drop-in
replacement for the previous `pybind11` extension module of the same name —
the public API is identical, so existing consumers (e.g. `pico-teleop`'s
`xr_client.py`) work unchanged.

This version has **no C++ build step** and works on any platform where
Python + gRPC run, including Ubuntu 20.04 / arm64 (Nvidia Orin) where the
old `libPXREARobotSDK.so` was the bottleneck.

## How it works

Connects to `EAService` gRPC on `127.0.0.1:60061` (served by either the
original C++ `XRoboToolkit-PC-Service` **or** the sibling pure-Python
`xrbt_service`) and subscribes to `WatchServerFeedback`. A background
daemon thread parses incoming `DeviceStateJson` payloads (same JSON shape
the old C++ bindings consumed) into thread-safe state; all getters return
snapshots under a single `threading.Lock`.

```
PC Service (C++ or Python)   ──gRPC 60061──▶   xrobotoolkit_sdk (this pkg)
```

## Install

```bash
pip install -e .
```

Dependencies: `grpcio`, `protobuf`.

Override the gRPC target with `XROBOTOOLKIT_GRPC_TARGET` (default
`127.0.0.1:60061`).

## Usage

```python
import xrobotoolkit_sdk as xrt

xrt.init()

print(xrt.get_left_controller_pose())    # [x, y, z, qx, qy, qz, qw]
print(xrt.get_right_trigger())           # float in [0, 1]
print(xrt.get_A_button())                # bool
print(xrt.get_time_stamp_ns())           # int

xrt.close()
```

See `examples/` for more.

## Public surface

Matches the old pybind module 1:1:

- Poses: `get_left_controller_pose`, `get_right_controller_pose`, `get_headset_pose`
- Inputs: `get_{left,right}_{trigger,grip,menu_button,axis,axis_click}`, `get_{A,B,X,Y}_button`
- Time: `get_time_stamp_ns`
- Hand: `get_{left,right}_hand_tracking_state`, `get_{left,right}_hand_is_active`
- Body: `is_body_data_available`, `get_body_joints_{pose,velocity,acceleration,timestamp}`, `get_body_timestamp_ns`
- Motion trackers: `num_motion_data_available`, `get_motion_tracker_{pose,velocity,acceleration,serial_numbers}`, `get_motion_timestamp_ns`
- Outbound: `device_control_json`, `send_bytes_to_device`

## Regenerating the gRPC stubs

The generated stubs are vendored under `python/xrobotoolkit_sdk/_generated/`.
To regenerate them from the proto in the sibling repo:

```bash
python -m grpc_tools.protoc \
    -I../XRoboToolkit-PC-Service-Python/proto \
    --python_out=python/xrobotoolkit_sdk/_generated \
    --grpc_python_out=python/xrobotoolkit_sdk/_generated \
    ../XRoboToolkit-PC-Service-Python/proto/PXREAService.proto

# patch the grpc stub's import back to relative:
sed -i 's/^import PXREAService_pb2 as/from . import PXREAService_pb2 as/' \
    python/xrobotoolkit_sdk/_generated/PXREAService_pb2_grpc.py
```
