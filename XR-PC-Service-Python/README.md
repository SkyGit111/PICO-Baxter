# XRoboToolkit-PC-Service-Python

A pure-Python, cross-platform replacement for the Ubuntu-only C++/Qt6
`XRoboToolkit-PC-Service`.

It speaks the same wire protocols in both directions, so the existing
`xrobotoolkit_sdk` pip package (used by `XRoboToolkit-Teleop-Sample-Python`)
works unchanged:

- **Upstream** — TCP server on `0.0.0.0:63901`, PICO/Unity framed protocol.
- **Downstream** — gRPC (`EAService`) on `127.0.0.1:60061`, same `.proto`,
  same `ServerFeedback{name="deviceStateJson", devicestatejson.statejson=<raw JSON>}`
  shape the C++ service emits.

v1 scope: headset pose + controller pose / buttons / triggers / grips / axes.
Hand tracking, body tracking, motion trackers, and robot-camera → VR video
streaming are explicitly designed-for but not implemented — see
`serialized-strolling-fairy.md` for the full plan.

## Install

```bash
pip install -e .[dev]
```

The generated gRPC stubs under `xrbt_service/generated/` are committed.
Regenerate them with:

```bash
python -m grpc_tools.protoc -Iproto \
    --python_out=xrbt_service/generated \
    --grpc_python_out=xrbt_service/generated \
    proto/PXREAService.proto
# patch the sibling import back to relative:
sed -i 's/^import PXREAService_pb2 as/from . import PXREAService_pb2 as/' \
    xrbt_service/generated/PXREAService_pb2_grpc.py
```

## Run

```bash
python -m xrbt_service.main
```

By default:
- upstream bind: `0.0.0.0:63901`
- downstream bind: `127.0.0.1:60061`

Override with env vars (`XRBT_UPSTREAM_HOST`, `XRBT_UPSTREAM_PORT`,
`XRBT_DOWNSTREAM_HOST`, `XRBT_DOWNSTREAM_PORT`) or with a `setting.ini`
in the working directory (same keys as the C++ service).

## Tests

```bash
pytest
```

## Architecture

```
   Unity TCP 63901  <─┐                ┌─> gRPC 60061 <─>  xrobotoolkit_sdk
                     upstream.py      downstream.py
                     frame_codec.py      │
                           │             │
                           ▼             │
                       device_model.py  <┘
                     (shared state + change events + event bus hooks)
```

The split between control plane (this service) and media plane
(`xrbt_service/media/`, reserved-but-empty in v1) is deliberate: see the
plan doc for why video must not flow through gRPC.
