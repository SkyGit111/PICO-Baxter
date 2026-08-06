"""gRPC client + getter surface for xrobotoolkit_sdk.

Connects to EAService on ``127.0.0.1:60061`` (override via
``XROBOTOOLKIT_GRPC_TARGET`` env var), subscribes to
``WatchServerFeedback``, and fans out each ``DeviceStateJson`` update
into the shared ``_STATE`` singleton.

All getters mirror ``bindings/py_bindings.cpp`` exactly.
"""

from __future__ import annotations

import logging
import os
import threading
from typing import List, Tuple

import grpc

from ._generated import PXREAService_pb2 as pb
from ._generated import PXREAService_pb2_grpc as pb_grpc
from ._parser import apply_state_json
from ._state import get_state, reset_state

logger = logging.getLogger(__name__)

DEFAULT_TARGET = "127.0.0.1:60061"


class _Client:
    def __init__(self) -> None:
        self._channel: grpc.Channel | None = None
        self._stub: pb_grpc.EAServiceStub | None = None
        self._stream = None  # type: ignore[assignment]
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._started = False

    def start(self, target: str) -> None:
        if self._started:
            return
        self._stop.clear()
        self._channel = grpc.insecure_channel(target)
        self._stub = pb_grpc.EAServiceStub(self._channel)

        self._thread = threading.Thread(
            target=self._run_loop,
            name="xrobotoolkit_sdk.subscriber",
            daemon=True,
        )
        self._thread.start()
        self._started = True
        logger.info("xrobotoolkit_sdk connected to %s", target)

    def stop(self) -> None:
        if not self._started:
            return
        self._stop.set()
        try:
            if self._stream is not None:
                self._stream.cancel()
        except Exception:
            pass
        if self._channel is not None:
            try:
                self._channel.close()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None
        self._stream = None
        self._stub = None
        self._channel = None
        self._started = False
        reset_state()
        logger.info("xrobotoolkit_sdk closed")

    def _run_loop(self) -> None:
        state = get_state()
        pid = os.getpid()
        while not self._stop.is_set():
            try:
                assert self._stub is not None
                self._stream = self._stub.WatchServerFeedback(pb.VRPid(pid=pid))
                for feedback in self._stream:
                    if self._stop.is_set():
                        break
                    if feedback.name != "deviceStateJson":
                        continue
                    raw = feedback.devicestatejson.statejson
                    if not raw:
                        continue
                    try:
                        apply_state_json(state, raw)
                    except Exception:
                        logger.exception("failed to parse deviceStateJson")
            except grpc.RpcError as exc:
                if self._stop.is_set():
                    return
                logger.warning("xrobotoolkit_sdk gRPC error: %s; reconnecting", exc)
                # brief backoff before reconnect
                if self._stop.wait(0.5):
                    return
            except Exception:
                if self._stop.is_set():
                    return
                logger.exception("xrobotoolkit_sdk subscriber crashed; reconnecting")
                if self._stop.wait(1.0):
                    return

    def send_control_json(self, dev_id: str, json_str: str) -> int:
        if not self._started or self._stub is None:
            raise RuntimeError("xrobotoolkit_sdk not initialized; call init() first")
        self._stub.DeviceControlJson(
            pb.DeviceControlParameterJson(devid=dev_id, parameter=json_str)
        )
        return 0

    def send_bytes(self, dev_id: str, blob: bytes) -> int:
        if not self._started or self._stub is None:
            raise RuntimeError("xrobotoolkit_sdk not initialized; call init() first")
        if isinstance(blob, str):
            blob = blob.encode("utf-8")
        self._stub.SendBytesToDevice(
            pb.DeviceBytesInfo(devid=dev_id, content=bytes(blob))
        )
        return 0


_CLIENT = _Client()


# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------


def init() -> None:
    """Initialize the SDK: connect to the EAService gRPC stream."""
    target = os.environ.get("XROBOTOOLKIT_GRPC_TARGET", DEFAULT_TARGET)
    _CLIENT.start(target)


def close() -> None:
    """Deinitialize: cancel the stream and close the gRPC channel."""
    _CLIENT.stop()


# ---------------------------------------------------------------------------
# Pose getters
# ---------------------------------------------------------------------------


def _pose_as_list(p) -> List[float]:
    return list(p)


def get_left_controller_pose() -> List[float]:
    s = get_state()
    with s.lock:
        return _pose_as_list(s.left_controller_pose)


def get_right_controller_pose() -> List[float]:
    s = get_state()
    with s.lock:
        return _pose_as_list(s.right_controller_pose)


def get_headset_pose() -> List[float]:
    s = get_state()
    with s.lock:
        return _pose_as_list(s.headset_pose)


# ---------------------------------------------------------------------------
# Triggers / grips / axes / buttons
# ---------------------------------------------------------------------------


def get_left_trigger() -> float:
    s = get_state()
    with s.lock:
        return s.left_trigger


def get_left_grip() -> float:
    s = get_state()
    with s.lock:
        return s.left_grip


def get_right_trigger() -> float:
    s = get_state()
    with s.lock:
        return s.right_trigger


def get_right_grip() -> float:
    s = get_state()
    with s.lock:
        return s.right_grip


def get_left_menu_button() -> bool:
    s = get_state()
    with s.lock:
        return s.left_menu_button


def get_right_menu_button() -> bool:
    s = get_state()
    with s.lock:
        return s.right_menu_button


def get_left_axis_click() -> bool:
    s = get_state()
    with s.lock:
        return s.left_axis_click


def get_right_axis_click() -> bool:
    s = get_state()
    with s.lock:
        return s.right_axis_click


def get_left_axis() -> List[float]:
    s = get_state()
    with s.lock:
        return list(s.left_axis)


def get_right_axis() -> List[float]:
    s = get_state()
    with s.lock:
        return list(s.right_axis)


def get_X_button() -> bool:  # noqa: N802 — matches Pybind
    s = get_state()
    with s.lock:
        return s.left_primary_button


def get_A_button() -> bool:  # noqa: N802
    s = get_state()
    with s.lock:
        return s.right_primary_button


def get_Y_button() -> bool:  # noqa: N802
    s = get_state()
    with s.lock:
        return s.left_secondary_button


def get_B_button() -> bool:  # noqa: N802
    s = get_state()
    with s.lock:
        return s.right_secondary_button


def get_time_stamp_ns() -> int:
    s = get_state()
    with s.lock:
        return s.timestamp_ns


# ---------------------------------------------------------------------------
# Hand tracking
# ---------------------------------------------------------------------------


def get_left_hand_tracking_state() -> List[List[float]]:
    s = get_state()
    with s.lock:
        return [list(p) for p in s.left_hand_tracking]


def get_right_hand_tracking_state() -> List[List[float]]:
    s = get_state()
    with s.lock:
        return [list(p) for p in s.right_hand_tracking]


def get_left_hand_is_active() -> int:
    s = get_state()
    with s.lock:
        return s.left_hand_is_active


def get_right_hand_is_active() -> int:
    s = get_state()
    with s.lock:
        return s.right_hand_is_active


# ---------------------------------------------------------------------------
# Body tracking
# ---------------------------------------------------------------------------


def is_body_data_available() -> bool:
    s = get_state()
    with s.lock:
        return s.body_data_available


def get_body_joints_pose() -> List[List[float]]:
    s = get_state()
    with s.lock:
        return [list(p) for p in s.body_joints_pose]


def get_body_joints_velocity() -> List[List[float]]:
    s = get_state()
    with s.lock:
        return [list(v) for v in s.body_joints_velocity]


def get_body_joints_acceleration() -> List[List[float]]:
    s = get_state()
    with s.lock:
        return [list(a) for a in s.body_joints_acceleration]


def get_body_joints_timestamp() -> List[int]:
    s = get_state()
    with s.lock:
        return list(s.body_joints_timestamp)


def get_body_timestamp_ns() -> int:
    s = get_state()
    with s.lock:
        return s.body_timestamp_ns


# ---------------------------------------------------------------------------
# Motion trackers
# ---------------------------------------------------------------------------


def num_motion_data_available() -> int:
    s = get_state()
    with s.lock:
        return s.num_motion_data_available


def get_motion_tracker_pose() -> List[List[float]]:
    s = get_state()
    with s.lock:
        return [list(p) for p in s.motion_tracker_pose]


def get_motion_tracker_velocity() -> List[List[float]]:
    s = get_state()
    with s.lock:
        return [list(v) for v in s.motion_tracker_velocity]


def get_motion_tracker_acceleration() -> List[List[float]]:
    s = get_state()
    with s.lock:
        return [list(a) for a in s.motion_tracker_acceleration]


def get_motion_tracker_serial_numbers() -> List[str]:
    s = get_state()
    with s.lock:
        return list(s.motion_tracker_serial_numbers)


def get_motion_timestamp_ns() -> int:
    s = get_state()
    with s.lock:
        return s.motion_timestamp_ns


# ---------------------------------------------------------------------------
# Outbound control
# ---------------------------------------------------------------------------


def device_control_json(dev_id: str, json_str: str) -> int:
    return _CLIENT.send_control_json(dev_id, json_str)


def send_bytes_to_device(dev_id: str, blob: bytes) -> int:
    return _CLIENT.send_bytes(dev_id, blob)
