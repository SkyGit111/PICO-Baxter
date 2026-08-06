"""Unit tests for the TrackingData JSON -> DeviceRecord mapping."""

from __future__ import annotations

import json
import math

from xrbt_service.device_model import DeviceRecord
from xrbt_service.tracking_parser import (
    apply_tracking,
    parse_function_envelope,
)


def _tracking_payload(value: dict) -> str:
    """Mirror Unity's wire shape: {functionName, value=<stringified JSON>}."""
    return json.dumps({"functionName": "Tracking", "value": json.dumps(value)})


def _fresh_record() -> DeviceRecord:
    return DeviceRecord(device_sn="SN-TEST", peer_addr=("10.0.0.1", 55555))


class TestEnvelope:
    def test_parses_stringified_value(self) -> None:
        payload = _tracking_payload({"Head": {"pose": "0,0,0,0,0,0,1"}})
        env = parse_function_envelope(payload)
        assert env is not None
        name, value = env
        assert name == "Tracking"
        assert isinstance(value, dict)
        assert value["Head"]["pose"] == "0,0,0,0,0,0,1"

    def test_returns_none_on_malformed_json(self) -> None:
        assert parse_function_envelope("{not json") is None
        assert parse_function_envelope("[1,2,3]") is None


class TestApplyTracking:
    def test_head_and_controllers(self) -> None:
        payload = _tracking_payload({
            "Head": {"pose": "0.1,0.2,0.3,0,0,0,1", "status": 7},
            "Controller": {
                "left": {
                    "pose": "1,2,3,0,0,0,1",
                    "trigger": 0.25,
                    "grip": 0.75,
                    "axisX": -0.5,
                    "axisY": 0.5,
                    "axisClick": False,
                    "primaryButton": True,
                    "secondaryButton": False,
                    "menuButton": False,
                },
                "right": {
                    "pose": "4,5,6,0,0,0,1",
                    "trigger": 1.0,
                    "grip": 0.0,
                    "axisX": 0.1,
                    "axisY": -0.1,
                    "axisClick": True,
                    "primaryButton": False,
                    "secondaryButton": True,
                    "menuButton": True,
                },
            },
        })

        rec = _fresh_record()
        assert apply_tracking(rec, payload) is True

        assert rec.raw_state_json == payload
        assert rec.head.pose == (0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0)
        assert rec.head.status == 7

        left = rec.left_controller
        assert left.pose == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
        assert math.isclose(left.trigger, 0.25)
        assert math.isclose(left.grip, 0.75)
        assert math.isclose(left.axis_x, -0.5)
        assert math.isclose(left.axis_y, 0.5)
        assert left.axis_click is False
        assert left.primary_button is True
        assert left.secondary_button is False
        assert left.menu_button is False

        right = rec.right_controller
        assert right.pose == (4.0, 5.0, 6.0, 0.0, 0.0, 0.0, 1.0)
        assert right.trigger == 1.0
        assert right.axis_click is True
        assert right.menu_button is True
        assert right.secondary_button is True

    def test_head_only(self) -> None:
        payload = _tracking_payload({"Head": {"pose": "9,8,7,0,0,0,1"}})
        rec = _fresh_record()
        assert apply_tracking(rec, payload) is True
        assert rec.head.pose == (9.0, 8.0, 7.0, 0.0, 0.0, 0.0, 1.0)
        assert rec.left_controller.pose == ()
        assert rec.right_controller.pose == ()

    def test_unknown_function_ignored_but_raw_stored(self) -> None:
        payload = json.dumps({"functionName": "timeTest", "value": ""})
        rec = _fresh_record()
        assert apply_tracking(rec, payload) is False
        assert rec.raw_state_json == payload

    def test_malformed_payload_still_stores_raw(self) -> None:
        rec = _fresh_record()
        assert apply_tracking(rec, "{not json") is False
        assert rec.raw_state_json == "{not json"

    def test_bad_pose_string_falls_back_to_empty(self) -> None:
        payload = _tracking_payload({"Head": {"pose": "not,a,pose"}})
        rec = _fresh_record()
        apply_tracking(rec, payload)
        assert rec.head.pose == ()

    def test_unicode_payload_roundtrips(self) -> None:
        # Unity / nlohmann::json both produce ``\uXXXX``-escaped blobs by
        # default; we must store the payload byte-for-byte so a downstream
        # ``json.loads`` recovers the original code points.
        payload = _tracking_payload({
            "Head": {"pose": "0,0,0,0,0,0,1"},
            "note": "üñîçødé",
        })
        rec = _fresh_record()
        assert apply_tracking(rec, payload) is True
        assert rec.raw_state_json == payload
        outer = json.loads(rec.raw_state_json)
        inner = json.loads(outer["value"])
        assert inner["note"] == "üñîçødé"
