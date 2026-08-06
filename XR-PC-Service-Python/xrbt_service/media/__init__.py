"""Media plane (v2).

Deliberately empty in v1. See ``serialized-strolling-fairy.md`` for the
full design. The namespace is reserved so a v2 drop-in can add:

- ``publisher.py`` — robot-camera frames -> network (RTSP / WebRTC / UDP)
- ``subscriber.py`` — PICO-camera frames -> local consumers
- ``signaling.py`` — negotiation over the existing control plane
- ``codecs.py`` — H.264 encode/decode via PyAV

Landing order does not require editing any v1 file because:

- ``DeviceRegistry.peer_addr_of(sn)`` already exposes where to send video.
- ``DeviceRegistry.on_device_connected / on_device_disconnected`` fire the
  lifecycle hooks a publisher needs.
- ``UpstreamServer.handlers`` is a mutable dict — register additional
  CMD bytes (``OPEN_CAMERA``, ``CLOSE_CAMERA``, ``0x72``) from here.
- ``main.py`` appends ``media.publisher.serve()`` to its ``gather`` list.
"""
