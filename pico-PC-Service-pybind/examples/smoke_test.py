"""End-to-end smoke test for the pure-Python xrobotoolkit_sdk.

Assumes a service (either the C++ XRoboToolkit-PC-Service or the sibling
xrbt_service Python port) is running on 127.0.0.1:60061 and at least one
PICO headset is connected.

Run:
    python examples/smoke_test.py

Override target:
    XROBOTOOLKIT_GRPC_TARGET=host:port python examples/smoke_test.py
"""
import time

import xrobotoolkit_sdk as xrt


def main() -> None:
    xrt.init()
    print("SDK initialized. Streaming for 5 seconds...\n")
    try:
        t_end = time.time() + 5.0
        while time.time() < t_end:
            print(
                f"left_pose={xrt.get_left_controller_pose()}  "
                f"right_trigger={xrt.get_right_trigger():.2f}  "
                f"A={xrt.get_A_button()}  "
                f"ts_ns={xrt.get_time_stamp_ns()}"
            )
            time.sleep(0.2)
    finally:
        xrt.close()
        print("\nSDK closed.")


if __name__ == "__main__":
    main()
