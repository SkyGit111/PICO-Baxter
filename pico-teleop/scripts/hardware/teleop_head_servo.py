"""Compatibility wrapper for the head-servo teleop control script."""

from __future__ import annotations

import os
import sys

import tyro  # type: ignore[import-not-found]

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from control.teleop_head_servo import main


if __name__ == "__main__":
    tyro.cli(main)
