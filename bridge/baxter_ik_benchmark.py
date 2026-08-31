#!/usr/bin/env python3
"""只读比较 Baxter ROS IK Service 与本地 baxter_pykdl。

本工具只读取当前关节和末端状态并执行运动学计算，绝不发送关节命令，也不会自动
enable Baxter。目标是在机器人主机上获得可复现的 solver 延迟、成功率和分支差异。
"""

import argparse
import json
import math
import statistics
import time
from typing import Any, Dict, List, Optional, Sequence


JointDict = Dict[str, float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument(
        "--offset",
        type=float,
        default=0.02,
        help="围绕当前末端位置的轴向测试偏移，单位为米。",
    )
    parser.add_argument(
        "--repeats",
        type=int,
        default=5,
        help="每个目标对每种后端重复求解的次数。",
    )
    parser.add_argument(
        "--json-output",
        default="",
        help="可选 JSON 结果文件；为空时只打印。",
    )
    args = parser.parse_args()
    if args.offset < 0.0 or args.offset > 0.10:
        parser.error("--offset 必须在 [0, 0.10] 米内")
    if args.repeats < 1 or args.repeats > 100:
        parser.error("--repeats 必须在 [1, 100] 内")
    return args


def percentile(values: Sequence[float], fraction: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    index = int(math.ceil(fraction * len(ordered))) - 1
    return ordered[max(0, min(index, len(ordered) - 1))]


def maximum_joint_difference(a: JointDict, b: JointDict) -> Optional[float]:
    if not a or set(a) != set(b):
        return None
    return max(abs(a[name] - b[name]) for name in a)


def summarize(records: List[Dict[str, Any]]) -> Dict[str, Any]:
    elapsed = [record["elapsed_ms"] for record in records]
    successful = [record for record in records if record["success"]]
    branch = [
        record["seed_difference_rad"]
        for record in successful
        if record["seed_difference_rad"] is not None
    ]
    return {
        "calls": len(records),
        "successes": len(successful),
        "success_rate": len(successful) / len(records) if records else 0.0,
        "time_ms": {
            "median": statistics.median(elapsed) if elapsed else None,
            "p95": percentile(elapsed, 0.95),
            "p99": percentile(elapsed, 0.99),
            "max": max(elapsed) if elapsed else None,
        },
        "seed_difference_rad": {
            "median": statistics.median(branch) if branch else None,
            "p95": percentile(branch, 0.95),
            "max": max(branch) if branch else None,
        },
    }


def target_positions(origin: Sequence[float], offset: float) -> List[List[float]]:
    targets = [list(origin)]
    for axis in range(3):
        for sign in (-1.0, 1.0):
            target = list(origin)
            target[axis] += sign * offset
            targets.append(target)
    return targets


def print_summary(name: str, summary: Dict[str, Any]) -> None:
    timing = summary["time_ms"]
    branch = summary["seed_difference_rad"]
    print("")
    print(name)
    print(
        "  success: {}/{} ({:.1%})".format(
            summary["successes"], summary["calls"], summary["success_rate"]
        )
    )
    print(
        "  time ms: median={} p95={} p99={} max={}".format(
            timing["median"], timing["p95"], timing["p99"], timing["max"]
        )
    )
    print(
        "  seed diff rad: median={} p95={} max={}".format(
            branch["median"], branch["p95"], branch["max"]
        )
    )


def main() -> int:
    args = parse_args()

    # 延迟导入，使 --help、py_compile 和纯 Python helper 测试不要求 ROS 环境。
    try:
        import baxter_interface
        import rospy
        from baxter_core_msgs.srv import SolvePositionIK
        from baxter_pykdl import baxter_kinematics
    except ImportError as exc:
        raise SystemExit(
            "缺少 Baxter/ROS 或 baxter_pykdl Python 依赖：{}".format(exc)
        ) from exc

    from baxter_precision_teleop import (
        copy_quaternion,
        create_target_pose,
        endpoint_position,
        solve_ik,
    )

    rospy.init_node("baxter_ik_benchmark", anonymous=True)
    limb = baxter_interface.Limb(args.side)
    kinematics = baxter_kinematics(args.side)
    service_name = (
        "/ExternalTools/{}/PositionKinematicsNode/IKService"
    ).format(args.side)
    rospy.wait_for_service(service_name, timeout=10.0)
    service = rospy.ServiceProxy(service_name, SolvePositionIK)

    endpoint = limb.endpoint_pose()
    origin = endpoint_position(endpoint)
    orientation = copy_quaternion(endpoint["orientation"])
    seed = {name: float(value) for name, value in limb.joint_angles().items()}
    joint_names = list(limb.joint_names())
    seed_vector = [seed[name] for name in joint_names]

    records: Dict[str, List[Dict[str, Any]]] = {
        "ros_service": [],
        "baxter_pykdl": [],
    }
    targets = target_positions(origin, args.offset)

    print("只读 IK 基准：不会发送任何机器人命令")
    print(
        "Arm: {}  targets: {}  repeats: {}".format(
            args.side, len(targets), args.repeats
        )
    )

    for target_index, position in enumerate(targets):
        target_pose = create_target_pose(position, orientation)
        for repeat in range(args.repeats):
            started = time.monotonic()
            try:
                valid, solution, code = solve_ik(service, target_pose, seed)
                error = None
            except Exception as exc:
                valid, solution, code = False, {}, 0
                error = "{}: {}".format(type(exc).__name__, exc)
            elapsed_ms = (time.monotonic() - started) * 1000.0
            records["ros_service"].append(
                {
                    "target": target_index,
                    "repeat": repeat,
                    "elapsed_ms": elapsed_ms,
                    "success": bool(valid),
                    "result_code": code,
                    "seed_difference_rad": maximum_joint_difference(
                        seed, solution
                    ),
                    "error": error,
                }
            )

            started = time.monotonic()
            try:
                local_values = kinematics.inverse_kinematics(
                    position,
                    [orientation.x, orientation.y, orientation.z, orientation.w],
                    seed_vector,
                )
                local_solution = (
                    {}
                    if local_values is None
                    else {
                        name: float(value)
                        for name, value in zip(joint_names, local_values)
                    }
                )
                local_valid = bool(local_solution)
                local_error = None
            except Exception as exc:
                local_solution = {}
                local_valid = False
                local_error = "{}: {}".format(type(exc).__name__, exc)
            elapsed_ms = (time.monotonic() - started) * 1000.0
            records["baxter_pykdl"].append(
                {
                    "target": target_index,
                    "repeat": repeat,
                    "elapsed_ms": elapsed_ms,
                    "success": local_valid,
                    "result_code": None,
                    "seed_difference_rad": maximum_joint_difference(
                        seed, local_solution
                    ),
                    "error": local_error,
                }
            )

    report = {
        "side": args.side,
        "origin": origin,
        "offset_m": args.offset,
        "repeats": args.repeats,
        "targets": targets,
        "summary": {
            name: summarize(backend_records)
            for name, backend_records in records.items()
        },
        "records": records,
        "hardware_motion_commanded": False,
    }

    for name, summary in report["summary"].items():
        print_summary(name, summary)

    if args.json_output:
        with open(args.json_output, "w", encoding="utf-8") as output:
            json.dump(report, output, ensure_ascii=False, indent=2)
        print("")
        print("JSON: {}".format(args.json_output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
